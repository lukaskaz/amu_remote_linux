/*
	Copyright 2001, 2002 Georges Menie (www.menie.org)
	stdarg version contributed by Christian Ettinger

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
	putchar is the only external dependency for this file,
	if you have a working putchar, leave it commented out.
	If not, uncomment the define below and
	replace outbyte(c) by your own function call.

#define putchar(c) outbyte(c)
*/

#include <stdarg.h>
#include <stdlib.h>     //NULL pointer definition

//static void printchar_old(char **str, int c)
//{
//	extern int putchar(int c);
//
//	if (str) {
//		**str = c;
//		++(*str);
//	}
//	else (void)putchar(c);
//}

static int printchar(char** str, const char* c, size_t size)
{
	extern int putchar(int c);
	int i = 0;

	for(i=0; i < size && c[i] != '\0'; i++) {
		if (str) {
			**str = c[i];
			++(*str);
		}
		else {
			(void)putchar(c[i]);
		}
	}

	return i;
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0;
	char padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}

	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			pc += printchar (out, &padchar, 1);
		}
	}

	pc += printchar(out, string, 0xFFFF);
	for ( ; width > 0; --width) {
		pc += printchar (out, &padchar, 1);
	}

	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINTI_BUF_LEN 12

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
    char print_buf[PRINTI_BUF_LEN];
    register char *s;
    register int t, neg = 0, pc = 0;
    register unsigned int u = i;
    char negchar = '-';

    if (i == 0) {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return prints (out, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0) {
        neg = 1;
        u = -i;
    }

    s = print_buf + PRINTI_BUF_LEN-1;
    *s = '\0';

    while (u) {
        t = u % b;
        if( t >= 10 )
            t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;
    }

    if (neg) {
        if( width && (pad & PAD_ZERO) ) {
            printchar (out, &negchar, 1);
            ++pc;
            --width;
        }
        else {
            *--s = '-';
        }
    }

    return pc + prints (out, s, width, pad);
}

//static int printfl(char **out, double f, int b, int sg, int width, int pad, int letbase, int prec)
//{
//    int pc = 0;
//    char separator = '.';
//    int intgr = f;
//    char buff[20] = {0};
//
//    if(prec == 0) {
//        prec = 1;
//    }
//    else if(prec == 1) {
//        prec = 10;
//    }
//    else if (prec == 2) {
//        prec = 100;
//    }
//    else if (prec == 3) {
//        prec = 1000;
//    }
//    else if (prec == 4) {
//        prec = 10000;
//    }
//    else if (prec == 5) {
//        prec = 100000;
//    }
//    else {
//        prec = 10;
//    }
//
//
//    if(prec) {
//        int fract = (int)(f*prec);
//
//        if(fract < 0) {
//            if(intgr == 0) {
//                pc += printchar (out, "-", 1);
//            }
//
//            fract *= (-1);
//        }
//        fract %=  prec;
//
//        pc += printi (out, intgr, b, sg, width, pad, letbase);
//        pc += printchar (out, &separator, 1);
//        pc += printi (out, fract, b, sg, width, pad, letbase);
//    }
//    else {
//        pc += printi (out, intgr, b, sg, width, pad, letbase);
//    }
//
//    return pc;
//}

#define PRINTFL_BUF_LEN 20
static int printfl(char **out, double f, int prec)
{
    int pc = 0, value = 0, base = 1, i = 0;
    char sign = '\0';
    char buff[PRINTFL_BUF_LEN+1] = {0};

    for(i = 0; i < prec; i++) {
        base *= 10;
    }

    value = f * base;
    if(value < 0) {
        sign = '-';
        value *= (-1);
    }

    i = sizeof(buff)-1;
    for(i -= 1; i > 0; i--, prec--) {
        if(prec == 0) {
            buff[i] = '.';
        }
        else {
            buff[i] = '0' + (value % 10);
            value /= 10;

            if(value == 0 && prec < 0) {
                break;
            }
        }
    }

    if(i > 0) {
        if(sign) {
            buff[--i] = sign;
        }

        pc += printchar (out, &buff[i], PRINTFL_BUF_LEN);
    }

    return pc;
}

static int print( char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == '.') {
			    char prec = *(++format) - '0';

			    format++;
			    if(*format == 'f') {
	                pc += printfl (out, va_arg( args, double ), prec);
	                continue;
			    }
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, int );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'f' ) {
			    pc += printfl (out, va_arg( args, double ), 1);
			    continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == '#') {
				pc += printchar (out, "0x", 2);

				format++;
				if(*format == 'x') {
					pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				}

				if(*format == 'X') {
					pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
				}

				continue;
			}
			if( *format == 'p') {
				int arg = va_arg(args, int);

				if(arg == 0x00) {
					pc += printchar (out, "(nil)", 5);
				}
				else {
					pc += printchar (out, "0x", 2);
					pc += printi (out, arg, 16, 0, width, pad, 'a');
				}

				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			pc += printchar (out, format, 1);
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

static int scan(const char* str, const char* format, va_list ap)
{
	//va_list ap;
	int value, tmp, pos, count = 0;
	char neg, fmt_code,* sval;
	//va_start(ap, format);

	for (count = 0; *format != 0 && *str != 0; format++, str++) {
		while (*format == ' ' && *format != 0) format++;//
		if (*format == 0) break;

		while (*str == ' ' && *str != 0) str++;//increment pointer of input string
		if (*str == 0) break;

		if (*format == '%')//recognize how to format
		{
			format++;
			if (*format == 'n')
			{
                		if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))//if in str sth like 0xff
                		{
                    			fmt_code = 'x';
                    			str += 2;
                		}
                		else if (str[0] == 'b')
               			{
                    			fmt_code = 'b';
                    			str++;
                		}
               			else
                    			fmt_code = 'd';
				}
			else
				fmt_code = *format; //it is format letter

			switch (fmt_code)
			{
			case 'x':
			case 'X':
				for (value = 0, pos = 0; *str != 0; str++, pos++)
				{
					if ('0' <= *str && *str <= '9')
						tmp = *str - '0';
					else if ('a' <= *str && *str <= 'f')
						tmp = *str - 'a' + 10;
					else if ('A' <= *str && *str <= 'F')
						tmp = *str - 'A' + 10;
					else
						break;

					value *= 16;
					value += tmp;
				}
				if (pos == 0) {
					return count;
				}

				*(va_arg(ap, int*)) = value;
				count++;
				break;

            		case 'b':
				for (value = 0, pos = 0; *str != 0; str++, pos++)
				{
					if (*str != '0' && *str != '1')
                        			break;
					value *= 2;
					value += *str - '0';
				}
				if (pos == 0) {
					return count;
				}

				*(va_arg(ap, int*)) = value;
				count++;
				break;

			case 'd':
				if (*str == '-')
				{
					neg = 1;
					str++;
				}
				else
					neg = 0;

				for (value = 0, pos = 0; *str != 0; str++, pos++)
				{
					if ('0' <= *str && *str <= '9')
						value = value*10 + (int)(*str - '0');
					else
						break;
				}

				if (pos == 0) {
					return count;
				}

				*(va_arg(ap, int*)) = neg ? -value : value;
				count++;
				break;

			case 'c':
				*(va_arg(ap, char*)) = *str;
				count++;
				break;
			case 's':
				sval = va_arg(ap, char*);


				while(*str){
					*sval++ = *str++;
					count++;
				}
				*sval = '\0';

				break;

			default:
				return count;
			}
		}
		else
		{
			if (*format != *str) {
				break;
			}
		}
	}

	return count;
}

int scanf(const char* format, ...)
{
	extern int putchar(int c);
	extern int getchar(void);

	char ch = 0;
	char buffer[10] = {0};
	int count = 0;
	int i = 0;

	va_list args;
	va_start( args, format );

	for(i=0; i < (sizeof(buffer)-1); i++) {
		ch = getchar();

		if(ch == 0x08) {
			if(i > 0) {
				putchar(ch);
				putchar(0x20);
				putchar(ch);
			}

			i = (i > 0) ? (i - 2) : (i - 1);
		}
		else if(ch != '\n' && ch != '\r') {
			putchar(ch);
			buffer[i] = ch;
		}
		else {
			break;
		}
	}
	buffer[i] = '\0';

	count =  scan(buffer, format, args);
	va_end(args);
	return count;
}

int printf(const char *format, ...)
{
        va_list args;
        
        va_start( args, format );
        return print( 0, format, args );
}

int sprintf(char *out, const char *format, ...)
{
        va_list args;
        
        va_start( args, format );
        return print( &out, format, args );
}

int snprintf( char *buf, unsigned int count, const char *format, ... )
{
        va_list args;
        
        ( void ) count;

        va_start( args, format );
        return print( &buf, format, args );
}


#ifdef TEST_PRINTF
int main(void)
{
	char *ptr = "Hello world!";
	char *np = 0;
	int i = 5;
	unsigned int bs = sizeof(int)*8;
	int mi;
	char buf[80];

	mi = (1 << (bs-1)) + 1;
	printf("%s\n", ptr);
	printf("printf test\n");
	printf("%s is null pointer\n", np);
	printf("%d = 5\n", i);
	printf("%d = - max int\n", mi);
	printf("char %c = 'a'\n", 'a');
	printf("hex %x = ff\n", 0xff);
	printf("hex %02x = 00\n", 0);
	printf("signed %d = unsigned %u = hex %x\n", -3, -3, -3);
	printf("%d %s(s)%", 0, "message");
	printf("\n");
	printf("%d %s(s) with %%\n", 0, "message");
	sprintf(buf, "justif: \"%-10s\"\n", "left"); printf("%s", buf);
	sprintf(buf, "justif: \"%10s\"\n", "right"); printf("%s", buf);
	sprintf(buf, " 3: %04d zero padded\n", 3); printf("%s", buf);
	sprintf(buf, " 3: %-4d left justif.\n", 3); printf("%s", buf);
	sprintf(buf, " 3: %4d right justif.\n", 3); printf("%s", buf);
	sprintf(buf, "-3: %04d zero padded\n", -3); printf("%s", buf);
	sprintf(buf, "-3: %-4d left justif.\n", -3); printf("%s", buf);
	sprintf(buf, "-3: %4d right justif.\n", -3); printf("%s", buf);

	return 0;
}

/*
 * if you compile this file with
 *   gcc -Wall $(YOUR_C_OPTIONS) -DTEST_PRINTF -c printf.c
 * you will get a normal warning:
 *   printf.c:214: warning: spurious trailing `%' in format
 * this line is testing an invalid % at the end of the format string.
 *
 * this should display (on 32bit int machine) :
 *
 * Hello world!
 * printf test
 * (null) is null pointer
 * 5 = 5
 * -2147483647 = - max int
 * char a = 'a'
 * hex ff = ff
 * hex 00 = 00
 * signed -3 = unsigned 4294967293 = hex fffffffd
 * 0 message(s)
 * 0 message(s) with %
 * justif: "left      "
 * justif: "     right"
 *  3: 0003 zero padded
 *  3: 3    left justif.
 *  3:    3 right justif.
 * -3: -003 zero padded
 * -3: -3   left justif.
 * -3:   -3 right justif.
 */

#endif


