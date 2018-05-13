
/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "stm32f10x.h"

#include <stdlib.h> // maps to newlib...
#include <malloc.h> // mallinfo...
#include <errno.h>  // ENOMEM
 
#include "newlib.h"
#if (__NEWLIB__ != 2) || (__NEWLIB_MINOR__ != 5)
  #warning "This wrapper was verified for newlib version 2.5.0; please ensure newlib's external requirements for malloc-family are unchanged!"
#endif
 
#include "FreeRTOS.h" // defines public interface we're implementing here
#include "task.h"


#ifndef STDOUT_USART
#define STDOUT_USART 2
#endif

#ifndef STDERR_USART
#define STDERR_USART 2
#endif

#ifndef STDIN_USART
#define STDIN_USART 2
#endif

#undef errno
extern int errno;

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

//int _write(int file, char *ptr, int len);

void _exit(int status) {
    while (1) {
        ;
    }
}

int _close(int file) {
    return -1;
}
/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid() {
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this

caddr_t _sbrk(int incr) {

    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
//         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}
 */

extern char __HeapBase, __HeapLimit, HEAP_SIZE;  // make sure to define these symbols in linker command file
static int heapBytesRemaining = (int)&HEAP_SIZE; // that's (&__HeapLimit)-(&__HeapBase)

caddr_t _sbrk(int incr) {
    static char *currentHeapEnd = &__HeapBase;

    vTaskSuspendAll();
    char *previousHeapEnd = currentHeapEnd;
    if (currentHeapEnd + incr > &__HeapLimit) {
	errno = ENOMEM;
	xTaskResumeAll();
        return (caddr_t)-1; // the malloc-family routine that called sbrk will return 0
    }

    currentHeapEnd += incr;
    heapBytesRemaining -= incr;

    xTaskResumeAll();
    return (caddr_t) previousHeapEnd;
}

void __malloc_lock()     {       vTaskSuspendAll(); };
void __malloc_unlock()   { (void)xTaskResumeAll();  };
 
// newlib also requires implementing locks for the application's environment memory space,
// accessed by newlib's setenv() and getenv() functions.
// As these are trivial functions, momentarily suspend task switching (rather than semaphore).
// ToDo: Move __env_lock/unlock to a separate newlib helper file.
void __env_lock()    {       vTaskSuspendAll(); };
void __env_unlock()  { (void)xTaskResumeAll();  };
 
/// /brief  Wrap malloc/malloc_r to help debug who requests memory and why.
/// Add to the linker command line: -Xlinker --wrap=malloc -Xlinker --wrap=_malloc_r
// Note: These functions are normally unused and stripped by linker.
/*
void *__wrap_malloc(size_t nbytes) {
    extern void * __real_malloc(size_t nbytes);
    void *p = __real_malloc(nbytes); // Solely for debug breakpoint...
    return p;
};
void *__wrap__malloc_r(void *reent, size_t nbytes) {
    extern void * __real__malloc_r(size_t nbytes);
    void *p = __real__malloc_r(nbytes); // Solely for debug breakpoint...
    return p;
};
*/

// ================================================================================================
// Implement FreeRTOS's memory API using newlib-provided malloc family.
// ================================================================================================


void *pvPortMalloc( size_t xSize ) PRIVILEGED_FUNCTION {
    void *p = malloc(xSize);
    return p;
}

void vPortFree( void *pv ) PRIVILEGED_FUNCTION {
    free(pv);
};
 

size_t xPortGetFreeHeapSize( void ) PRIVILEGED_FUNCTION {
    struct mallinfo mi = mallinfo();
    return mi.fordblks + heapBytesRemaining;
}


// GetMinimumEverFree is not available in newlib's malloc implementation.
// So, no implementation provided: size_t xPortGetMinimumEverFreeHeapSize( void ) PRIVILEGED_FUNCTION;
 
//! No implementation needed, but stub provided in case application already calls vPortInitialiseBlocks
void vPortInitialiseBlocks( void ) PRIVILEGED_FUNCTION {};

//char * _sbrk(int incr) { return sbrk(incr); };



/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}




