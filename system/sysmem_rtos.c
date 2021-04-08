#include <stdlib.h>
#include <malloc.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "stm32f10x.h"

#include "newlib.h"
#include "FreeRTOS.h" // defines public interface we're implementing here
#if !defined(configUSE_NEWLIB_REENTRANT) ||  (configUSE_NEWLIB_REENTRANT!=1)
  #warning "#define configUSE_NEWLIB_REENTRANT 1 // Required for thread-safety of newlib sprintf, dtoa, strtok, etc..."
  // If you're *REALLY* sure you don't need FreeRTOS's newlib reentrancy support, comment out the above warning...
#endif
#include "task.h"

typedef struct A_BLOCK_LINK {
        struct A_BLOCK_LINK *pxNextFreeBlock;
        size_t xBlockSize;
} BlockLink_t;

static const uint16_t heapSTRUCT_SIZE   = ( ( sizeof ( BlockLink_t ) + ( portBYTE_ALIGNMENT - 1 ) ) & ~portBYTE_ALIGNMENT_MASK );

caddr_t _sbrk(int incr)
{
    errno = ENOMEM;
    return (caddr_t)-1;
}

void __malloc_lock(struct _reent* re)
{
    vTaskSuspendAll();
};

void __malloc_unlock(struct _reent* re)
{
    xTaskResumeAll();
};

void __env_lock()
{
    vTaskSuspendAll();
};

void __env_unlock()
{
    xTaskResumeAll();
};

void* _malloc_r(struct _reent* re, size_t size)
{
    return pvPortMalloc(size);
}

void* _calloc_r(struct _reent* re, size_t num, size_t size)
{
    return pvPortMalloc(num*size);
}

void* _realloc_r(struct _reent *re, void* oldAddr, size_t newSize)
{
    BlockLink_t *block = NULL;
    size_t toCopy = 0;
    void *newAddr = NULL;

    newAddr = pvPortMalloc(newSize);
    if (newAddr != NULL) {
        /* We need the block struct pointer to get the current size */
        block = oldAddr - heapSTRUCT_SIZE;
        /* determine the size to be copied */
        toCopy = (newSize<block->xBlockSize)?(newSize):(block->xBlockSize);
        /* copy old block into new one */
        memcpy((void *)newAddr, (void *)oldAddr, (size_t)toCopy);
        vPortFree(oldAddr);
    }
    return newAddr;
}

void _free_r(struct _reent *re, void* ptr)
{
    vPortFree(ptr);
}
