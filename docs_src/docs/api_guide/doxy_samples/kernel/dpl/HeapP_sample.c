
//! [include]
#include <stdio.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/DebugP.h>
//! [include]
 
//! [heap def]
/* Heap memory, recommend to align to HeapP_BYTE_ALIGNMENT */
#define MY_HEAP_MEM_SIZE  (32*1024u)
uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/* Heap handle */
HeapP_Object gMyHeapObj;
//! [heap def]

void samples()
{
//! [create]
    HeapP_construct(&gMyHeapObj, gMyHeapMem, MY_HEAP_MEM_SIZE);
//! [create]

//! [alloc]
    void *ptr;
    
    /* allocate memory from heap */
    ptr = HeapP_alloc(&gMyHeapObj, 1024u);
    DebugP_assert(ptr!=NULL);

    /* use the memory */

    /* free the memory back to the same heap */
    HeapP_free(&gMyHeapObj, ptr);
//! [alloc]

}