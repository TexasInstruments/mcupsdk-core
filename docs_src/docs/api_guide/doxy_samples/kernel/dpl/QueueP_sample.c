
//! [include]
#include <kernel/dpl/QueueP.h>
//! [include]
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/csl_types.h>

//! [define]
/* Define a client structure for the queue elements.
   The object of type QueueP_Elem should be placed at the head of the structure. */
typedef struct Test_Queue_Elem_s
{
    QueueP_Elem lnk;
    uint32_t    index;
} Test_Queue_Elem;

Test_Queue_Elem  elem1, elem2;
QueueP_Object    qObj;
//! [define]

void samples()
{
{
//! [queue_usage]
    QueueP_Handle   handle;
    Test_Queue_Elem *pElem;
    /* Create the Queue. */
    handle = QueueP_create(&qObj);

    /* Put elements in a Queue. */
    QueueP_put(handle, (QueueP_Elem *)&elem1);
    QueueP_put(handle, (QueueP_Elem *)&elem2);

    /* Get elements from the queue. */
    pElem = (Test_Queue_Elem *)QueueP_get(handle);
    pElem = (Test_Queue_Elem *)QueueP_get(handle);
    /* Typically pElem would be processed by application. 
     * In sample typecast to void to kill warning 
     * regarding variable set but not used 
     */
    (void) pElem;

    QueueP_delete(handle);
//! [queue_usage]
}
}

