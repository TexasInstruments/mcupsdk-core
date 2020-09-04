
//! [include]
#include <kernel/dpl/TaskP.h>
//! [include]
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/csl_types.h>

//! [define]
/* Task priority, stack, stack size and task objects, these MUST be global's */
#define MY_TASK_PRI         (8U)
#define MY_TASK_STACK_SIZE  (4*1024U)
uint8_t gMyTaskStack[MY_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gMyTask;

/* Application specific task arguments */
typedef struct {
    uint32_t value;
} MyTask_Args;

MyTask_Args gMyTask_args;
//! [define]

//! [taskmain]
/* Task entry point or main function for this task */
void myTaskMain(void *args)
{
    MyTask_Args *myArgs = (MyTask_Args*)args;
    DebugP_assert(myArgs != NULL);

    /* myArgs points to structure pointer passed during TaskP_construct */

    /* do something in the task */

    /* when done call below function, DO NOT 'return` from this function */
    TaskP_exit();
}
//! [taskmain]

void samples()
{
{
//! [create]
    int32_t status;
    TaskP_Params myTaskParams; /* this need not be global variable */

    TaskP_Params_init(&myTaskParams);
    myTaskParams.name = "MY_TASK";
    myTaskParams.stackSize = MY_TASK_STACK_SIZE;
    myTaskParams.stack = gMyTaskStack;
    myTaskParams.priority = MY_TASK_PRI;
    myTaskParams.args = &gMyTask_args;
    myTaskParams.taskMain = myTaskMain;

    status = TaskP_construct(&gMyTask, &myTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);
//! [create]
}
}

void get_load()
{
{
//! [load]
    TaskP_Load taskLoad;
    uint32_t cpuLoad;

    cpuLoad = TaskP_loadGetTotalCpuLoad();
    DebugP_log(" LOAD: CPU  = %2d.%2d %%\r\n", cpuLoad/100, cpuLoad%100 );

    TaskP_loadGet(&gMyTask, &taskLoad);
    DebugP_log(" LOAD: %s = %2d.%2d %%\r\n", taskLoad.name, taskLoad.cpuLoad/100, taskLoad.cpuLoad%100 );
//! [load]
}
}
