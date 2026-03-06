"use strict";
/*
 * Copyright (c) 2026, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


Object.defineProperty(exports, "__esModule", { value: true });

class Heap {
}

class Hwi {
}

class TaskModule {
}

class TaskInstance {
}

class QueueInstance {
}

class SystemStack{
}

class TimerInstance {
}

class FreeRTOS {
    constructor(ctx) {
        this.ctx = ctx;
        this.viewMap = [
            { name: 'Heap Overview', fxn: this.getHeap.bind(this), structName: Heap },
            { name: 'HW Interrupts', fxn: this.getHwiInstances.bind(this), structName: Hwi },
            { name: 'Semaphores, Mutex and Queue Instances', fxn: this.getQueueInstances.bind(this), structName: QueueInstance },
            { name: 'Stack Overview', fxn: this.getSystemStack.bind(this), structName: SystemStack },
            { name: 'Task Instances', fxn: this.getTaskInstances.bind(this), structName: TaskInstance },
            { name: 'Task Overview', fxn: this.getTaskModule.bind(this), structName: TaskModule },
            { name: 'Timers Overview', fxn: this.getTimers.bind(this), structName: TimerInstance },
        ];
        this.Program = this.ctx.getProgram();
    }

    async getHeap(){
        let table = new Array();

        let heapInfo = new Heap();
        heapInfo.Type = "System";

        let a53 = await this.isA53();
        let c7x = await this.isC7x();
        let r5f = await this.isR5F();
        let m4f = await this.isM4F();

        if (a53) {
            try {
                heapInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__heap_start__")).toString(16).toUpperCase();
                heapInfo.Size = await this.Program.lookupSymbolValue("__TI_HEAP_SIZE");
            }
            catch (e) {

            }
        }
        else if (r5f || m4f) {
            try {
                heapInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("_sys_memory")).toString(16).toUpperCase();
                heapInfo.Size = await this.Program.lookupSymbolValue("__SYSMEM_SIZE");
            }
            catch (e) {

            }
        }
        else if (c7x) {
            try {
                heapInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("_sys_memory")).toString(16).toUpperCase();
                heapInfo.Size = await this.Program.lookupSymbolValue("__TI_SYSMEM_SIZE");
            }
            catch (e) {

            }
        }

        heapInfo.Description = "Heap used by malloc() and pvPortMalloc()";

        table.push(heapInfo);
        table.sort(compareType);
        return (table);
    }

    async getHwiInstances() {
        let table = new Array();

        let a53 = await this.isA53();
        let c7x = await this.isC7x();
        let r5f = await this.isR5F();
        let m4f = await this.isM4F();

        if(r5f || a53 || m4f){
            try {
                let hwiCtrl = await this.Program.fetchVariable("gHwiCtrl");

                for (let i = 0; i < hwiCtrl.isr.length; i++) {
                    if (hwiCtrl.isr[i] != 0) {
                        let hwi = new Hwi();

                        hwi.InterruptNum = i;
                        hwi.IsrArgs = (hwiCtrl.isrArgs[i] != null) ? "0x" + hwiCtrl.isrArgs[i].toString(16).toUpperCase() : "";

                        if (hwiCtrl.isr[i] != null) {
                            hwi.IsrAddress = "0x" + hwiCtrl.isr[i].toString(16).toUpperCase();
                            const symbolLookup = await this.Program.task.symbols.lookupSymbols(hwi.IsrAddress + ",1");
                            hwi.IsrFunctionName = symbolLookup?.symbols?.[0]?.name || "";
                        } else {
                            hwi.IsrAddress = "";
                            hwi.IsrFunctionName = "";
                        }

                        table.push(hwi);
                    }
                }
            }
            catch (e) {

            }
        }

        if(c7x){
            try {
                let hwiCtrl = await this.Program.fetchVariable("Hwi_Module_state");

                for (let i = 0; i < hwiCtrl.dispatchTable.length; i++) {
                    if (hwiCtrl.dispatchTable[i] != 0) {
                        let hwi = new Hwi();

                        let hwiObjAddr = hwiCtrl.dispatchTable[i];
                        let hwiObj = await this.Program.fetchFromAddr(hwiObjAddr, "Hwi_Object");

                        hwi.InterruptNum = i;
                        hwi.IsrArgs = (hwiObj.args != null) ? "0x" + hwiObj.args.toString(16).toUpperCase() : "";

                        if (hwiObj.fxn != null) {
                            hwi.IsrAddress = "0x" + hwiObj.fxn.toString(16).toUpperCase();
                            const symbolLookup = await this.Program.task.symbols.lookupSymbols(hwi.IsrAddress + ",1");
                            hwi.IsrFunctionName = symbolLookup?.symbols?.[0]?.name || "";
                        } else {
                            hwi.IsrAddress = "";
                            hwi.IsrFunctionName = "";
                        }

                        table.push(hwi);
                    }
                }
            }
            catch (e) {

            }
        }
        return table;
    }

    async getTaskModule() {
        let view = [];

        let taskInfo = new TaskModule();

        try {
            let readyList = await this.Program.fetchVariable('pxReadyTasksLists');
            taskInfo.NumPriorities = readyList.length;
        }
        catch (e) {

        }

        try {
            taskInfo.NumTasks = await this.Program.fetchVariable('uxCurrentNumberOfTasks');
        }
        catch (e) {

        }

        try {
            taskInfo.TopReadyPriority = await this.Program.fetchVariable("uxTopReadyPriority");
        }
        catch (e) {

        }

        try {
            taskInfo.NumOverflows = await this.Program.fetchVariable("xNumOfOverflows");
        }
        catch (e) {

        }
        try {
            taskInfo.SchedulerStarted = Boolean(await this.Program.fetchVariable("xSchedulerRunning"));
        }
        catch (e) {

        }
        try {
            taskInfo.State = await this.Program.fetchVariable("uxSchedulerSuspended") ? "Suspended" : "Running";
        }
        catch (e) {

        }

        view.push(taskInfo);
        return (view);
    }

    async getTaskInstances() {
        const table = [];

        try {
            const readyList = await this.Program.fetchVariable('pxReadyTasksLists');
            for (let i = 0; i < readyList.length; i++) {
                await this.fillInTaskInstance(table, readyList[i], 'Ready');
            }
        }
        catch (e) {

        }

        try {
            const delay1List = await this.Program.fetchVariable('xDelayedTaskList1');
            await this.fillInTaskInstance(table, delay1List, 'Blocked');
        }
        catch (e) {

        }

        try {
            const delay2List = await this.Program.fetchVariable('xDelayedTaskList2');
            await this.fillInTaskInstance(table, delay2List, 'Blocked');
        }
        catch (e) {

        }

        try {
            const suspendedList = await this.Program.fetchVariable('xSuspendedTaskList');
            await this.fillInTaskInstance(table, suspendedList, 'Suspended');
        }
        catch (e) {

        }

        try {
            const terminatedList = await this.Program.fetchVariable('xTasksWaitingTermination');
            await this.fillInTaskInstance(table, terminatedList, 'Terminated');
        }
        catch (e) {

        }

        table.sort(compareAddress);
        return (table);
    }

    async getSystemStack() {
        let table = new Array();

        let a53 = await this.isA53();
        let c7x = await this.isC7x();
        let r5f = await this.isR5F();
        let m4f = await this.isM4F();
        let freertos = await this.isFREERTOS();

        if (r5f) {
            try {
                let stackInfo = new SystemStack();

                stackInfo.Type = "IRQ";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__IRQ_STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__IRQ_STACK_SIZE");

                if (freertos)
                    stackInfo.Description = "Stack used by IRQ for initial IRQ handling before switching to SVC stack";

                table.push(stackInfo);
            }
            catch (e) {

            }

            try {
                let stackInfo = new SystemStack()

                stackInfo.Type = "FIQ";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__FIQ_STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__FIQ_STACK_SIZE");

                table.push(stackInfo);
            }
            catch (e) {

            }

            try {
                let stackInfo = new SystemStack();

                stackInfo.Type = "SVC";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__SVC_STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__SVC_STACK_SIZE");
                if (freertos)
                    stackInfo.Description = "Stack used by SVC handler and also by IRQ handler after initial IRQ handling. User ISR runs within this stack context";

                table.push(stackInfo);
            }
            catch (e) {

            }

            try {
                let stackInfo = new SystemStack()

                stackInfo.Type = "ABORT";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__ABORT_STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__ABORT_STACK_SIZE");

                table.push(stackInfo);
            }
            catch (e) {

            }

            try {
                let stackInfo = new SystemStack()

                stackInfo.Type = "UNDEFINED";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__UNDEFINED_STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__UNDEFINED_STACK_SIZE");

                table.push(stackInfo);
            }
            catch (e) {

            }

            try {
                let stackInfo = new SystemStack()
                stackInfo.Type = "STACK";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__STACK_SIZE");
                if (freertos)
                    stackInfo.Description = "Stack used by program until FreeRTOS scheduler is started in main()";
                else
                    stackInfo.Description = "Stack used by non ISR context";
                table.push(stackInfo);
            }
            catch (e) {

            }
        }
        else if (m4f) {
            try {
                let stackInfo = new SystemStack()
                stackInfo.Type = "STACK";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__STACK_SIZE");
                if (freertos)
                    stackInfo.Description = "Stack used by program until FreeRTOS scheduler is started in main()";
                else
                    stackInfo.Description = "Stack used by non ISR context";
                table.push(stackInfo);
            }
            catch (e) {

            }
        }
        else if (a53) {
            try {
                let stackInfo = new SystemStack();
                stackInfo.Type = "EL1 STACK";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__TI_STACK_BASE")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__TI_STACK_SIZE");
                if (freertos)
                    stackInfo.Description = "Stack used by program until FreeRTOS scheduler is started in main() and later stack used by ISR context";
                else
                    stackInfo.Description = "Stack used by non ISR and ISR context";
                table.push(stackInfo);
            }
            catch (e) {

            }
        }
        else if (c7x) {
            try {
                let stackInfo = new SystemStack();
                stackInfo.Type = "STACK";
                stackInfo.BaseAddress = "0x" + (await this.Program.lookupSymbolValue("__TI_STACK_END")).toString(16).toUpperCase();
                stackInfo.Size = await this.Program.lookupSymbolValue("__TI_STACK_SIZE");
                if (freertos)
                    stackInfo.Description = "Stack used by program until FreeRTOS scheduler is started in main() and later stack used by ISR context";
                else
                    stackInfo.Description = "Stack used by non ISR and ISR context";
                table.push(stackInfo);
            }
            catch (e) {

            }
        }
        return (table);
    }

    async fillInTaskInstance(table, list, state)
    {
        let ptrSize = 4;
        let a53 = await this.isA53();
        let c7x = await this.isC7x();

        if(a53 || c7x)
        {
            ptrSize = 8;
        }

        try {
            let tcbBase = list.xListEnd.pxNext - ptrSize;
            const currentTask = await this.Program.fetchVariable('pxCurrentTCB');

            for (let i = 0; i < list.uxNumberOfItems; i++) {
                try {
                    const task = await this.Program.fetchFromAddr(tcbBase, 'TCB_t');
                    const taskInfo = new TaskInstance();
                    taskInfo.Address =  "0x" + tcbBase.toString(16).toUpperCase();

                    let name = '';
                    for (let j = 0; j < 12; j++) {
                        if (task.pcTaskName[j] == 0)
                            break;
                        name = name + String.fromCharCode(task.pcTaskName[j]);
                    }

                    taskInfo.TaskName = name;
                    taskInfo.Priority = task.uxPriority;
                    taskInfo.BasePriority = task.uxBasePriority;

                    if (tcbBase == currentTask) {
                        taskInfo.State = 'Running';
                    }
                    else {
                        taskInfo.State = state;
                    }

                    taskInfo.StackBase = task.pxStack;
                    taskInfo.CurrentTaskSP = task.pxTopOfStack;

                    const stackPattern = 0xa5;
                    const wordSize = 4;

                    if (taskInfo.CurrentTaskSP <= taskInfo.StackBase) {
                        taskInfo.UnusedStackSize = 'Stack Overflow';
                    } else {
                        try {
                            /* Binary search from StackBase toward CurrentTaskSP to find pattern boundary */
                            let low = taskInfo.StackBase;
                            let high = taskInfo.CurrentTaskSP;

                            while (low <= high) {
                                let mid = Math.floor((low + high) / 2);

                                try {
                                    const word = await this.Program.fetchArray('8u', mid, wordSize);
                                    const isPattern = (word[0] === stackPattern && word[1] === stackPattern &&
                                        word[2] === stackPattern && word[3] === stackPattern);

                                    if (isPattern) {
                                        low = mid + wordSize;
                                    } else {
                                        high = mid - wordSize;
                                    }
                                } catch (e) {
                                    high = mid - wordSize;
                                }
                            }

                            const endOfUnusedArea = low;
                            taskInfo.UnusedStackSize = endOfUnusedArea - taskInfo.StackBase;

                        } catch (e) {

                        }
                    }

                    if (typeof taskInfo.StackBase === 'number') {
                        taskInfo.StackBase = "0x" + taskInfo.StackBase.toString(16).toUpperCase();
                    }
                    if (typeof taskInfo.CurrentTaskSP === 'number') {
                        taskInfo.CurrentTaskSP = "0x" + taskInfo.CurrentTaskSP.toString(16).toUpperCase();
                    }

                    table.push(taskInfo);
                    tcbBase = task.xStateListItem.pxNext - ptrSize;
                }
                catch (e) {

                }
            }
        }
        catch (e) {

        }
    }

    async getTimers() {
        const table = [];
        let errVal = 0;

        try{
            const activeList1 = await this.Program.fetchVariable("xActiveTimerList1");
            await this.fillInTimerInstance(table, activeList1);
        }
        catch(e){
            errVal++;
        }
        try{
            const activeList2 = await this.Program.fetchVariable("xActiveTimerList2");
            await this.fillInTimerInstance(table, activeList2);
        }
        catch(e){
            errVal++;
        }

        if(errVal == 2){
            let message = new TimerInstance();
            message.Address = "No active timers, or configUSE_TIMERS is set to 0";
            table.push(message);
            return (table);
        }

        table.sort(compareHandle);
        return table;
    }

    async fillInTimerInstance(table, list) {
        if (list.uxNumberOfItems > 0) {
            try {
                let listItem = await this.Program.fetchFromAddr(list.xListEnd.pxNext, "ListItem_t");
                for (let i = 0; i < list.uxNumberOfItems; i++) {
                    try {
                        let timer = await this.Program.fetchFromAddr(listItem.pvOwner, "Timer_t");
                        let timerInfo = new TimerInstance();

                        timerInfo.Handle = "0x" + listItem.pvOwner.toString(16).toUpperCase();
                        timerInfo.Name = await this.helperReadStringFromAddr(timer.pcTimerName, 16);
                        timerInfo.PeriodInTicks = timer.xTimerPeriodInTicks;
                        timerInfo.AutoReload = "No";
                        timerInfo.Active = "No";
                        timerInfo.StaticallyAlloc = "No";

                        if (timer.ucStatus & 0x1)
                            timerInfo.Active = "Yes";
                        if (timer.ucStatus & 0x2)
                            timerInfo.StaticallyAlloc = "Yes";
                        if (timer.ucStatus & 0x4)
                            timerInfo.AutoReload = "Yes";

                        timerInfo.CallbackAddress = (timer.pxCallbackFunction != 0) ? "0x" + timer.pxCallbackFunction.toString(16).toUpperCase() : "-";
                        timerInfo.TimerID = timer.pvTimerID;

                        table.push(timerInfo);

                        if (i < (list.uxNumberOfItems - 1)) {
                            /* Traverse the list */
                            listItem = await this.Program.fetchFromAddr(listItem.pxNext, "ListItem_t");
                        }
                    }
                    catch (e) {

                    }
                }
            }
            catch (e) {

            }
        }
    }

    async isR5F() {
        try {
            let status = false;
            const symbolValue = await this.Program.lookupSymbolValue("__IRQ_STACK_END");
            if (symbolValue >= 0) {
                status = true;
            }
            else {
                status = false;
            }
            return status;
        }
        catch (e) {
            return false;
        }
    }

    async isA53() {
        try {
            let status = false;
            const symbolValue = await this.Program.lookupSymbolValue("__data_start__");
            if (symbolValue >= 0) {
                status = true;
            }
            else {
                status = false;
            }
            return status;
        }
        catch (e) {
            return false;
        }
    }

    async isC7x() {
        try {
            let status = false;
            const symbolValue = await this.Program.lookupSymbolValue("gMmu_tableArray_NS");
            if (symbolValue > 0) {
                status = true;
            }
            else {
                status = false;
            }
            return status;
        }
        catch (e) {
            return false;
        }
    }

    async isM4F() {
        try {
            let status = false;
            const symbolValue = await this.Program.lookupSymbolValue("gHwiP_vectorTable");
            if (symbolValue >= 0) {
                status = true;
            }
            else {
                status = false;
            }
            return status;
        }
        catch (e) {
            return false;
        }
    }

    async isFREERTOS() {
        try {
            let status = false;
            const symbolValue = await this.Program.lookupSymbolValue("pxCurrentTCB");
            if (symbolValue > 0) {
                status = true;
            }
            else {
                status = false;
            }
            return status;
        }
        catch (e) {
            return false;
        }
    }

    async helperReadStringFromAddr(ptr, maxLen){
        let name = "";
        try {
            let arr = await this.Program.fetchFromAddr(ptr, "char", maxLen);
            for (let i = 0; i < arr.length; i++) {
                if (arr[i] == 0) break;
                name += String.fromCharCode(arr[i]);
            }
        }
        catch (e) {

        }
        return name;
    }

    async helperGetListOfAddressesInListObj(listObj){
        let list = [];
        try {
            let currentItem = await this.Program.fetchFromAddr(listObj.xListEnd.pxNext, "ListItem_t");
            for (let i = 0; i < listObj.uxNumberOfItems; i++) {
                let address = currentItem.pvOwner;
                list.push(address);

                /* Traverse the list */
                currentItem = await this.Program.fetchFromAddr(currentItem.pxNext, "ListItem_t");
            }
        }
        catch (e) {

        }
        return list;
    }

    async getQueueInstances() {
        try {
            const table = [];

            let xQueueRegistry = await this.Program.fetchVariable("xQueueRegistry");
            if (!xQueueRegistry) {
                throw new Error("xQueueRegistry is undefined or null");
            }

            for (let i = 0; i < xQueueRegistry.length; i++) {
                if (xQueueRegistry[i].pcQueueName != 0 && xQueueRegistry[i].xHandle != 0) {
                    let xHandle = await this.Program.fetchFromAddr(xQueueRegistry[i].xHandle, "xQUEUE_ROV");
                    if (!xHandle) {
                        throw new Error("xHandle is undefined or null");
                    }

                    let queue = new QueueInstance();
                    queue.Type = await this.getQueueType(xHandle.ucQueueType);
                    queue.Name = await this.helperReadStringFromAddr(xQueueRegistry[i].pcQueueName, 16)
                    queue.Handle = "0x" + Number(xQueueRegistry[i].xHandle).toString(16).toUpperCase();
                    queue.CurCount = xHandle.uxMessagesWaiting;
                    queue.MaxCount = xHandle.uxLength;
                    queue.QueueElemSize = xHandle.uxItemSize;
                    queue.TasksWaitingToRecv = xHandle.xTasksWaitingToReceive.uxNumberOfItems;
                    queue.TasksWaitingToSend = xHandle.xTasksWaitingToSend.uxNumberOfItems;

                    if (queue.TasksWaitingToRecv > 0) {
                        let task = await this.getTaskTcbFromQueueList(xHandle.xTasksWaitingToReceive);

                        if (!task) {
                            throw new Error("task is undefined or null");
                        }

                        queue.TopWaitingToRecvTaskHandle = "0x" + task.tcbBase.toString(16).toUpperCase();
                        queue.TopWaitingToRecvTaskName = this.getString(task.tcb.pcTaskName);
                    }

                    if (queue.TasksWaitingToSend > 0) {
                        let task = await this.getTaskTcbFromQueueList(xHandle.xTasksWaitingToSend);

                        if (!task) {
                            throw new Error("task is undefined or null");
                        }

                        queue.TopWaitingToSendTaskHandle = "0x" + task.tcbBase.toString(16).toUpperCase();
                        queue.TopWaitingToSendTaskName = this.getString(task.tcb.pcTaskName);
                    }

                    if ((queue.Type == "Mutex" || queue.Type == "Mutex (Recursive)") && xHandle.xSemaphore.xMutexHolder != 0) {
                        let tcb = await this.Program.fetchFromAddr(xHandle.xSemaphore.xMutexHolder, "TCB_t");

                        if (!tcb) {
                            throw new Error("tcb is undefined or null");
                        }

                        queue.MutexHolderTaskHandle = "0x" + Number(xHandle.xSemaphore.xMutexHolder).toString(16).toUpperCase();
                        queue.MutexHolderTaskName = this.getString(tcb.pcTaskName);
                        queue.RecursiveMutexCallCount = xHandle.xSemaphore.uxRecursiveCallCount;
                    }

                    table.push(queue);
                }
            }

            table.sort(compareType);
            return table;
        } catch (error) {
            return [];
        }
    }

    getQueueType(ucQueueType){
        switch (ucQueueType) {
            default:
            case 0:
                return "Queue";
            case 1:
                return "Mutex";
            case 2:
                return "Semaphore (Counting)";
            case 3:
                return "Semaphore (Binary)";
            case 4:
                return "Mutex (Recursive)";
        }
    }

    async getTaskTcbFromQueueList(list) {
        let listItem = await this.Program.fetchFromAddr(list.xListEnd.pxNext, "ListItem_t");
        let tcb = await this.Program.fetchFromAddr(listItem.pvOwner, "TCB_t");
        return { tcb: tcb, tcbBase: listItem.pvOwner};
    }

    getString(charPtr)
    {
        let name = "";
        for (let j = 0; j < 12; j++) {
            if (charPtr[j] == 0) break;
            name = name + String.fromCharCode(charPtr[j]);
        }
        return name;
    }
    
    getModuleName() {
        return 'FreeRTOS';
    }
}

/* Helper functions */
function compareAddress(a, b) {
    return +(a.Address ?? 0) - +(b.Address ?? 0);
}

function compareHandle(a, b) {
    return +(a.Handle ?? 0) - +(b.Handle ?? 0);
}

function compareType(a, b) {
    return +(a.Type ?? 0) - +(b.Type ?? 0);
}

exports.classCtor = FreeRTOS;