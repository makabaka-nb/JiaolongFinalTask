#include"usertask.h"

#include "cmsis_os.h"
#include"cmsis_os2.h"
uint32_t send=0;
uint32_t recv=0;

osSemaphoreAttr_t test_semaphore_attributes = {.name = "test_semaphore"};
osSemaphoreId_t test_semaphore_handle;


osThreadId_t testTaskHandle;
constexpr osThreadAttr_t testTask_attributes = {
    .name="testTask",
    .stack_size = 256*4,
    .priority=osPriorityBelowNormal
};

[[noreturn]] void test_task(void *) {
    while (true) {
        const auto tick=osKernelGetTickCount();
        if (send++%5==0) {
            osSemaphoreRelease(test_semaphore_handle);
        }
        osDelayUntil(tick+1);
    }
}

osThreadId_t test1TaskHandle;
constexpr osThreadAttr_t test1Task_attributes = {
    .name="test1Task",
    .stack_size = 256*4,
    .priority=osPriorityBelowNormal
};

[[noreturn]] void test1_task(void *) {
    while (true) {
        osSemaphoreAcquire(test_semaphore_handle,osWaitForever);
        recv++;
    }
}

void user_tasks_init() {
    test_semaphore_handle = osSemaphoreNew(1,0,&test_semaphore_attributes);
    testTaskHandle=osThreadNew(test_task,nullptr,&testTask_attributes);
    test1TaskHandle=osThreadNew(test1_task,nullptr,&test1Task_attributes);


}
