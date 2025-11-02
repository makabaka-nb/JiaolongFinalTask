#include"usertask.h"

#include "cmsis_os.h"
#include"cmsis_os2.h"
uint32_t send=0;
uint32_t recv=0;

osMessageQueueId_t test_queue_handle;
osMessageQueueAttr_t test_queue_atrributes ={.name="test_queue"};


osThreadId_t testTaskHandle;
constexpr osThreadAttr_t testTask_attributes = {
    .name="testTask",
    .stack_size = 256*4,
    .priority=osPriorityBelowNormal
};

[[noreturn]] void test_task(void *) {
    while (true) {
        const auto tick=osKernelGetTickCount();
        send+=1;
        osMessageQueuePut(test_queue_handle,&send,0,0);
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
        osMessageQueueGet(test_queue_handle,&recv,0,0);
    }
}

void user_tasks_init() {
    test_queue_handle = osMessageQueueNew(10,sizeof(uint32_t),&test_queue_atrributes);
    testTaskHandle=osThreadNew(test_task,nullptr,&testTask_attributes);
    test1TaskHandle=osThreadNew(test1_task,nullptr,&test1Task_attributes);


}
