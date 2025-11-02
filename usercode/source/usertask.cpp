#include"usertask.h"

#include "cmsis_os.h"
#include"cmsis_os2.h"
uint32_t count=0;

osThreadId_t testTaskHandle;
constexpr osThreadAttr_t testTask_attributes = {
    .name="testTask",
    .stack_size = 256*4,
    .priority=osPriorityBelowNormal
};

void test_task(void *) {
    while (true) {
        const auto tick=osKernelGetTickCount();
        ++count;
        osDelayUntil(tick+1);
    }
}

void user_tasks_init() {
    testTaskHandle=osThreadNew(test_task,nullptr,&testTask_attributes);

}
