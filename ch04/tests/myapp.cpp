
#include "rttest/rttest.h"
#include <stdio.h>

void* control_loop(void* arg) {
  // 你的实际控制逻辑代码放在这里
  return NULL;
}

int main(int argc, char** argv) {
  // 1. 初始化参数：运行10000次，周期1ms，使用FIFO调度，优先级90
  struct timespec period = {0, 1000000}; // 1毫秒 (秒, 纳秒)
  rttest_init(10000, period, SCHED_FIFO, 90, 0, 0, NULL);

  // 2. 为实时任务做准备（锁定内存防止交换，设置线程优先级）
  rttest_lock_and_prefault_dynamic();
  rttest_set_sched_priority(90, SCHED_FIFO);

  // 3. 启动测试，rttest会确保control_loop尽可能精确地按1ms周期运行
  rttest_spin(control_loop, NULL);

  // 4. 获取并打印结果
  struct rttest_results results;
  rttest_calculate_statistics(&results);
  printf("最大延迟: %lld ns\n", (long long)results.max_latency);

  // 5. 将详细的延迟数据写入文件以供分析
  rttest_write_results();
  rttest_finish();
  return 0;
}