#include "./pt_app/pt_app.h"

/* ====================== 1. 任务控制块 (每个任务一个) ====================== */
static struct pt pt_task1;  // 通用任务1
static struct pt pt_task2;  // 通用任务2
static struct pt pt_task3;  // 通用任务3
// 新增任务：在这里加一行 static struct pt pt_任务名;

/* ====================== 2. 任务函数 (统一模板) ====================== */
/**
 * @brief 通用任务模板1 → 修改延时和业务代码即可
 */
static void task1_function(void)
{
    static uint32_t last_tick;
    pt_begin(&pt_task1);

    last_tick = HAL_GetTick();
    while(1)
    {
        // ============== 【改这里】延时毫秒数 ==============
        pt_wait(&pt_task1, (HAL_GetTick() - last_tick) >= 100);  // 100ms 执行一次
        // ================================================

        last_tick = HAL_GetTick();

        // ============== 【丢这里】你的业务处理函数 ==============
        // 示例：LED翻转 / 按键扫描 / 串口处理 / 传感器读取
        // GpioOut_Toggle(0);
        // ====================================================
    }
    pt_end(&pt_task1);
}

/**
 * @brief 通用任务模板2
 */
static void task2_function(void)
{
    static uint32_t last_tick;
    pt_begin(&pt_task2);

    last_tick = HAL_GetTick();
    while(1)
    {
        // ============== 【改这里】延时毫秒数 ==============
        pt_wait(&pt_task2, (HAL_GetTick() - last_tick) >= 10);  // 10ms 执行一次
        // ================================================

        last_tick = HAL_GetTick();

        // ============== 【丢这里】你的业务处理函数 ==============

        // ====================================================
    }
    pt_end(&pt_task2);
}

/**
 * @brief 通用任务模板3
 */
static void task3_function(void)
{
    static uint32_t last_tick;
    pt_begin(&pt_task3);

    last_tick = HAL_GetTick();
    while(1)
    {
        // ============== 【改这里】延时毫秒数 ==============
        pt_wait(&pt_task3, (HAL_GetTick() - last_tick) >= 50);  // 50ms 执行一次
        // ================================================

        last_tick = HAL_GetTick();

        // ============== 【丢这里】你的业务处理函数 ==============

        // ====================================================
    }
    pt_end(&pt_task3);
}

/* ====================== 3. 初始化所有任务 ====================== */
void pt_app_init(void)
{
    pt_task1 = (struct pt)pt_init();
    pt_task2 = (struct pt)pt_init();
    pt_task3 = (struct pt)pt_init();
    // 新增任务：在这里加一行 pt_任务名 = (struct pt)pt_init();
}

/* ====================== 4. 调度所有任务 ====================== */
void pt_app_task(void)
{
    task1_function();
    task2_function();
    task3_function();
    // 新增任务：在这里加一行 任务函数();
}
