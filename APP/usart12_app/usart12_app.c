#include "./usart12_app/usart12_app.h"

/* --- ����1 (USART1) �����Դ --- */
uint8_t uart1_rx_dma_buffer[128] = {0}; // UART1��DMA���ջ�����
uint8_t uart1_process_buffer[128] = {0}; // UART1�����ݴ���������
struct rt_ringbuffer uart1_ringbuffer;       // UART1�Ļ��λ������ṹ��
uint8_t ringbuffer1_pool[128] = {0};        // UART1���λ��������ڴ��

/* --- ����2 (USART2 - ��������) �����Դ --- */
uint8_t uart2_rx_dma_buffer[128] = {0}; // UART2��DMA���ջ�����
uint8_t uart2_process_buffer[128] = {0}; // UART2�����ݴ���������
struct rt_ringbuffer uart2_ringbuffer;       // UART2�Ļ��λ������ṹ��
uint8_t ringbuffer2_pool[128] = {0};        // UART2���λ��������ڴ��

// =======================================================
// ===           ͨ��״̬������������                  ===
// =======================================================

// --- 1. ����֡��ʽ�ͻ�������С ---
#define FRAME_HEADER '#'  // ֡ͷ
#define FRAME_TAIL   '@'  // ֡β
#define CMD_BUFFER_SIZE 32 // ������󳤶�(����ͷβ)���ɸ�����Ҫ����

// --- 2. Ϊ����1(PC)����һ�׶�����״̬���� ---
static char    g_u1_cmd_buffer[CMD_BUFFER_SIZE]; // ���ڴ�Ŵ���1���ڽ��յ�����
static uint8_t g_u1_cmd_len = 0;                 // ��ǰ�ѽ�������ĳ���
static uint8_t g_is_u1_receiving = 0;            // ����1��״̬��־: 0-����, 1-���ڽ���

// --- 3. Ϊ����2(����)����һ�׶�����״̬���� ---
static char    g_u2_cmd_buffer[CMD_BUFFER_SIZE]; // ���ڴ�Ŵ���2���ڽ��յ�����
static uint8_t g_u2_cmd_len = 0;                 // ��ǰ�ѽ�������ĳ���
static uint8_t g_is_u2_receiving = 0;            // ����2��״̬��־: 0-����, 1-���ڽ���


//��������Ǵ��ڷ��ͺ���, ���Ը�ÿ������ʹ��, ��һ���Ǵ���Ĵ��ڵľ��, �������printf���÷�
int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buffer[512]; // ��ʱ�洢��ʽ������ַ���
	va_list arg;      // �����ɱ����
	int len;          // �����ַ�������

	va_start(arg, format);
	// ��ȫ�ظ�ʽ���ַ����� buffer
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);

	// ͨ�� HAL �ⷢ�� buffer �е�����
	HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
	return len;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. ȷ����Ŀ�괮�� (USART1)
    if (huart->Instance == USART1)
    {
        // 2. ����ֹͣ��ǰ�� DMA ���� (������ڽ�����)
        //    ��Ϊ�����ж���ζ�ŷ��ͷ��Ѿ�ֹͣ����ֹ DMA �����ȴ������
        HAL_UART_DMAStop(huart);

        // 3. �� DMA ����������Ч������ (Size ���ֽ�) ���Ƶ�������������
        memcpy(uart1_process_buffer, uart1_rx_dma_buffer, Size); 
        // ע�⣺����ʹ���� Size��ֻ����ʵ�ʽ��յ�������
        
        rt_ringbuffer_put(&uart1_ringbuffer, uart1_rx_dma_buffer, Size);

        // 6. **�ؼ�������������һ�� DMA ���н���**
        //    �����ٴε��ã�����ֻ�������һ��
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, sizeof(uart1_rx_dma_buffer));
        
        // 7. 关半传输中断, 否则一帧会回调两次(HT + TC)
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
		// --- �ж��Ǵ���2�������¼� ---
    else if (huart->Instance == USART2)
    {
        HAL_UART_DMAStop(huart);
        
        // ������2���յ������ݷ��봮��2�Ļ��λ�����
        rt_ringbuffer_put(&uart2_ringbuffer, uart2_rx_dma_buffer, Size);
        
        // ������������2��DMA����
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buffer, sizeof(uart2_rx_dma_buffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

// ����1(PC)�����������
/* =======================================================
 * ===        UART4 电机调试指令台 (串口1 输入)        ===
 * =======================================================
 * 帧格式沿用本文件已有的 #指令@ 状态机。
 *   #SCAN@   扫描 UART4 上 ID 1~16
 *   #E1@ #E0@   使能 / 失能
 *   #V100@ #V-100@   速度模式, 正数CW 负数CCW, 0=停
 *   #A20@    加速度档位 0~255 (0=不走曲线, 直接冲)
 *   #S@      急停
 *   #R@      读实时位置 + 实时转速
 *   #ID3@    改电机ID并存Flash
 *   #BC100@  广播(addr=0)跑起来 —— 不知道ID时的兜底
 *   #DBG1@ #DBG0@   开/关收发HEX打印
 */

static uint16_t g_acc = 10;      /* 当前加速度档位 */

/* 解析一条应答帧并翻译成人话。返回 true 表示认出来了 */
static bool motor_report(const uint8_t *f, uint16_t n)
{
    /* 驱动器不认识的命令一律回 [addr][00][EE][6B], 不回显是哪条错了 */
    if (n >= 4 && f[1] == 0x00 && f[2] == 0xEE) {
        my_printf(&huart1, "    -> COMMAND ERROR: driver rejected that read byte\r\n");
        return true;
    }
    if (n >= 8 && f[1] == ZDT_EMM_RD_POS) {
        int32_t raw = ((int32_t)f[3] << 24) | ((int32_t)f[4] << 16) |
                      ((int32_t)f[5] << 8)  |  (int32_t)f[6];
        float deg = raw * 360.0f / 65536.0f;
        if (f[2]) deg = -deg;
        my_printf(&huart1, "    -> pos = %.2f deg\r\n", deg);
        return true;
    }
    if (n >= 6 && f[1] == ZDT_EMM_RD_VEL) {
        int32_t rpm = ((int32_t)f[3] << 8) | (int32_t)f[4];
        if (f[2]) rpm = -rpm;
        my_printf(&huart1, "    -> speed = %ld RPM\r\n", (long)rpm);
        return true;
    }
    return false;
}

/* 发一条读命令并等应答 */
static void motor_query(uint8_t rd_cmd)
{
    zdt_port_rx_clear(ZDT_BUS_CHASSIS);
    zdt_read_raw(&chassis, rd_cmd);

    uint16_t n = zdt_port_wait_frame(ZDT_BUS_CHASSIS, 50);
    if (!n) {
        my_printf(&huart1, "    -> no reply (timeout)\r\n");
        return;
    }
    motor_report(zdt_port_rx_data(ZDT_BUS_CHASSIS), n);
    zdt_port_rx_clear(ZDT_BUS_CHASSIS);
}

/* 处理电机相关指令。返回 true 表示这条已被处理 */
static bool motor_command(const char* cmd)
{
    if (strcmp(cmd, "SCAN") == 0) {
        zdt_scan_bus(ZDT_BUS_CHASSIS);
        return true;
    }
    if (strcmp(cmd, "E1") == 0 || strcmp(cmd, "E0") == 0) {
        bool on = (cmd[1] == '1');
        zdt_enable(&chassis, on);
        my_printf(&huart1, "  enable=%d (id=%u)\r\n", (int)on, (unsigned)chassis.addr);
        return true;
    }
    if (strcmp(cmd, "S") == 0) {
        zdt_stop(&chassis);
        my_printf(&huart1, "  STOP\r\n");
        return true;
    }
    if (strcmp(cmd, "R") == 0) {
        motor_query(ZDT_EMM_RD_POS);
        motor_query(ZDT_EMM_RD_VEL);
        return true;
    }
    if (strcmp(cmd, "DBG1") == 0 || strcmp(cmd, "DBG0") == 0) {
        zdt_dbg_set(cmd[3] == '1');
        my_printf(&huart1, "  hex dump=%c\r\n", cmd[3]);
        return true;
    }
    /* 注意: 带参数的单字母指令必须要求后面确实有字符, 否则会吃掉本文件
     * 原有的 #A@ (GpioOut_Toggle) 这种无参指令。 */
    if (cmd[0] == 'V' && cmd[1] != '\0') {
        int rpm = atoi(cmd + 1);
        if (rpm == 0) {
            zdt_stop(&chassis);
            my_printf(&huart1, "  V0 -> stop\r\n");
        } else {
            zdt_dir_t dir = (rpm > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
            uint16_t  mag = (uint16_t)((rpm > 0) ? rpm : -rpm);
            zdt_run_vel(&chassis, dir, g_acc, (float)mag);
            my_printf(&huart1, "  run %s %u RPM (acc=%u)\r\n",
                      (dir == ZDT_DIR_CW) ? "CW" : "CCW",
                      (unsigned)mag, (unsigned)g_acc);
        }
        return true;
    }
    if (cmd[0] == 'A' && cmd[1] != '\0') {   /* 光一个 #A@ 留给原来的 GPIO toggle */
        int a = atoi(cmd + 1);
        if (a < 0)   a = 0;
        if (a > 255) a = 255;
        g_acc = (uint16_t)a;
        my_printf(&huart1, "  acc gear = %u (takes effect on next #V)\r\n", (unsigned)g_acc);
        return true;
    }
    if (strncmp(cmd, "ID", 2) == 0) {
        int id = atoi(cmd + 2);
        if (id < 1 || id > 247) {
            my_printf(&huart1, "  bad id (1..247)\r\n");
            return true;
        }
        zdt_set_id(&chassis, true, (uint8_t)id);
        my_printf(&huart1, "  ID %u -> %u, saved to flash.\r\n",
                  (unsigned)chassis.addr, (unsigned)id);
        my_printf(&huart1, "  !! now edit main.c: zdt_init(&chassis, ZDT_FW_EMM, %u, "
                           "zdt_port4_send) and reflash\r\n", (unsigned)id);
        return true;
    }
    if (strncmp(cmd, "BC", 2) == 0) {
        /* 广播地址 0: 总线上所有电机都响应, 用来确认链路是否通 */
        static zdt_t bc;
        static bool  bc_ready = false;
        if (!bc_ready) { zdt_init(&bc, ZDT_FW_EMM, 0, zdt_port4_send); bc_ready = true; }

        int rpm = atoi(cmd + 2);
        zdt_enable(&bc, true);
        HAL_Delay(10);
        if (rpm == 0) {
            zdt_stop(&bc);
            my_printf(&huart1, "  broadcast stop\r\n");
        } else {
            zdt_dir_t dir = (rpm > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
            uint16_t  mag = (uint16_t)((rpm > 0) ? rpm : -rpm);
            zdt_run_vel(&bc, dir, g_acc, (float)mag);
            my_printf(&huart1, "  broadcast run %u RPM (all motors on UART4)\r\n",
                      (unsigned)mag);
        }
        return true;
    }
    return false;
}

// 串口1(PC)命令处理函数
void process_command_u1(const char* cmd)
{
    my_printf(&huart1, "[PC] Received command: %s\r\n", cmd);

    if (motor_command(cmd)) return;

    if (strcmp(cmd, "A") == 0)
    {
        GpioOut_Toggle(0);
        my_printf(&huart1, "Action: Toggled GPIO 0.\r\n");
    }
    else if (strcmp(cmd, "B") == 0)
    {
        // ִ��B����Ĳ���...
        my_printf(&huart1, "Action: Command B executed.\r\n");
    }
    else
    {
        my_printf(&huart1, "Error: Unknown command from PC.\r\n");
    }
}

// ����2(����)�����������
void process_command_u2(const char* cmd)
{
    my_printf(&huart1, "[BT] Received command: %s\r\n", cmd); // ͨ������1��ӡ������Ϣ

    if (strcmp(cmd, "LED_ON") == 0)
    {
        // ������д��LED�Ĵ���
        my_printf(&huart1, "Action: BT command LED_ON executed.\r\n");
    }
    else if (strcmp(cmd, "LED_OFF") == 0)
    {
        // ������д�ر�LED�Ĵ���
        my_printf(&huart1, "Action: BT command LED_OFF executed.\r\n");
    }
    else
    {
        my_printf(&huart1, "Error: Unknown command from BT.\r\n");
    }
}

// ����1���ֽ���������
void parse_uart1_data(const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        if (g_is_u1_receiving) // �����ǰ�ڡ�����״̬��
        {
            if (byte == FRAME_TAIL) // 1. �յ���֡β'@'
            {
                g_u1_cmd_buffer[g_u1_cmd_len] = '\0'; // �����ַ���������
                process_command_u1(g_u1_cmd_buffer); // ������������
                g_is_u1_receiving = 0; // �лء�����״̬��
                g_u1_cmd_len = 0;      // ��ռ���
            }
            else if (byte == FRAME_HEADER) // 2. �쳣����֡���յ����µ�֡ͷ'#'
            {
                g_u1_cmd_len = 0; // �����������൱�ڶ���֮ǰ�յ��ģ����¿�ʼ
            }
            else if (g_u1_cmd_len < CMD_BUFFER_SIZE - 1) // 3. �յ���ͨ�����ֽ�
            {
                g_u1_cmd_buffer[g_u1_cmd_len++] = byte; // ���뻺����
            }
            else // 4. �쳣�����������
            {
                g_is_u1_receiving = 0; // �����������ݣ��лء�����״̬��
                g_u1_cmd_len = 0;
            }
        }
        else if (byte == FRAME_HEADER) // �����ǰ�ڡ�����״̬�������յ���֡ͷ'#'
        {
            g_is_u1_receiving = 1; // �л���������״̬��
            g_u1_cmd_len = 0;      // ׼����ͷ��ʼ����
        }
        // ����ڡ�����״̬���յ���'#'�ֽڣ���ֱ�Ӻ���
    }
}

// ����2���ֽ��������� (����ṹ��ȫһ����ֻ�ǲ����ı�����ͬ)
void parse_uart2_data(const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        if (g_is_u2_receiving)
        {
            if (byte == FRAME_TAIL)
            {
                g_u2_cmd_buffer[g_u2_cmd_len] = '\0';
                process_command_u2(g_u2_cmd_buffer);
                g_is_u2_receiving = 0;
                g_u2_cmd_len = 0;
            }
            else if (byte == FRAME_HEADER)
            {
                g_u2_cmd_len = 0;
            }
            else if (g_u2_cmd_len < CMD_BUFFER_SIZE - 1)
            {
                g_u2_cmd_buffer[g_u2_cmd_len++] = byte;
            }
            else
            {
                g_is_u2_receiving = 0;
                g_u2_cmd_len = 0;
            }
        }
        else if (byte == FRAME_HEADER)
        {
            g_is_u2_receiving = 1;
            g_u2_cmd_len = 0;
        }
    }
}


// --- ����1 (��PCͨ��) �Ĵ������� (��׳��) ---
void uart1_task(void)
{
    uint16_t length;
    length = rt_ringbuffer_data_len(&uart1_ringbuffer);
    if(length == 0) return;
    
    // �ӻ�����ȡ�����ݵ�����������
    // ע�⣺������ length �� sizeof �Ľ�Сֵ����ֹ���
    uint16_t read_len = (length < sizeof(uart1_process_buffer)) ? length : sizeof(uart1_process_buffer);
    rt_ringbuffer_get(&uart1_ringbuffer, uart1_process_buffer, read_len);
    
    // ����״̬��������������һ���ֽ���
    parse_uart1_data(uart1_process_buffer, read_len);
    
    // �����Ѿ�������Ҫ memset �ˣ���Ϊ���������������������������
}

// --- ����2 (������ģ��ͨ��) �Ĵ������� (��׳��) ---
void uart2_task(void)
{
    uint16_t length;
    length = rt_ringbuffer_data_len(&uart2_ringbuffer);
    if(length == 0) return;
    
    uint16_t read_len = (length < sizeof(uart2_process_buffer)) ? length : sizeof(uart2_process_buffer);
    rt_ringbuffer_get(&uart2_ringbuffer, uart2_process_buffer, read_len);
    
    // ���ô���2��״̬��������
    parse_uart2_data(uart2_process_buffer, read_len);
}
