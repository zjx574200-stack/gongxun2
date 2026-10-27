#include "./ioout_app/ioout_app.h"
/**
 * @brief GPIO���ͨ�����ڲ����ݽṹ
 */
typedef struct
{
    GPIO_TypeDef *port; // GPIO �˿�
    uint16_t pin;       // GPIO ����
    uint8_t is_used;    // ͨ���Ƿ�ʹ��/���� (1:����, 0:����)
} GpioOut_Channel_t;

/* Private variables ---------------------------------------------------------*/
// ����һ����̬�������洢����ͨ����������Ϣ
static GpioOut_Channel_t s_gpio_channels[GPIO_OUT_CHANNEL_MAX];

/* Function implementations --------------------------------------------------*/

/**
 * @brief  ��ʼ��GPIO���ģ�顣
 */
void GpioOut_Init(void)
{
    for (uint8_t i = 0; i < GPIO_OUT_CHANNEL_MAX; i++)
    {
        s_gpio_channels[i].port = NULL;
        s_gpio_channels[i].pin = 0;
        s_gpio_channels[i].is_used = 0; // Ĭ��ȫ������
    }
}

/**
 * @brief  ���ò�����һ��GPIO���ͨ����
 */
void GpioOut_ConfigureChannel(uint8_t channel, GPIO_TypeDef *port, uint16_t pin)
{
    // �������
    if (channel >= GPIO_OUT_CHANNEL_MAX || port == NULL)
    {
        return; // ��Ч������ֱ�ӷ���
    }

    s_gpio_channels[channel].port = port;
    s_gpio_channels[channel].pin = pin;
    s_gpio_channels[channel].is_used = 1; // ���Ϊ����
}

/**
 * @brief  ����һ��GPIO���ͨ����
 */
void GpioOut_DisableChannel(uint8_t channel)
{
    if (channel >= GPIO_OUT_CHANNEL_MAX)
    {
        return;
    }
    s_gpio_channels[channel].is_used = 0; // ���Ϊ����
}

/**
 * @brief  ����ָ��ͨ���������ƽ��
 */
void GpioOut_SetState(uint8_t channel, GpioOut_State_e state)
{
    // ���ͨ�����Ƿ���Ч���Լ�ͨ���Ƿ�������
    if (channel >= GPIO_OUT_CHANNEL_MAX || s_gpio_channels[channel].is_used == 0)
    {
        return;
    }

    // ���� state �������� HAL �⺯��
    HAL_GPIO_WritePin(s_gpio_channels[channel].port, s_gpio_channels[channel].pin, (GPIO_PinState)state);
}

/**
 * @brief  ����ָ��ͨ�����Ϊ�ߵ�ƽ��
 */
void GpioOut_SetHigh(uint8_t channel)
{
    if (channel >= GPIO_OUT_CHANNEL_MAX || s_gpio_channels[channel].is_used == 0)
    {
        return;
    }
    HAL_GPIO_WritePin(s_gpio_channels[channel].port, s_gpio_channels[channel].pin, GPIO_PIN_SET);
}

/**
 * @brief  ����ָ��ͨ�����Ϊ�͵�ƽ��
 */
void GpioOut_SetLow(uint8_t channel)
{
    if (channel >= GPIO_OUT_CHANNEL_MAX || s_gpio_channels[channel].is_used == 0)
    {
        return;
    }
    HAL_GPIO_WritePin(s_gpio_channels[channel].port, s_gpio_channels[channel].pin, GPIO_PIN_RESET);
}

/**
 * @brief  ��תָ��ͨ���������ƽ��
 */
void GpioOut_Toggle(uint8_t channel)
{
    if (channel >= GPIO_OUT_CHANNEL_MAX || s_gpio_channels[channel].is_used == 0)
    {
        return;
    }
    HAL_GPIO_TogglePin(s_gpio_channels[channel].port, s_gpio_channels[channel].pin);
}

/**
 * @brief  ��һ����������ʹ�õĳ�ʼ������
 */

void ioout_init(void)
{
    GpioOut_Init(); // GPIO����ĳ�ʼ����Ŀ�İ�����IO�ڽ���
    /* GPIO������ */
    GpioOut_ConfigureChannel(0, GPIOC, GPIO_PIN_13); // 通道0 -> PC13
    GpioOut_ConfigureChannel(1, GPIOC, GPIO_PIN_14); // 通道1 -> PC14 (配合ebtn按键用)
}

/**
 * @brief  LED�������
 */
void led_task(void)
{

    //GpioOut_Toggle(0);  //���õ�1����Ϊ��ת��ƽ

    
}
