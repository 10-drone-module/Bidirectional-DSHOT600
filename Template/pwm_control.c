/* Includes ------------------------------------------------------------------*/
#include "pwm_control.h"
#include "SEGGER_RTT.h"
#include "gd32e23x.h"
#include "main.h"
#include "systick.h"
#include <string.h>

#define ESC_CMD_BUFFER_LEN 18
#define ESC_BIT_0 0
#define ESC_BIT_1 1

extern uint16_t dshoot_dma_buffer[140]; // 32
extern uint32_t send_buffer_dshoot[54];

uint16_t dshot_cmd_ch[MOTOR_NUM][ESC_CMD_BUFFER_LEN] = {0};
uint8_t dshoot_pin[MOTOR_NUM] = {0, 1, 4, 5};
uint32_t bit_dshoot_cmd_data[MOTOR_NUM + 1][ESC_CMD_BUFFER_LEN * 3] = {0};

uint8_t dma_status = 0;
uint16_t speed_rec_ = 0;
static motor_speed_str motor_speed;

uint32_t success_cnt = 0;
uint32_t fail_cnt1, fail_cnt2, fail_cnt3;
float percent;
uint32_t run_cnt = 0;

void dshotEncode(uint8_t *channel, uint16_t (*dshot_data)[18], uint32_t (*data)[54])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        // data head and end respectively pull high
        data[i][0] = 0x01U << (channel[i]);
        data[i][1] = 0x01U << (channel[i]);
        data[i][2] = 0x01U << (channel[i]);
        data[i][51] = 0x01U << (channel[i]);
        data[i][52] = 0x01U << (channel[i]);
        data[i][53] = 0x01U << (channel[i]);

        for (uint8_t j = 0; j < 16; j++)
        {
            if (dshot_data[i][j])
            {
                data[i][3 + j * 3] = 0x01U << (channel[i] + 16);
                data[i][3 + j * 3 + 1] = 0x01U << (channel[i] + 16);
                data[i][3 + j * 3 + 2] = 0x01U << (channel[i]);
            }
            else
            {
                data[i][3 + j * 3] = 0x01U << (channel[i] + 16);
                data[i][3 + j * 3 + 1] = 0x01U << (channel[i]);
                data[i][3 + j * 3 + 2] = 0x01U << (channel[i]);
            }
        }
    }

    for (uint8_t i = 0; i < 54; i++)
    {
        data[4][i] = data[0][i] | data[1][i] | data[2][i] | data[3][i];
    }
}

uint32_t dshotDecode(uint32_t buffer[], uint32_t count)  
{  
    volatile uint32_t value = 0; // 存储解码后的值  
    uint32_t oldValue = buffer[0]; // 记录前一个值，初始为buffer的第一个元素  
    int bits = 0; // 记录已处理的比特位数  
    int len; // 长度临时变量，用于计算当前段的长度  

    run_cnt++; // 运行计数增加  

    // 遍历buffer，从第一个元素到最后一个元素  
    for (uint32_t i = 1; i <= count; i++)  
    {  
        if (i < count)  
        {  
            int diff = buffer[i] - oldValue; // 计算当前值与上一个值的差  
            if (bits >= 21) // 如果比特数已达到21位  
            {  
                fail_cnt1++; // 增加故障计数  
                break; // 退出循环  
            }  
            len = (diff + 3) / 6; // 根据差值计算当前段的长度（单位为6us）  
        }  
        else  
        {  
            len = 21 - bits; // 最后的比特段长度计算  
        }  

        value <<= len; // 将已解码值左移len位以腾出位置  
        value |= 1 << (len - 1); // 在当前长度的位置上设置比特  
        oldValue = buffer[i]; // 更新上一个值为当前值  
        bits += len; // 更新已处理的比特数量  
    }  

    // 如果处理的比特数量不等于21，计数增加  
    if (bits != 21)  
    {  
        fail_cnt2++; // 增加故障计数  
        return 0xffff; // 返回错误标志  
    }  

    // 解码映射表  
    static const uint32_t decode[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,  
                                        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8,  1,  0, 4,  12, 0};  

    // 根据前21位解析出对应的值  
    volatile uint32_t decodedValue = decode[value & 0x1f]; // 取value的低5位  
    decodedValue |= decode[(value >> 5) & 0x1f] << 4; // 取下一个5位并左移4位  
    decodedValue |= decode[(value >> 10) & 0x1f] << 8; // 再取下一个5位并左移8位  
    decodedValue |= decode[(value >> 15) & 0x1f] << 12; // 取最后5位并左移12位  

    uint32_t csum = decodedValue; // 计算校验和  
    csum = csum ^ (csum >> 8); // 对字节进行异或运算  
    csum = csum ^ (csum >> 4); // 对半字节进行异或运算  

    // 校验和检查  
    if ((csum & 0xf) != 0xf) // 检查校验和的最后4位是否为1111  
    {  
        // if (start_flag) { // 原注释，可能用于调试标志  
        fail_cnt3++; // 增加故障计数  
        // }  
        return 0xffff; // 返回错误标志  
    }  
    
    decodedValue >>= 4; // 右移4位，获取有效数据位  

    // 如果解码值仍为0x0fff，返回0  
    if (decodedValue == 0x0fff)  
    {  
        return 0; // 特殊情况处理，返回0  
    }  

    // 将解码值进行位移和处理，得到最终的值  
    decodedValue = (decodedValue & 0x000001ff) << ((decodedValue & 0xfffffe00) >> 9);   
    if (!decodedValue) // 如果解码值为0，则返回0xffff  
    {  
        return 0xffff; // 返回错误标志  
    }  

    // 计算最终值  
    uint32_t ret = (1000000 * 60 / 100 + decodedValue / 2) / decodedValue;   
    // volatile float erpm = ret * 100; // 计算电机转速  
    volatile float rpm = ret * 100 * 2 / 7; // 计算另一种转速表达方式  
    success_cnt++; // 成功计数增加  

    // 计算故障率百分比  
    // percent = ((fail_cnt1+fail_cnt2+fail_cnt3)*100)/(success_cnt+fail_cnt1+fail_cnt2+fail_cnt3);  
    percent = ((fail_cnt2 + fail_cnt3) * 100) / (run_cnt); // 更新故障率  

    return ret; // 返回解码后的值  
}

void dshotDecodePre(uint8_t *channel, uint16_t *data)  
{  
    uint32_t speed_temp = 0; // 临时变量，用于存储计算的速度  
    edge_data_str edge_data = {0}; // 初始化边缘数据结构  
    edge_dispose_str edge_dispose = {0}; // 初始化边缘处理数据结构  

    // 遍历4个通道  
    for (uint8_t i = 0; i < 1; i++)  
    {  
        edge_dispose.old_hight_low_status = 0; // 记录上一个电平状态  
        edge_dispose.hight_low_status = 0; // 当前电平状态  
        edge_dispose.start_flag = 0; // 开始信号标志  
        edge_dispose.signal_cnt = 0; // 信号计数  
        edge_dispose.vessel_number = 0; // 边缘数据索引  

        // 遍历数据中的位，最多140位  
        for (int j = 0; j < 140; j++)  
        {  
            // 检查当前位是否为低电平  
            if (!(data[j] >> channel[i] & 1))  
            {  
                edge_dispose.hight_low_status = 1; // 设置为高电平  
                edge_dispose.start_flag = 1; // 设置开始信号标志  
            }  
            else  
            {  
                edge_dispose.hight_low_status = 0; // 设置为低电平  
            }  

            // 如果当前位为低电平，增加信号计数  
            if (edge_dispose.start_flag)  
            {  
                edge_dispose.signal_cnt++; // 记录信号的发生次数  
            }  

            // 如果电平状态发生变化  
            if (edge_dispose.old_hight_low_status != edge_dispose.hight_low_status)  
            {  
                // 记录边缘信号数据，并更新信号计数  
                edge_data.edge[i][edge_dispose.vessel_number++] = (edge_dispose.signal_cnt - 1) * 2; // 计算信号位置  
                edge_data.edge_cnt[i]++; // 增加当前通道的信号计数  
            }  

            // 更新之前的电平状态  
            edge_dispose.old_hight_low_status = edge_dispose.hight_low_status;  
        }  

        // 调用解码函数，得到电机速度  
        speed_temp = dshotDecode(&edge_data.edge[i][0], edge_data.edge_cnt[i]);  
        
        // 如果速度有效，则更新电机速度  
        if (speed_temp != 65535)  
        {  
            motor_speed.speed[i] = speed_temp; // 将计算的速度保存到电机速度数组中  
        }  
    }  
}


uint16_t add_checksum_and_telemetry_double(uint16_t packet)
{
    uint16_t packet_telemetry = (packet << 1) | 0;
    uint8_t i;
    int crc = 0;
    int csum_data = packet_telemetry;
    crc = (~(csum_data ^ (csum_data >> 4) ^ (csum_data >> 8))) & 0x0F;
    packet_telemetry = (packet_telemetry << 4) | crc;
    return packet_telemetry; // append checksum
}

static void pwmWriteDigital(uint16_t *esc_cmd, uint16_t value)
{
    value = ((value > 2047) ? 2047 : value);
    volatile uint16_t value1 = add_checksum_and_telemetry_double(value);
    esc_cmd[0] = (value1 & 0x8000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[1] = (value1 & 0x4000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[2] = (value1 & 0x2000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[3] = (value1 & 0x1000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[4] = (value1 & 0x0800) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[5] = (value1 & 0x0400) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[6] = (value1 & 0x0200) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[7] = (value1 & 0x0100) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[8] = (value1 & 0x0080) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[9] = (value1 & 0x0040) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[10] = (value1 & 0x0020) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[11] = (value1 & 0x0010) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[12] = (value1 & 0x8) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[13] = (value1 & 0x4) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[14] = (value1 & 0x2) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[15] = (value1 & 0x1) ? ESC_BIT_1 : ESC_BIT_0;
}

static void pwm_control_protocol(uint16_t *pwm)
{
    for (uint8_t i = 0; i < PWM_MAX; i++)
    {
        pwmWriteDigital(dshot_cmd_ch[i], pwm[i]);
    }
}

static void printfRpm(void)
{
    static uint16_t count = 0;
    if (count > 1000)
    {
        count = 0;
        SEGGER_RTT_SetTerminal(0);
        SEGGER_RTT_printf(0, "RPM %d %d %d %d,set_speed %d\r\n", motor_speed.speed[0], motor_speed.speed[1],
                          motor_speed.speed[2], motor_speed.speed[3], speed_rec_);

        SEGGER_RTT_printf(0, "run_cnt:%d,success_cnt:%d ,fail_cnt1:%d,fail_cnt2:%d,fail_cnt3:%d \r\n", run_cnt,
                          success_cnt, fail_cnt1, fail_cnt2, fail_cnt3);

        run_cnt = 0;
        success_cnt = 0;
        fail_cnt1 = 0;
        fail_cnt2 = 0;
        fail_cnt3 = 0;
        percent = 0;
    }
    else
    {
        count++;
    }
}

static void changeSpeedBySegger(void)
{
    uint8_t get_value_speed = 0;
    if (SEGGER_RTT_HasKey())
    {
        get_value_speed = SEGGER_RTT_GetKey();
        SEGGER_RTT_SetTerminal(0);
        SEGGER_RTT_printf(0, "SEGGER_RTT_GetKey = %d\r\n", get_value_speed);
        switch (get_value_speed)
        {
        case 49:
            speed_rec_ = 0; // stop;
            break;
        case 50:
            speed_rec_ = 100;
            break;
        case 51:
            speed_rec_ = 200;
            break;
        case 52:
            speed_rec_ = 300;
            break;
        case 53:
            speed_rec_ = 400;
            break;
        default:
            break;
        }
    }
}

static void setMotorSpeed(uint16_t *speed)
{
    pwm_control_protocol(speed);
    dshotEncode(dshoot_pin, dshot_cmd_ch, bit_dshoot_cmd_data);
    memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
}

static void speedConvertBuf(uint16_t speed)
{
    uint16_t buf[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        buf[i] = speed;
    }
    setMotorSpeed(buf);
}

static void readySendIoDmaTimer(void)
{
    dma_status = 0;
    gpio_configuration_output();
    dma_configuration_send_dshoot();
    timer_configuration_send_dshoot();
}

static void sendDshot(void)
{
    speedConvertBuf(speed_rec_);
    readySendIoDmaTimer();
    gpio_bit_set(GPIOA, GPIO_PIN_6);
}

void test_dshot600_1ms(void)
{
    delay_1ms(1);
    sendDshot();
    printfRpm();
    changeSpeedBySegger();
}

void TIMER2_IRQHandler(void)
{
    static uint8_t timer_io_toggle = 0;
    if (timer_flag_get(TIMER2, TIMER_FLAG_CH0))
    {
        timer_flag_clear(TIMER2, TIMER_FLAG_CH0);
        if (timer_io_toggle)
        {
            gpio_bit_set(GPIOA, GPIO_PIN_1);
            timer_io_toggle = 0;
        }
        else
        {
            gpio_bit_reset(GPIOA, GPIO_PIN_1);
            timer_io_toggle = 1;
        }
    }
}

void DMA_Channel3_4_IRQHandler(void)
{
    if (SET == dma_flag_get(DMA_CH3, DMA_FLAG_FTF))
    {
        timer_disable(TIMER2);
        dma_channel_disable(DMA_CH3);
        dma_flag_clear(DMA_CH3, DMA_FLAG_FTF);
        if (dma_status == 0)
        {
            __set_PRIMASK(1);
            gpio_configuration_input();
            dma_configuration_receive_dshoot();
            gpio_bit_reset(GPIOA, GPIO_PIN_6);
            timer_configuration_receive_dshoot();
            __set_PRIMASK(0);
            gpio_bit_set(GPIOA, GPIO_PIN_1);
            dma_status = 1;
        }
        else if (dma_status == 1)
        {
            gpio_bit_reset(GPIOA, GPIO_PIN_1);
            gpio_bit_set(GPIOA, GPIO_PIN_5);            
            dshotDecodePre(dshoot_pin, dshoot_dma_buffer);
            gpio_bit_reset(GPIOA, GPIO_PIN_5);
        }
    }
}
