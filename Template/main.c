/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

    \version 2019-02-19, V1.0.0, firmware for GD32E23x
    \version 2020-12-12, V1.1.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "main.h"

#define TIMER2_CH0_ADDR (0x40000400 + 0x34)

#define GPIOA_ISTAT_ADDR (0X48000010)
#define GPIOB_ISTAT_ADDR (0X48000410)
#define GPIOC_ISTAT_ADDR (0X48000810)

#define GPIOA_BOP_ADDR (0x48000018)
#define GPIOB_BOP_ADDR (0x48000418)
#define GPIOC_BOP_ADDR (0x48000818)

uint16_t dshoot_dma_buffer[140]; // 32
uint32_t send_buffer_dshoot[54];

static DmaCache_s rec_dma_cache_;
static DmaCache_s send_dma_cache_;

static InputRegGpio_s motor_input_reg_;

/* configure the GPIO ports */
void gpio_config(void);
/* configure the TIMER peripheral */
void timer_config(void);
/* configure the TIMER interrupt */
void nvic_config(void);
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f);

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while (RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE))
        ;
    return ch;
}

void gpioInitTestPin(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_6);
    gpio_bit_set(GPIOA, GPIO_PIN_0);
    gpio_bit_set(GPIOA, GPIO_PIN_1);
    gpio_bit_set(GPIOA, GPIO_PIN_5);
    gpio_bit_set(GPIOA, GPIO_PIN_6);
}

/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_configuration_output(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
    gpio_bit_set(GPIOB, GPIO_PIN_0);
    gpio_bit_set(GPIOB, GPIO_PIN_1);
    gpio_bit_set(GPIOB, GPIO_PIN_4);
    gpio_bit_set(GPIOB, GPIO_PIN_5);
}

static void gpioRegLoad(InputRegGpio_s des, uint32_t gpio_periph)
{
    GPIO_CTL(gpio_periph) = des.ctl;
    GPIO_PUD(gpio_periph) = des.pud;
}

static void gpioRegSave(InputRegGpio_s *des, uint32_t gpio_periph)
{
    des->ctl = GPIO_CTL(gpio_periph);
    des->pud = GPIO_PUD(gpio_periph);
}

void gpio_configuration_input(void)
{
    gpioRegLoad(motor_input_reg_, GPIOB);
}

static void gpio_configuration_input_init(void)
{
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
    gpioRegSave(&motor_input_reg_, GPIOB);
}

/**
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
  */
void nvic_configuration(void)
{
    nvic_irq_enable(DMA_Channel3_4_IRQn, 0);
    // nvic_irq_enable(TIMER2_IRQn, 0);
}



void bbSaveDMARegs(DmaCache_s *des, dma_channel_enum dma_ch)
{
    des->ctl = DMA_CHCTL(dma_ch);
    des->cnt = DMA_CHCNT(dma_ch);
    des->p_addr = DMA_CHPADDR(dma_ch);
    des->m_addr = DMA_CHMADDR(dma_ch);
}

void bbLoadDMARegs(DmaCache_s source, dma_channel_enum dma_ch)
{
    DMA_CHCTL(dma_ch) = source.ctl;
    DMA_CHCNT(dma_ch) = source.cnt;
    DMA_CHPADDR(dma_ch) = source.p_addr;
    DMA_CHMADDR(dma_ch) = source.m_addr;
    dma_channel_enable(dma_ch);
}

void dma_configuration_receive_dshoot(void)
{
    bbLoadDMARegs(rec_dma_cache_, DMA_CH3);
}

void dma_configuration_send_dshoot(void)
{
    bbLoadDMARegs(send_dma_cache_, DMA_CH3);
}

void dma_configuration_receive_dshoot_init(void)
{
    dma_parameter_struct dma_init_struct;

    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);

    /* initialize DMA channel4 */
    dma_deinit(DMA_CH3);
    /* DMA channel4 initialize */
    dma_init_struct.periph_addr = (uint32_t)GPIOB_ISTAT_ADDR;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr = (uint32_t)dshoot_dma_buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.number = 140;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH3, &dma_init_struct);
    dma_interrupt_enable(DMA_CH3, DMA_INT_FTF);
    bbSaveDMARegs(&rec_dma_cache_, DMA_CH3);
}

void dma_configuration_send_dshoot_init(void)
{
    dma_parameter_struct dma_init_struct;

    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);

    /* initialize DMA channel4 */
    dma_deinit(DMA_CH3);
    /* DMA channel4 initialize */
    dma_init_struct.periph_addr = (uint32_t)GPIOB_BOP_ADDR;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr = (uint32_t)send_buffer_dshoot;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.number = 16 * 3 + 3;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH3, &dma_init_struct);
    dma_interrupt_enable(DMA_CH3, DMA_INT_FTF);
    bbSaveDMARegs(&send_dma_cache_, DMA_CH3);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_configuration_send_dshoot(void)
{

    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;
    /* enable the TIMER clock */
    rcu_periph_clock_enable(RCU_TIMER2);
    /* deinit a TIMER */
    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
    timer_initpara.prescaler = 0; // 35 72
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = (37); //
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);
    timer_channel_input_struct_para_init(&timer_icinitpara);
    timer_channel_dma_request_source_select(TIMER2, TIMER_DMAREQUEST_CHANNELEVENT);
    /* enable the TIMER DMA */
    timer_dma_enable(TIMER2, TIMER_DMA_CH0D);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* enable DMA channel4 */
    dma_channel_enable(DMA_CH3);
    /* TIMER2 counter enable */
    timer_enable(TIMER2);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_configuration_receive_dshoot(void)
{
    // timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;
    /* enable the TIMER clock */
    rcu_periph_clock_enable(RCU_TIMER2);
    /* deinit a TIMER */
    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
    timer_initpara.prescaler = 0; // 35 72
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = (31); // 32-1
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);
    /* enable the TIMER DMA */
    timer_dma_enable(TIMER2, TIMER_DMA_CH0D);
    /* TIMER2 counter enable */
    timer_enable(TIMER2);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* configure systick */
    systick_config();
    /* configure the GPIO ports */
    gpio_configuration_output();
    gpioInitTestPin();
    /* configure COM port */
    gd_eval_com_init(EVAL_COM);
    /* configure the TIMER interrupt */
    nvic_configuration();

    /* 配置通道0，上行配置*/
    SEGGER_RTT_ConfigUpBuffer(0, "RTTUP", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    /* 配置通道0，下行配置*/
    SEGGER_RTT_ConfigDownBuffer(0, "RTTDOWN", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    dma_configuration_receive_dshoot_init();
    dma_configuration_send_dshoot_init();
    gpio_configuration_input_init();

    while (1)
    {
        test_dshot600_1ms();
    }
}
