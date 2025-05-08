/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_w25qxx_interface_template.c
 * @brief     driver w25qxx interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-07-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_w25qxx_interface.h"
#include "stm32f10x.h"
#include "delay.h" // 假设 delay.h 提供了 delay_ms 和 delay_us
#include <stdio.h>
#include <stdarg.h>

// SPI 引脚定义 (W25Q128 on SPI2 based on environment.md)
#define W25QXX_SPI_PERIPH              SPI2
#define W25QXX_SPI_RCC_APB1Periph      RCC_APB1Periph_SPI2
#define W25QXX_GPIO_RCC_APB2Periph     RCC_APB2Periph_GPIOB

#define W25QXX_SPI_PORT                GPIOB
#define W25QXX_CS_PIN                  GPIO_Pin_12
#define W25QXX_CLK_PIN                 GPIO_Pin_13
#define W25QXX_MISO_PIN                GPIO_Pin_14
#define W25QXX_MOSI_PIN                GPIO_Pin_15

// CS片选宏
#define W25QXX_CS_LOW()                GPIO_ResetBits(W25QXX_SPI_PORT, W25QXX_CS_PIN)
#define W25QXX_CS_HIGH()               GPIO_SetBits(W25QXX_SPI_PORT, W25QXX_CS_PIN)

// SPI 发送和接收一个字节的辅助函数
static uint8_t w25qxx_spi_send_receive_byte(uint8_t byte_to_send)
{
    /* 等待发送缓冲区为空 */
    while (SPI_I2S_GetFlagStatus(W25QXX_SPI_PERIPH, SPI_I2S_FLAG_TXE) == RESET);
    /* 发送数据 */
    SPI_I2S_SendData(W25QXX_SPI_PERIPH, byte_to_send);
    /* 等待接收缓冲区非空 */
    while (SPI_I2S_GetFlagStatus(W25QXX_SPI_PERIPH, SPI_I2S_FLAG_RXNE) == RESET);
    /* 返回接收到的数据 */
    return SPI_I2S_ReceiveData(W25QXX_SPI_PERIPH);
}

/**
 * @brief  SPI/QSPI????????
 * @return ????
 *         - 0 ????????
 *         - 1 ????????
 * @note   
 * 1. ???SPI??????????????????????RCC???????
 * 2. ????GPIO????SCK/MISO/MOSI?????????CS????????????
 * 3. ????SPI?????????????CPOL/CPHA????MSB???????
 * 4. ???QSPI???????????????/????????????о????
 */
uint8_t w25qxx_interface_spi_qspi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    /* 使能 GPIOB 和 SPI2 时钟 */
    RCC_APB2PeriphClockCmd(W25QXX_GPIO_RCC_APB2Periph, ENABLE);
    RCC_APB1PeriphClockCmd(W25QXX_SPI_RCC_APB1Periph, ENABLE);

    /* 配置 SPI2 引脚: SCK(PB13), MISO(PB14), MOSI(PB15) */
    GPIO_InitStructure.GPIO_Pin = W25QXX_CLK_PIN | W25QXX_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = W25QXX_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    /* 配置 CS 引脚 (PB12) */
    GPIO_InitStructure.GPIO_Pin = W25QXX_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    /* 初始化时拉高 CS */
    W25QXX_CS_HIGH();

    /* SPI2 配置 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 双线全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                    // 主模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                // 8位数据帧
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                     // 时钟极性：空闲时低电平 (W25QXX支持模式0或3, 这里选模式0)
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                   // 时钟相位：第一个边沿采样 (W25QXX支持模式0或3, 这里选模式0)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                      // NSS软件管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // 波特率预分频 (APB1_CLK/4, e.g. 36MHz/4 = 9MHz)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;               // MSB在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;                         // CRC多项式，通常不使用
    SPI_Init(W25QXX_SPI_PERIPH, &SPI_InitStructure);

    /* 使能 SPI2 */
    SPI_Cmd(W25QXX_SPI_PERIPH, ENABLE);
    
    /* 初始发送一个字节以确保SPI总线稳定，可选 */
    w25qxx_spi_send_receive_byte(0xFF);

    return 0;
}

/**
 * @brief  SPI/QSPI??????????
 * @return ????
 *         - 0 ??????????
 *         - 1 ??????????
 * @note   
 * 1. ???SPI???????
 * 2. ??λGPIO???????
 * 3. ???DMA???????????????
 */
uint8_t w25qxx_interface_spi_qspi_deinit(void)
{
    /* 失能 SPI2 */
    SPI_Cmd(W25QXX_SPI_PERIPH, DISABLE);
    /* SPI2 外设复位 */
    SPI_I2S_DeInit(W25QXX_SPI_PERIPH);
    /* 关闭 SPI2 时钟 */
    RCC_APB1PeriphClockCmd(W25QXX_SPI_RCC_APB1Periph, DISABLE);
    
    // 可以选择将GPIO引脚恢复到默认状态，例如模拟输入，以降低功耗
    // GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO_InitStructure.GPIO_Pin = W25QXX_CLK_PIN | W25QXX_MISO_PIN | W25QXX_MOSI_PIN | W25QXX_CS_PIN;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    // GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    return 0;
}

/**
 * @brief      SPI/QSPI读写操作
 * @param[in]  instruction 指令字节
 * @param[in]  instruction_line 指令线数(1/2/4线)
 * @param[in]  address 地址数据(3/4字节)
 * @param[in]  address_line 地址线数
 * @param[in]  address_len 地址长度
 * @param[in]  alternate 备用字节数据
 * @param[in]  alternate_line 备用字节线数
 * @param[in]  alternate_len 备用字节长度
 * @param[in]  dummy 空周期数
 * @param[in]  *in_buf 输入数据缓冲区
 * @param[in]  in_len 输入数据长度
 * @param[out] *out_buf 输出数据缓冲区
 * @param[in]  out_len 输出数据长度
 * @param[in]  data_line 数据线数
 * @return     状态码
 *             - 0 成功
 *             - 1 失败
 * @note       
 * 1. 按照指令/地址/数据顺序进行SPI传输
 * 2. 对于多线模式需要正确配置线数参数
 * 3. 可选使用DMA传输提高效率
 */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line)
{
    // 对于标准的SPI Flash (如W25QXX)，通常 instruction_line, address_line, data_line 均为1 (单线SPI)
    // QSPI的复杂模式切换在此基础SPI驱动中不直接支持，这里实现标准SPI操作
    // instruction_line, address_line, alternate_line, data_line 参数在此简化实现中未使用，假定为1线

    uint32_t i;

    W25QXX_CS_LOW();

    /* 发送指令 */
    if (instruction_line > 0) // 确保指令有效
    {
         w25qxx_spi_send_receive_byte(instruction);
    }

    /* 发送地址 */
    if (address_len > 0)
    {
        for (i = 0; i < address_len; i++)
        {
            w25qxx_spi_send_receive_byte((address >> (8 * (address_len - 1 - i))) & 0xFF);
        }
    }

    /* 发送备用字节 (Alternate Bytes) - W25QXX标准操作中不常用 */
    if (alternate_len > 0)
    {
        // 根据具体Flash型号和操作实现，这里简化处理
        // for (i = 0; i < alternate_len; i++)
        // {
        //     w25qxx_spi_send_receive_byte( (alternate >> (8 * (alternate_len - 1 - i))) & 0xFF );
        // }
    }

    /* 发送空周期 (Dummy Cycles) */
    for (i = 0; i < dummy; i++)
    {
        w25qxx_spi_send_receive_byte(0xFF); // 通常发送0xFF作为dummy byte
    }

    /* 数据写入 */
    if (in_buf != NULL && in_len > 0)
    {
        for (i = 0; i < in_len; i++)
        {
            w25qxx_spi_send_receive_byte(in_buf[i]);
        }
    }

    /* 数据读取 */
    if (out_buf != NULL && out_len > 0)
    {
        for (i = 0; i < out_len; i++)
        {
            out_buf[i] = w25qxx_spi_send_receive_byte(0xFF); // 发送dummy byte以接收数据
        }
    }

    W25QXX_CS_HIGH();

    return 0; // 成功
}

/**
 * @brief     毫秒级延时
 * @param[in] ms 延时毫秒数
 * @note      
 * 1. 基于SysTick定时器实现
 * 2. 也可使用TIM2等硬件定时器
 * 3. 确保延时精度满足要求
 */
void w25qxx_interface_delay_ms(uint32_t ms)
{
    delay_ms(ms); // 调用 delay.h 中的 delay_ms
}

/**
 * @brief     微秒级延时
 * @param[in] us 延时微秒数
 * @note      
 * 1. 基于DWT周期计数器实现
 * 2. 使用精确NOP指令延时
 * 3. 确保延时精度满足要求
 */
void w25qxx_interface_delay_us(uint32_t us)
{
    delay_us(us); // 调用 delay.h 中的 delay_us
}

/**
 * @brief     ??????????
 * @param[in] fmt ??????????
 * @note     ??????????д?????飺
 * 1. ???USART1??????????
 * 2. ???DMA???????CPU???
 * 3. ????????????????????115200??
 * 4. ??????????????????
 */
void w25qxx_interface_debug_print(const char *const fmt, ...)
{
    // 假设 USART1 已经被初始化用于调试输出
    // 并且 printf 已经通过 usart.c 重定向到 USART1
    // 如果没有重定向，需要手动实现字符发送
    char buffer[256];
    char *p;
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // 通过 USART1 发送 (假设 USART1_SendChar 或类似函数存在于 usart.c, 或 printf 已重定向)
    // 如果 printf 已重定向, 直接调用 printf(buffer) 即可
    // 这里使用一个简单的循环通过标准库函数发送，前提是 USART1 已配置且 TX 引脚正确
    // 实际项目中，通常会有一个封装好的串口发送字符串函数

    for (p = buffer; *p; p++)
    {
        // 等待发送数据寄存器为空
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        // 发送数据
        USART_SendData(USART1, (uint16_t)*p);
        // 等待发送完成
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
    // 注意: 上述 USART 发送代码需要确保 USART1 已被正确初始化。
    // 如果项目中 usart.c 提供了 printf 的重定向，则可以直接使用 printf(buffer);
    // 或者，如果 usart.h/c 提供了如 USART1_Puts(char *str) 之类的函数，应优先使用。
}
