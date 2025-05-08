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
#include "delay.h" // ���� delay.h �ṩ�� delay_ms �� delay_us
#include <stdio.h>
#include <stdarg.h>

// SPI ���Ŷ��� (W25Q128 on SPI2 based on environment.md)
#define W25QXX_SPI_PERIPH              SPI2
#define W25QXX_SPI_RCC_APB1Periph      RCC_APB1Periph_SPI2
#define W25QXX_GPIO_RCC_APB2Periph     RCC_APB2Periph_GPIOB

#define W25QXX_SPI_PORT                GPIOB
#define W25QXX_CS_PIN                  GPIO_Pin_12
#define W25QXX_CLK_PIN                 GPIO_Pin_13
#define W25QXX_MISO_PIN                GPIO_Pin_14
#define W25QXX_MOSI_PIN                GPIO_Pin_15

// CSƬѡ��
#define W25QXX_CS_LOW()                GPIO_ResetBits(W25QXX_SPI_PORT, W25QXX_CS_PIN)
#define W25QXX_CS_HIGH()               GPIO_SetBits(W25QXX_SPI_PORT, W25QXX_CS_PIN)

// SPI ���ͺͽ���һ���ֽڵĸ�������
static uint8_t w25qxx_spi_send_receive_byte(uint8_t byte_to_send)
{
    /* �ȴ����ͻ�����Ϊ�� */
    while (SPI_I2S_GetFlagStatus(W25QXX_SPI_PERIPH, SPI_I2S_FLAG_TXE) == RESET);
    /* �������� */
    SPI_I2S_SendData(W25QXX_SPI_PERIPH, byte_to_send);
    /* �ȴ����ջ������ǿ� */
    while (SPI_I2S_GetFlagStatus(W25QXX_SPI_PERIPH, SPI_I2S_FLAG_RXNE) == RESET);
    /* ���ؽ��յ������� */
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
 * 4. ???QSPI???????????????/????????????��????
 */
uint8_t w25qxx_interface_spi_qspi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    /* ʹ�� GPIOB �� SPI2 ʱ�� */
    RCC_APB2PeriphClockCmd(W25QXX_GPIO_RCC_APB2Periph, ENABLE);
    RCC_APB1PeriphClockCmd(W25QXX_SPI_RCC_APB1Periph, ENABLE);

    /* ���� SPI2 ����: SCK(PB13), MISO(PB14), MOSI(PB15) */
    GPIO_InitStructure.GPIO_Pin = W25QXX_CLK_PIN | W25QXX_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = W25QXX_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // ��������
    GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    /* ���� CS ���� (PB12) */
    GPIO_InitStructure.GPIO_Pin = W25QXX_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    /* ��ʼ��ʱ���� CS */
    W25QXX_CS_HIGH();

    /* SPI2 ���� */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // ˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                    // ��ģʽ
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                // 8λ����֡
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                     // ʱ�Ӽ��ԣ�����ʱ�͵�ƽ (W25QXX֧��ģʽ0��3, ����ѡģʽ0)
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                   // ʱ����λ����һ�����ز��� (W25QXX֧��ģʽ0��3, ����ѡģʽ0)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                      // NSS�������
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // ������Ԥ��Ƶ (APB1_CLK/4, e.g. 36MHz/4 = 9MHz)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;               // MSB��ǰ
    SPI_InitStructure.SPI_CRCPolynomial = 7;                         // CRC����ʽ��ͨ����ʹ��
    SPI_Init(W25QXX_SPI_PERIPH, &SPI_InitStructure);

    /* ʹ�� SPI2 */
    SPI_Cmd(W25QXX_SPI_PERIPH, ENABLE);
    
    /* ��ʼ����һ���ֽ���ȷ��SPI�����ȶ�����ѡ */
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
 * 2. ??��GPIO???????
 * 3. ???DMA???????????????
 */
uint8_t w25qxx_interface_spi_qspi_deinit(void)
{
    /* ʧ�� SPI2 */
    SPI_Cmd(W25QXX_SPI_PERIPH, DISABLE);
    /* SPI2 ���踴λ */
    SPI_I2S_DeInit(W25QXX_SPI_PERIPH);
    /* �ر� SPI2 ʱ�� */
    RCC_APB1PeriphClockCmd(W25QXX_SPI_RCC_APB1Periph, DISABLE);
    
    // ����ѡ��GPIO���Żָ���Ĭ��״̬������ģ�����룬�Խ��͹���
    // GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO_InitStructure.GPIO_Pin = W25QXX_CLK_PIN | W25QXX_MISO_PIN | W25QXX_MOSI_PIN | W25QXX_CS_PIN;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    // GPIO_Init(W25QXX_SPI_PORT, &GPIO_InitStructure);

    return 0;
}

/**
 * @brief      SPI/QSPI��д����
 * @param[in]  instruction ָ���ֽ�
 * @param[in]  instruction_line ָ������(1/2/4��)
 * @param[in]  address ��ַ����(3/4�ֽ�)
 * @param[in]  address_line ��ַ����
 * @param[in]  address_len ��ַ����
 * @param[in]  alternate �����ֽ�����
 * @param[in]  alternate_line �����ֽ�����
 * @param[in]  alternate_len �����ֽڳ���
 * @param[in]  dummy ��������
 * @param[in]  *in_buf �������ݻ�����
 * @param[in]  in_len �������ݳ���
 * @param[out] *out_buf ������ݻ�����
 * @param[in]  out_len ������ݳ���
 * @param[in]  data_line ��������
 * @return     ״̬��
 *             - 0 �ɹ�
 *             - 1 ʧ��
 * @note       
 * 1. ����ָ��/��ַ/����˳�����SPI����
 * 2. ���ڶ���ģʽ��Ҫ��ȷ������������
 * 3. ��ѡʹ��DMA�������Ч��
 */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line)
{
    // ���ڱ�׼��SPI Flash (��W25QXX)��ͨ�� instruction_line, address_line, data_line ��Ϊ1 (����SPI)
    // QSPI�ĸ���ģʽ�л��ڴ˻���SPI�����в�ֱ��֧�֣�����ʵ�ֱ�׼SPI����
    // instruction_line, address_line, alternate_line, data_line �����ڴ˼�ʵ����δʹ�ã��ٶ�Ϊ1��

    uint32_t i;

    W25QXX_CS_LOW();

    /* ����ָ�� */
    if (instruction_line > 0) // ȷ��ָ����Ч
    {
         w25qxx_spi_send_receive_byte(instruction);
    }

    /* ���͵�ַ */
    if (address_len > 0)
    {
        for (i = 0; i < address_len; i++)
        {
            w25qxx_spi_send_receive_byte((address >> (8 * (address_len - 1 - i))) & 0xFF);
        }
    }

    /* ���ͱ����ֽ� (Alternate Bytes) - W25QXX��׼�����в����� */
    if (alternate_len > 0)
    {
        // ���ݾ���Flash�ͺźͲ���ʵ�֣�����򻯴���
        // for (i = 0; i < alternate_len; i++)
        // {
        //     w25qxx_spi_send_receive_byte( (alternate >> (8 * (alternate_len - 1 - i))) & 0xFF );
        // }
    }

    /* ���Ϳ����� (Dummy Cycles) */
    for (i = 0; i < dummy; i++)
    {
        w25qxx_spi_send_receive_byte(0xFF); // ͨ������0xFF��Ϊdummy byte
    }

    /* ����д�� */
    if (in_buf != NULL && in_len > 0)
    {
        for (i = 0; i < in_len; i++)
        {
            w25qxx_spi_send_receive_byte(in_buf[i]);
        }
    }

    /* ���ݶ�ȡ */
    if (out_buf != NULL && out_len > 0)
    {
        for (i = 0; i < out_len; i++)
        {
            out_buf[i] = w25qxx_spi_send_receive_byte(0xFF); // ����dummy byte�Խ�������
        }
    }

    W25QXX_CS_HIGH();

    return 0; // �ɹ�
}

/**
 * @brief     ���뼶��ʱ
 * @param[in] ms ��ʱ������
 * @note      
 * 1. ����SysTick��ʱ��ʵ��
 * 2. Ҳ��ʹ��TIM2��Ӳ����ʱ��
 * 3. ȷ����ʱ��������Ҫ��
 */
void w25qxx_interface_delay_ms(uint32_t ms)
{
    delay_ms(ms); // ���� delay.h �е� delay_ms
}

/**
 * @brief     ΢�뼶��ʱ
 * @param[in] us ��ʱ΢����
 * @note      
 * 1. ����DWT���ڼ�����ʵ��
 * 2. ʹ�þ�ȷNOPָ����ʱ
 * 3. ȷ����ʱ��������Ҫ��
 */
void w25qxx_interface_delay_us(uint32_t us)
{
    delay_us(us); // ���� delay.h �е� delay_us
}

/**
 * @brief     ??????????
 * @param[in] fmt ??????????
 * @note     ??????????��?????�
 * 1. ???USART1??????????
 * 2. ???DMA???????CPU???
 * 3. ????????????????????115200??
 * 4. ??????????????????
 */
void w25qxx_interface_debug_print(const char *const fmt, ...)
{
    // ���� USART1 �Ѿ�����ʼ�����ڵ������
    // ���� printf �Ѿ�ͨ�� usart.c �ض��� USART1
    // ���û���ض�����Ҫ�ֶ�ʵ���ַ�����
    char buffer[256];
    char *p;
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // ͨ�� USART1 ���� (���� USART1_SendChar �����ƺ��������� usart.c, �� printf ���ض���)
    // ��� printf ���ض���, ֱ�ӵ��� printf(buffer) ����
    // ����ʹ��һ���򵥵�ѭ��ͨ����׼�⺯�����ͣ�ǰ���� USART1 �������� TX ������ȷ
    // ʵ����Ŀ�У�ͨ������һ����װ�õĴ��ڷ����ַ�������

    for (p = buffer; *p; p++)
    {
        // �ȴ��������ݼĴ���Ϊ��
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        // ��������
        USART_SendData(USART1, (uint16_t)*p);
        // �ȴ��������
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
    // ע��: ���� USART ���ʹ�����Ҫȷ�� USART1 �ѱ���ȷ��ʼ����
    // �����Ŀ�� usart.c �ṩ�� printf ���ض��������ֱ��ʹ�� printf(buffer);
    // ���ߣ���� usart.h/c �ṩ���� USART1_Puts(char *str) ֮��ĺ�����Ӧ����ʹ�á�
}
