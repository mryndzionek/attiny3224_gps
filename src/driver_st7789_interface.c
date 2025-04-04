/**
 *Copyright (c) 2015 - present LibDriver All rights reserved
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
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_st7789_interface_template.c
 * @brief     driver st7789 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2023-04-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/04/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_st7789_interface.h"

#include <avr/io.h>
#include <util/delay.h>

#define SPI_PORT PORTA
#define SPI_MOSI_PIN PIN5_bm
#define SPI_MISO_PIN PIN6_bm
#define SPI_CLK_PIN PIN7_bm

#define DISP_WIRE_PORT PORTB
#define WIRE_CMD_PIN PIN0_bm
#define WIRE_RESET_PIN PIN1_bm

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t st7789_interface_spi_init(void) {
  SPI0.CTRLB = SPI_MODE_0_gc;
  SPI0.CTRLA = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESC_DIV4_gc;

  PORTA.DIR |= PIN1_bm;   // MOSI channel
  PORTA.DIR &= ~PIN2_bm;  // MISO channel
  PORTA.DIR |= PIN3_bm;   // SCK channel
  return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t st7789_interface_spi_deinit(void) { return 0; }

/**
 * @brief     interface spi bus write
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t st7789_interface_spi_write_cmd(uint8_t *buf, uint16_t len) {
  SPI0.CTRLB |= SPI_BUFEN_bm;
  for (uint16_t i = 0; i < len; i++) {
    SPI0.DATA = buf[i];
    while (!(SPI0.INTFLAGS & SPI_DREIF_bm)) {
      ;
    }
  }
  while (!(SPI0.INTFLAGS & SPI_IF_bm)) {
    ;
  }
  SPI0.CTRLB &= ~SPI_BUFEN_bm;
  return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void st7789_interface_delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    _delay_ms(1);
  }
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void st7789_interface_debug_print(const char *const fmt, ...) { (void)fmt; }

/**
 * @brief  interface command && data gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t st7789_interface_cmd_data_gpio_init(void) {
  DISP_WIRE_PORT.DIR |= WIRE_CMD_PIN;
  return 0;
}

/**
 * @brief  interface command && data gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t st7789_interface_cmd_data_gpio_deinit(void) { return 0; }

/**
 * @brief     interface command && data gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t st7789_interface_cmd_data_gpio_write(uint8_t value) {
  if (value) {
    DISP_WIRE_PORT.OUTSET = WIRE_CMD_PIN;
  } else {
    DISP_WIRE_PORT.OUTCLR = WIRE_CMD_PIN;
  }
  return 0;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t st7789_interface_reset_gpio_init(void) {
  DISP_WIRE_PORT.DIR |= WIRE_RESET_PIN;
  return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t st7789_interface_reset_gpio_deinit(void) { return 0; }

/**
 * @brief     interface reset gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t st7789_interface_reset_gpio_write(uint8_t value) {
  if (value) {
    DISP_WIRE_PORT.OUTSET = WIRE_RESET_PIN;
  } else {
    DISP_WIRE_PORT.OUTCLR = WIRE_RESET_PIN;
  }
  return 0;
}
