#include "display_driver.h"

#include "driver_st7789.h"
#include "driver_st7789_interface.h"

#define ST7789_BASIC_DEFAULT_POSITIVE_VOLTAGE_GAMMA \
  {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,        \
   0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23} /**< gamma */
#define ST7789_BASIC_DEFAULT_NEGATIVA_VOLTAGE_GAMMA \
  {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,        \
   0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23} /**< gamma */

#define ST7789_BASIC_DEFAULT_GAMMA_CURVE ST7789_GAMMA_CURVE_1
#define ST7789_BASIC_DEFAULT_ACCESS                                      \
  (ST7789_ORDER_PAGE_TOP_TO_BOTTOM | ST7789_ORDER_COLUMN_LEFT_TO_RIGHT | \
   ST7789_ORDER_PAGE_COLUMN_NORMAL | ST7789_ORDER_LINE_TOP_TO_BOTTOM |   \
   ST7789_ORDER_COLOR_RGB | ST7789_ORDER_REFRESH_LEFT_TO_RIGHT) /**< access */
#define ST7789_BASIC_DEFAULT_RGB_INTERFACE_COLOR_FORMAT \
  ST7789_RGB_INTERFACE_COLOR_FORMAT_262K /**< 262K color format */
#define ST7789_BASIC_DEFAULT_CONTROL_INTERFACE_COLOR_FORMAT \
  ST7789_CONTROL_INTERFACE_COLOR_FORMAT_16_BIT /**< 16bit color format */
#define ST7789_BASIC_DEFAULT_BRIGHTNESS 0xFF   /**< 0xFF brightness */
#define ST7789_BASIC_DEFAULT_BRIGHTNESS_BLOCK \
  ST7789_BOOL_FALSE /**< disable brightness block */
#define ST7789_BASIC_DEFAULT_DISPLAY_DIMMING \
  ST7789_BOOL_FALSE /**< disable display dimming */
#define ST7789_BASIC_DEFAULT_BACKLIGHT \
  ST7789_BOOL_FALSE /**< disable backlight */
#define ST7789_BASIC_DEFAULT_COLOR_ENHANCEMENT \
  ST7789_BOOL_TRUE /**< enable color enhancement */
#define ST7789_BASIC_DEFAULT_COLOR_ENHANCEMENT_MODE \
  ST7789_COLOR_ENHANCEMENT_MODE_USER_INTERFACE /**< user interface */
#define ST7789_BASIC_DEFAULT_COLOR_ENHANCEMENT_LEVEL \
  ST7789_COLOR_ENHANCEMENT_LEVEL_HIGH                     /**< high level */
#define ST7789_BASIC_DEFAULT_CABC_MINIMUM_BRIGHTNESS 0x00 /**< 0x00 */
#define ST7789_BASIC_DEFAULT_RAM_ACCESS \
  ST7789_RAM_ACCESS_MCU /**< mcu access \
                         */
#define ST7789_BASIC_DEFAULT_DISPLAY_MODE \
  ST7789_DISPLAY_MODE_MCU /**< mcu display mode */
#define ST7789_BASIC_DEFAULT_FRAME_TYPE \
  ST7789_FRAME_TYPE_0 /**< frame type 0 \
                       */
#define ST7789_BASIC_DEFAULT_DATA_MODE \
  ST7789_DATA_MODE_MSB /**< data mode msb */
#define ST7789_BASIC_DEFAULT_RGB_BUS_WIDTH \
  ST7789_RGB_BUS_WIDTH_18_BIT /**< 18 bits */
#define ST7789_BASIC_DEFAULT_PIXEL_TYPE \
  ST7789_PIXEL_TYPE_0 /**< pixel type 0 \
                       */
#define ST7789_BASIC_DEFAULT_DIRECT_RGB_MODE \
  ST7789_DIRECT_RGB_MODE_MEM /**< rgb mode mem */
#define ST7789_BASIC_DEFAULT_RGB_IF_ENABLE_MODE \
  ST7789_RGB_IF_ENABLE_MODE_MCU                        /**< enable mode mcu */
#define ST7789_BASIC_DEFAULT_VSPL ST7789_PIN_LEVEL_LOW /**< level low */
#define ST7789_BASIC_DEFAULT_HSPL ST7789_PIN_LEVEL_LOW /**< level low */
#define ST7789_BASIC_DEFAULT_DPL ST7789_PIN_LEVEL_LOW  /**< level low */
#define ST7789_BASIC_DEFAULT_EPL ST7789_PIN_LEVEL_LOW  /**< level low */
#define ST7789_BASIC_DEFAULT_VBP 0x02                  /**< 0x02 */
#define ST7789_BASIC_DEFAULT_HBP 0x14                  /**< 0x14 */
#define ST7789_BASIC_DEFAULT_PORCH_NORMAL_BACK 0x0C    /**< 0x0C */
#define ST7789_BASIC_DEFAULT_PORCH_NORMAL_FRONT 0x0C   /**< 0x0C */
#define ST7789_BASIC_DEFAULT_PORCH_ENABLE \
  ST7789_BOOL_FALSE                                        /**< disable porch */
#define ST7789_BASIC_DEFAULT_PORCH_IDEL_BACK 0x03          /**< 0x03 */
#define ST7789_BASIC_DEFAULT_PORCH_IDEL_FRONT 0x03         /**< 0x03 */
#define ST7789_BASIC_DEFAULT_PORCH_PART_BACK 0x03          /**< 0x03 */
#define ST7789_BASIC_DEFAULT_PORCH_PART_FRONT 0x03         /**< 0x03 */
#define ST7789_BASIC_DEFAULT_SEPARATE_FR ST7789_BOOL_FALSE /**< disable fr */
#define ST7789_BASIC_DEFAULT_FRAME_RATE_DIVIDED \
  ST7789_FRAME_RATE_DIVIDED_CONTROL_DIV_1 /**< div 1 */
#define ST7789_BASIC_DEFAULT_INVERSION_IDLE_MODE \
  ST7789_INVERSION_IDLE_MODE_DOT                  /**< dot mode */
#define ST7789_BASIC_DEFAULT_IDLE_FRAME_RATE 0x0F /**< 0x0F */
#define ST7789_BASIC_DEFAULT_INVERSION_PARTIAL_MODE \
  ST7789_INVERSION_PARTIAL_MODE_DOT                 /**< dot mode */
#define ST7789_BASIC_DEFAULT_IDLE_PARTIAL_RATE 0x0F /**< 0x0F */
#define ST7789_BASIC_DEFAULT_NON_DISPLAY_SOURCE_OUTPUT_LEVEL \
  ST7789_NON_DISPLAY_SOURCE_OUTPUT_LEVEL_V63 /**< v63 */
#define ST7789_BASIC_DEFAULT_NON_DISPLAY_AREA_SCAN_MODE \
  ST7789_NON_DISPLAY_AREA_SCAN_MODE_NORMAL /**< normal scan mode */
#define ST7789_BASIC_DEFAULT_NON_DISPLAY_FRAME_FREQUENCY \
  ST7789_NON_DISPLAY_FRAME_FREQUENCY_EVERY            /**< every frame */
#define ST7789_BASIC_DEFAULT_VGHS ST7789_VGHS_14P97_V /**< 14.97V */
#define ST7789_BASIC_DEFAULT_VGLS_NEGATIVE \
  ST7789_VGLS_NEGATIVE_8P23                           /**< -8.23 */
#define ST7789_BASIC_DEFAULT_GATE_ON_TIMING 0x22      /**< 0x22 */
#define ST7789_BASIC_DEFAULT_GATE_OFF_TIMING_RGB 0x07 /**< 0x07 */
#define ST7789_BASIC_DEFAULT_GATE_OFF_TIMING 0x05     /**< 0x05 */
#define ST7789_BASIC_DEFAULT_DIGITAL_GAMMA \
  ST7789_BOOL_TRUE                                  /**< enable digital gamma */
#define ST7789_BASIC_DEFAULT_VCOMS 1.625f           /**< 1.625 vcoms */
#define ST7789_BASIC_DEFAULT_XMY ST7789_BOOL_FALSE  /**< disable xmy */
#define ST7789_BASIC_DEFAULT_XBGR ST7789_BOOL_TRUE  /**< enable xbgr */
#define ST7789_BASIC_DEFAULT_XINV ST7789_BOOL_FALSE /**< disable xinv */
#define ST7789_BASIC_DEFAULT_XMX ST7789_BOOL_TRUE   /**< enable xmx */
#define ST7789_BASIC_DEFAULT_XMH ST7789_BOOL_TRUE   /**< enable xmh */
#define ST7789_BASIC_DEFAULT_XMV ST7789_BOOL_FALSE  /**< disable xmv */
#define ST7789_BASIC_DEFAULT_XGS ST7789_BOOL_FALSE  /**< disable xgs */
#define ST7789_BASIC_DEFAULT_VDV_VRH_FROM \
  ST7789_VDV_VRH_FROM_CMD                      /**< from cmd */
#define ST7789_BASIC_DEFAULT_VRHS 4.8f         /**< 4.8 */
#define ST7789_BASIC_DEFAULT_VDV 0.0f          /**< 0.0 */
#define ST7789_BASIC_DEFAULT_VCOMS_OFFSET 0.0f /**< 0.0 */
#define ST7789_BASIC_DEFAULT_INVERSION_SELECTION \
  ST7789_INVERSION_SELECTION_DOT /**< dot */
#define ST7789_BASIC_DEFAULT_FRAME_RATE \
  ST7789_FRAME_RATE_60_HZ                             /**< frame rate 60Hz */
#define ST7789_BASIC_DEFAULT_LED_ON ST7789_BOOL_FALSE /**< disable led on */
#define ST7789_BASIC_DEFAULT_LED_PWM_INIT \
  ST7789_BOOL_FALSE /**< disable led pwm init */
#define ST7789_BASIC_DEFAULT_LED_PWM_FIX \
  ST7789_BOOL_FALSE /**< disable led pwm fix */
#define ST7789_BASIC_DEFAULT_LED_PWM_POLARITY \
  ST7789_BOOL_FALSE /**< disable led pwm polarity */
#define ST7789_BASIC_DEFAULT_PWM_FREQUENCY \
  ST7789_PWM_FREQUENCY_9P8_KHZ                      /**< pwm 9.8KHz */
#define ST7789_BASIC_DEFAULT_AVDD ST7789_AVDD_6P8_V /**< 6.8V */
#define ST7789_BASIC_DEFAULT_AVCL_NEGTIVE \
  ST7789_AVCL_NEGTIVE_4P8_V                       /**< -4.8V */
#define ST7789_BASIC_DEFAULT_VDS ST7789_VDS_2P3_V /**< 2.3V */
#define ST7789_BASIC_DEFAULT_COMMAND_2_ENABLE \
  ST7789_BOOL_FALSE /**< disable command 2 */
#define ST7789_BASIC_DEFAULT_POSITIVE_VOLTAGE_GAMMA \
  {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,        \
   0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23} /**< gamma */
#define ST7789_BASIC_DEFAULT_NEGATIVA_VOLTAGE_GAMMA \
  {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,        \
   0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}      /**< gamma */
#define ST7789_BASIC_DEFAULT_GATE_LINE 240        /**< 320 */
#define ST7789_BASIC_DEFAULT_FIRST_SCAN_LINE 0x00 /**< 0x00 */
#define ST7789_BASIC_DEFAULT_GATE_SCAN_MODE \
  ST7789_GATE_SCAN_MODE_INTERLACE /**< interlace */
#define ST7789_BASIC_DEFAULT_GATE_SCAN_DIRECTION \
  ST7789_GATE_SCAN_DIRECTION_0_319                       /**< 320 */
#define ST7789_BASIC_DEFAULT_SPI2_LANE ST7789_BOOL_FALSE /**< disable */
#define ST7789_BASIC_DEFAULT_COMMAND_TABLE_2 \
  ST7789_BOOL_FALSE /**< disable command table2 */
#define ST7789_BASIC_DEFAULT_SBCLK_DIV ST7789_SBCLK_DIV_3     /**< div3 */
#define ST7789_BASIC_DEFAULT_STP14CK_DIV ST7789_STP14CK_DIV_6 /**< div6 */
#define ST7789_BASIC_DEFAULT_SOURCE_EQUALIZE_TIME 0x11        /**< 0x11 */
#define ST7789_BASIC_DEFAULT_SOURCE_PRE_DRIVE_TIME 0x11       /**< 0x11 */
#define ST7789_BASIC_DEFAULT_GATE_EQUALIZE_TIME 0x08          /**< 0x08 */
#define ST7789_BASIC_DEFAULT_PROGRAM_MODE \
  ST7789_BOOL_FALSE /**< disable program mode */

static st7789_handle_t gs_handle;
uint8_t display_driver_init(void) {
  uint8_t res;
  uint8_t reg;
  uint16_t i;
  uint8_t param_positive[14] = ST7789_BASIC_DEFAULT_POSITIVE_VOLTAGE_GAMMA;
  uint8_t param_negative[14] = ST7789_BASIC_DEFAULT_NEGATIVA_VOLTAGE_GAMMA;
  uint8_t params[64];

  /* link functions */
  DRIVER_ST7789_LINK_INIT(&gs_handle, st7789_handle_t);
  DRIVER_ST7789_LINK_SPI_INIT(&gs_handle, st7789_interface_spi_init);
  DRIVER_ST7789_LINK_SPI_DEINIT(&gs_handle, st7789_interface_spi_deinit);
  DRIVER_ST7789_LINK_SPI_WRITE_COMMAND(&gs_handle,
                                       st7789_interface_spi_write_cmd);
  DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_INIT(
      &gs_handle, st7789_interface_cmd_data_gpio_init);
  DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_DEINIT(
      &gs_handle, st7789_interface_cmd_data_gpio_deinit);
  DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_WRITE(
      &gs_handle, st7789_interface_cmd_data_gpio_write);
  DRIVER_ST7789_LINK_RESET_GPIO_INIT(&gs_handle,
                                     st7789_interface_reset_gpio_init);
  DRIVER_ST7789_LINK_RESET_GPIO_DEINIT(&gs_handle,
                                       st7789_interface_reset_gpio_deinit);
  DRIVER_ST7789_LINK_RESET_GPIO_WRITE(&gs_handle,
                                      st7789_interface_reset_gpio_write);
  DRIVER_ST7789_LINK_DELAY_MS(&gs_handle, st7789_interface_delay_ms);
  DRIVER_ST7789_LINK_DEBUG_PRINT(&gs_handle, st7789_interface_debug_print);

  /* st7789 init */
  res = st7789_init(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: init failed.\n");

    return 1;
  }

  /* set default column */
  res = st7789_set_column(&gs_handle, DISPLAY_COLUMNS);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set column failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default row */
  res = st7789_set_row(&gs_handle, DISPLAY_ROWS);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set row failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* sleep out */
  res = st7789_sleep_out(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: sleep out failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* idle mode off */
  res = st7789_idle_mode_off(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: idle mode off failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* normal display mode on */
  res = st7789_normal_display_mode_on(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: normal display mode on failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* display inversion on */
  res = st7789_display_inversion_on(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: display inversion on failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default gamma */
  res = st7789_set_gamma(&gs_handle, ST7789_BASIC_DEFAULT_GAMMA_CURVE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set gamma failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default memory data access control */
  res = st7789_set_memory_data_access_control(&gs_handle,
                                              ST7789_BASIC_DEFAULT_ACCESS);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set memory data access control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default pixel format */
  res = st7789_set_interface_pixel_format(
      &gs_handle, ST7789_BASIC_DEFAULT_RGB_INTERFACE_COLOR_FORMAT,
      ST7789_BASIC_DEFAULT_CONTROL_INTERFACE_COLOR_FORMAT);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set interface pixel format failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default brightness */
  res = st7789_set_display_brightness(&gs_handle,
                                      ST7789_BASIC_DEFAULT_BRIGHTNESS);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set display brightness failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default brightness control */
  res = st7789_set_display_control(
      &gs_handle, ST7789_BASIC_DEFAULT_BRIGHTNESS_BLOCK,
      ST7789_BASIC_DEFAULT_DISPLAY_DIMMING, ST7789_BASIC_DEFAULT_BACKLIGHT);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set display control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default color enhancement */
  res = st7789_set_brightness_control_and_color_enhancement(
      &gs_handle, ST7789_BASIC_DEFAULT_COLOR_ENHANCEMENT,
      ST7789_BASIC_DEFAULT_COLOR_ENHANCEMENT_MODE,
      ST7789_BASIC_DEFAULT_COLOR_ENHANCEMENT_LEVEL);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set brightness control and color enhancement failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default cabc minimum brightness */
  res = st7789_set_cabc_minimum_brightness(
      &gs_handle, ST7789_BASIC_DEFAULT_CABC_MINIMUM_BRIGHTNESS);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set cabc minimum brightness failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default ram control */
  res = st7789_set_ram_control(
      &gs_handle, ST7789_BASIC_DEFAULT_RAM_ACCESS,
      ST7789_BASIC_DEFAULT_DISPLAY_MODE, ST7789_BASIC_DEFAULT_FRAME_TYPE,
      ST7789_BASIC_DEFAULT_DATA_MODE, ST7789_BASIC_DEFAULT_RGB_BUS_WIDTH,
      ST7789_BASIC_DEFAULT_PIXEL_TYPE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set ram control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default rgb interface control */
  res = st7789_set_rgb_interface_control(
      &gs_handle, ST7789_BASIC_DEFAULT_DIRECT_RGB_MODE,
      ST7789_BASIC_DEFAULT_RGB_IF_ENABLE_MODE, ST7789_BASIC_DEFAULT_VSPL,
      ST7789_BASIC_DEFAULT_HSPL, ST7789_BASIC_DEFAULT_DPL,
      ST7789_BASIC_DEFAULT_EPL, ST7789_BASIC_DEFAULT_VBP,
      ST7789_BASIC_DEFAULT_HBP);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set rgb interface control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default porch */
  res = st7789_set_porch(&gs_handle, ST7789_BASIC_DEFAULT_PORCH_NORMAL_BACK,
                         ST7789_BASIC_DEFAULT_PORCH_NORMAL_FRONT,
                         ST7789_BASIC_DEFAULT_PORCH_ENABLE,
                         ST7789_BASIC_DEFAULT_PORCH_IDEL_BACK,
                         ST7789_BASIC_DEFAULT_PORCH_IDEL_FRONT,
                         ST7789_BASIC_DEFAULT_PORCH_PART_BACK,
                         ST7789_BASIC_DEFAULT_PORCH_PART_FRONT);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set porch failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default frame rate control */
  res = st7789_set_frame_rate_control(
      &gs_handle, ST7789_BASIC_DEFAULT_SEPARATE_FR,
      ST7789_BASIC_DEFAULT_FRAME_RATE_DIVIDED,
      ST7789_BASIC_DEFAULT_INVERSION_IDLE_MODE,
      ST7789_BASIC_DEFAULT_IDLE_FRAME_RATE,
      ST7789_BASIC_DEFAULT_INVERSION_PARTIAL_MODE,
      ST7789_BASIC_DEFAULT_IDLE_PARTIAL_RATE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set frame rate control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default partial mode control */
  res = st7789_set_partial_mode_control(
      &gs_handle, ST7789_BASIC_DEFAULT_NON_DISPLAY_SOURCE_OUTPUT_LEVEL,
      ST7789_BASIC_DEFAULT_NON_DISPLAY_AREA_SCAN_MODE,
      ST7789_BASIC_DEFAULT_NON_DISPLAY_FRAME_FREQUENCY);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set partial mode control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default gate control */
  res = st7789_set_gate_control(&gs_handle, ST7789_BASIC_DEFAULT_VGHS,
                                ST7789_BASIC_DEFAULT_VGLS_NEGATIVE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set gate control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default gate on timing adjustment */
  res = st7789_set_gate_on_timing_adjustment(
      &gs_handle, ST7789_BASIC_DEFAULT_GATE_ON_TIMING,
      ST7789_BASIC_DEFAULT_GATE_OFF_TIMING_RGB,
      ST7789_BASIC_DEFAULT_GATE_OFF_TIMING);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set gate on timing adjustment failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default digital gamma */
  res =
      st7789_set_digital_gamma(&gs_handle, ST7789_BASIC_DEFAULT_DIGITAL_GAMMA);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set digital gamma failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* vcom convert to register  */
  res = st7789_vcom_convert_to_register(&gs_handle, ST7789_BASIC_DEFAULT_VCOMS,
                                        &reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: vcom convert to register failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default vcoms */
  res = st7789_set_vcoms(&gs_handle, reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set vcoms failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default lcm control */
  res = st7789_set_lcm_control(
      &gs_handle, ST7789_BASIC_DEFAULT_XMY, ST7789_BASIC_DEFAULT_XBGR,
      ST7789_BASIC_DEFAULT_XINV, ST7789_BASIC_DEFAULT_XMX,
      ST7789_BASIC_DEFAULT_XMH, ST7789_BASIC_DEFAULT_XMV,
      ST7789_BASIC_DEFAULT_XGS);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set lcm control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default vdv vrh cmd */
  res = st7789_set_vdv_vrh_from(&gs_handle, ST7789_BASIC_DEFAULT_VDV_VRH_FROM);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set vdv vrh from failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* vrhs convert to register */
  res = st7789_vrhs_convert_to_register(&gs_handle, ST7789_BASIC_DEFAULT_VRHS,
                                        &reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: vrhs convert to register failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default vrhs */
  res = st7789_set_vrhs(&gs_handle, reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set vrhs failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* vdv convert to register */
  res = st7789_vdv_convert_to_register(&gs_handle, ST7789_BASIC_DEFAULT_VDV,
                                       &reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: vdv convert to register failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default vdv */
  res = st7789_set_vdv(&gs_handle, reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set vdv failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* vcoms offset convert to register */
  res = st7789_vcoms_offset_convert_to_register(
      &gs_handle, ST7789_BASIC_DEFAULT_VCOMS_OFFSET, &reg);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: vcoms offset convert to register failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default vcoms offset */
  res = st7789_set_vcoms_offset(&gs_handle, reg);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set vcoms offset failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default frame rate */
  res = st7789_set_frame_rate(&gs_handle,
                              ST7789_BASIC_DEFAULT_INVERSION_SELECTION,
                              ST7789_BASIC_DEFAULT_FRAME_RATE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set frame rate failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default cabc control */
  res = st7789_set_cabc_control(&gs_handle, ST7789_BASIC_DEFAULT_LED_ON,
                                ST7789_BASIC_DEFAULT_LED_PWM_INIT,
                                ST7789_BASIC_DEFAULT_LED_PWM_FIX,
                                ST7789_BASIC_DEFAULT_LED_PWM_POLARITY);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set cabc control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default pwm frequency */
  res =
      st7789_set_pwm_frequency(&gs_handle, ST7789_BASIC_DEFAULT_PWM_FREQUENCY);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set pwm frequency failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default power control 1 */
  res = st7789_set_power_control_1(&gs_handle, ST7789_BASIC_DEFAULT_AVDD,
                                   ST7789_BASIC_DEFAULT_AVCL_NEGTIVE,
                                   ST7789_BASIC_DEFAULT_VDS);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set power control 1 failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default command 2 */
  res = st7789_set_command_2_enable(&gs_handle,
                                    ST7789_BASIC_DEFAULT_COMMAND_2_ENABLE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set command 2 enable failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default positive voltage gamma control */
  res = st7789_set_positive_voltage_gamma_control(&gs_handle, param_positive);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set positive voltage gamma control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default negative voltage gamma control */
  res = st7789_set_negative_voltage_gamma_control(&gs_handle, param_negative);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set negative voltage gamma control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* create the table */
  for (i = 0; i < 64; i++) {
    params[i] = i * 4;
  }

  /* set default digital gamma look up table red */
  res = st7789_set_digital_gamma_look_up_table_red(&gs_handle, params);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set digital gamma look up table red ailed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default digital gamma look up table blue */
  res = st7789_set_digital_gamma_look_up_table_blue(&gs_handle, params);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: set digital gamma look up table blue ailed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default gate line convert to register */
  res = st7789_gate_line_convert_to_register(
      &gs_handle, ST7789_BASIC_DEFAULT_GATE_LINE, &reg);
  if (res != 0) {
    st7789_interface_debug_print(
        "st7789: gate line convert to register failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default gate */
  res = st7789_set_gate(&gs_handle, reg, ST7789_BASIC_DEFAULT_FIRST_SCAN_LINE,
                        ST7789_BASIC_DEFAULT_GATE_SCAN_MODE,
                        ST7789_BASIC_DEFAULT_GATE_SCAN_DIRECTION);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set gate failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default spi2 */
  res = st7789_set_spi2_enable(&gs_handle, ST7789_BASIC_DEFAULT_SPI2_LANE,
                               ST7789_BASIC_DEFAULT_COMMAND_TABLE_2);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set spi2 enable failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default power control 2 */
  res = st7789_set_power_control_2(&gs_handle, ST7789_BASIC_DEFAULT_SBCLK_DIV,
                                   ST7789_BASIC_DEFAULT_STP14CK_DIV);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set power control 2 failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default equalize time control */
  res = st7789_set_equalize_time_control(
      &gs_handle, ST7789_BASIC_DEFAULT_SOURCE_EQUALIZE_TIME,
      ST7789_BASIC_DEFAULT_SOURCE_PRE_DRIVE_TIME,
      ST7789_BASIC_DEFAULT_GATE_EQUALIZE_TIME);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set equalize time control failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* set default program mode */
  res = st7789_set_program_mode_enable(&gs_handle,
                                       ST7789_BASIC_DEFAULT_PROGRAM_MODE);
  if (res != 0) {
    st7789_interface_debug_print("st7789: set program mode enable ailed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* display on */
  res = st7789_display_on(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: display on failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  /* clear */
  res = st7789_clear(&gs_handle);
  if (res != 0) {
    st7789_interface_debug_print("st7789: clear failed.\n");
    (void)st7789_deinit(&gs_handle);

    return 1;
  }

  return 0;
}

uint8_t display_driver_deinit(void) {
  /* st7789 deinit */
  if (st7789_deinit(&gs_handle) != 0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_write_point(uint16_t x, uint16_t y, uint32_t color) {
  /* draw point */
  if (st7789_draw_point(&gs_handle, x, y, color) != 0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_draw_picture_16bits(uint16_t left, uint16_t top,
                                           uint16_t right, uint16_t bottom,
                                           uint16_t *img) {
  /* draw picture in 16 bits */
  if (st7789_draw_picture_16bits(&gs_handle, left, top, right, bottom, img) !=
      0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_clear(void) {
  /* st7789 clear */
  if (st7789_clear(&gs_handle) != 0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_string(uint16_t x, uint16_t y, char *str, uint16_t len,
                              uint32_t color, st7789_font_t font) {
  /* write string */
  if (st7789_write_string(&gs_handle, x, y, str, len, color, font) != 0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_rect(uint16_t left, uint16_t top, uint16_t right,
                            uint16_t bottom, uint32_t color) {
  /* fill rect */
  if (st7789_fill_rect(&gs_handle, left, top, right, bottom, color) != 0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_on(void) {
  /* display on */
  if (st7789_display_on(&gs_handle) != 0) {
    return 1;
  }

  return 0;
}

uint8_t display_driver_off(void) {
  /* display off */
  if (st7789_display_off(&gs_handle) != 0) {
    return 1;
  }

  return 0;
}