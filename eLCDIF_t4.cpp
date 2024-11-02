#include "eLCDIF_t4.h"


/*

#### Public Functions ####

*/

FLASHMEM void eLCDIF_t4::begin(eLCDIF_t4_config config, BUS_WIDTH busWidth, WORD_LENGTH wordLength, COLOR_DEPTH colorDepth) {
  internal_config = config;
  initLCDPins(colorDepth);
  setVideoClock(4*config.clk_num, config.clk_den);
  initLCDIF(config, busWidth, wordLength, colorDepth);
};

FASTRUN void eLCDIF_t4::setCurrentBufferAddress(void*buffer) {
  LCDIF_CUR_BUF = (uint32_t)buffer;
};

FASTRUN void eLCDIF_t4::setNextBufferAddress(void*buffer) {
  LCDIF_NEXT_BUF = (uint32_t)buffer;
};

FLASHMEM void eLCDIF_t4::onCompleteCallback(CBF callback) {
  _callback = callback;
};

FLASHMEM void eLCDIF_t4::runLCD() {
  attachInterruptVector(IRQ_LCDIF, LCDIF_ISR);
  NVIC_SET_PRIORITY(IRQ_LCDIF, 128);
  NVIC_ENABLE_IRQ(IRQ_LCDIF);

  Serial.println("Unmasking frame interrupt");
  // unmask CUR_FRAME_DONE interrupt
  LCDIF_CTRL1_SET = LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN;
  // VSYNC_EDGE interrupt also available to notify beginning of raster
  //LCDIF_CTRL1_SET = LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN;
  Serial.println("Running LCD");
  // start LCD
  LCDIF_CTRL_SET = LCDIF_CTRL_RUN | LCDIF_CTRL_DOTCLK_MODE;
};




/*

#### Private Functions ####

*/

FLASHMEM void eLCDIF_t4::setVideoClock(int num, int den){
  int post_divide = 0;
  while (num < 27*den) num <<= 1, ++post_divide;
  int div_select = num / den;
  num -= div_select * den;

  // div_select valid range: 27-54
  float freq = ((float)num / den + div_select) * 24.0f / (1 << post_divide);
  Serial.print("VID_PLL: ");
  Serial.print(freq);
  Serial.print("Mhz, div_select: ");
  Serial.println(div_select);

  // switch video PLL to bypass, enable, set div_select
  CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_ENABLE | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(div_select);
  // clear misc2 vid post-divider
  CCM_ANALOG_MISC2_CLR = CCM_ANALOG_MISC2_VIDEO_DIV(3);
  switch (post_divide) {
      case 0: // div by 1
        CCM_ANALOG_PLL_VIDEO_SET = CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(2);
        break;
      case 1: // div by 2
        CCM_ANALOG_PLL_VIDEO_SET = CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(1);
        break;
      // div by 4
      // case 2: PLL_VIDEO pos_div_select already set to 0
      case 3: // div by 8 (4*2)
        CCM_ANALOG_MISC2_SET = CCM_ANALOG_MISC2_VIDEO_DIV(1);
        break;
      case 4: // div by 16 (4*4)
        CCM_ANALOG_MISC2_SET = CCM_ANALOG_MISC2_VIDEO_DIV(3);
        break;
  }
  CCM_ANALOG_PLL_VIDEO_NUM = num;
  CCM_ANALOG_PLL_VIDEO_DENOM = den;
  // ensure PLL is powered
  CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_VIDEO_POWERDOWN;
  // wait for lock
  Serial.print("Waiting for PLL Lock...");
  while (!(CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK));
  // deactivate bypass
  CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_VIDEO_BYPASS;
  Serial.println("done.");


  Serial.print("Configuring LCD pix_clk source...");
  // gate clocks from lcd
  CCM_CCGR2 &= ~CCM_CCGR2_LCD(CCM_CCGR_ON);
  CCM_CCGR3 &= ~CCM_CCGR3_LCDIF_PIX(CCM_CCGR_ON);
  // set LCDIF source to PLL5, pre-divide by 4
  uint32_t r = CCM_CSCDR2;
  r &= ~(CCM_CSCDR2_LCDIF_PRE_CLK_SEL(7) | CCM_CSCDR2_LCDIF_PRED(7));
  r |= CCM_CSCDR2_LCDIF_PRE_CLK_SEL(2) | CCM_CSCDR2_LCDIF_PRED(3);
  CCM_CSCDR2 = r;
  // set LCDIF post-divide to 1
  CCM_CBCMR &= ~CCM_CBCMR_LCDIF_PODF(7);
  CCM_CCGR2 |= CCM_CCGR2_LCD(CCM_CCGR_ON);
  CCM_CCGR3 |= CCM_CCGR3_LCDIF_PIX(CCM_CCGR_ON);
  Serial.println("done.");

};

FLASHMEM void eLCDIF_t4::initLCDPins(COLOR_DEPTH colorDepth) {
  // Always initialize control pins (DCLK, DE, H_SYNC, V_SYNC)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0; // DCLK
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0; // DE
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0; // H_SYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0; // V_SYNC

  // Set drive strength for control pins
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = 0xFF; // DCLK
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = 0xFF; // DE
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = 0xFF; // H_SYNC
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = 0xFF; // V_SYNC

  // Initialize data pins based on color depth
  // Common data pins for all configurations (RAW8 and higher)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_04 = 0;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_05 = 0;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_06 = 0;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_07 = 0;

  // Additional pins for 16-bit (RGB565)
  if (colorDepth == COLOR_DEPTH_RGB565) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_08 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_12 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_13 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15 = 0;
  }

  // Additional pins for 18-bit (RGB666)
  if (colorDepth == COLOR_DEPTH_RGB666) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_08 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_12 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_13 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04 = 0;
  }

  // Additional pins for 24-bit (RGB888 and XRGB8888)
  if (colorDepth == COLOR_DEPTH_RGB888 || colorDepth == COLOR_DEPTH_XRGB8888) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_08 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_12 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_13 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_05 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_06 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_07 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_08 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_09 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10 = 0;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_11 = 0;
  }

  // Set drive strength for data pins
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_04 = 0xFF;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_05 = 0xFF;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_06 = 0xFF;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_07 = 0xFF;

  if (colorDepth >= COLOR_DEPTH_RGB565) {
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_08 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_09 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_12 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_13 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_14 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_15 = 0xFF;
  }

  if (colorDepth >= COLOR_DEPTH_RGB666) {
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_00 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_01 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_02 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_03 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = 0xFF;
  }

  if (colorDepth >= COLOR_DEPTH_RGB888) {
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_07 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_08 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_09 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_10 = 0xFF;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = 0xFF;
  }
}

FLASHMEM void eLCDIF_t4::initLCDIF(eLCDIF_t4_config config, BUS_WIDTH busWidth, WORD_LENGTH wordLength, COLOR_DEPTH colorDepth) {
  Serial.print("Resetting LCDIF...");
  // reset LCDIF
  LCDIF_CTRL_CLR = LCDIF_CTRL_CLKGATE;
  while (LCDIF_CTRL & LCDIF_CTRL_CLKGATE);
  Serial.print("poking reset...");
  LCDIF_CTRL_SET = LCDIF_CTRL_SFTRST;
  while (!(LCDIF_CTRL & LCDIF_CTRL_SFTRST));
  
  Serial.print("re-enabling clock...");
  LCDIF_CTRL_CLR = LCDIF_CTRL_SFTRST | LCDIF_CTRL_CLKGATE;
  Serial.println("done.");

  Serial.println("Initializing LCDIF registers...");
  
  // Set up the control and control1 values based on color depth and word length
  uint32_t ctrlWordLength = LCDIF_CTRL_WORD_LENGTH(wordLength);
  uint32_t ctrl1BytePackingFormat = 0x0F;  // Default value for byte packing

  switch (colorDepth) {
    case COLOR_DEPTH_RAW8:
      ctrl1BytePackingFormat = LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0F);
      break;
    case COLOR_DEPTH_RGB565:
      ctrl1BytePackingFormat = LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0F);
      break;
    case COLOR_DEPTH_RGB666:
      ctrl1BytePackingFormat = LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x07);
      break;
    case COLOR_DEPTH_XRGB8888:
      ctrl1BytePackingFormat = LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x07);
      break;
    case COLOR_DEPTH_RGB888:
      ctrl1BytePackingFormat = LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0F);
      break;
    default:
      Serial.println("Unsupported color depth.");
      return;
  }

  LCDIF_CTRL = ctrlWordLength | LCDIF_CTRL_LCD_DATABUS_WIDTH(busWidth) | LCDIF_CTRL_DOTCLK_MODE | LCDIF_CTRL_BYPASS_COUNT | LCDIF_CTRL_MASTER;
  LCDIF_CTRL1 = LCDIF_CTRL1_RECOVER_ON_UNDERFLOW | ctrl1BytePackingFormat;

  LCDIF_CTRL2 = LCDIF_CTRL2_OUTSTANDING_REQ(4) | LCDIF_CTRL2_BURST_LEN_8(0);
  LCDIF_TRANSFER_COUNT = LCDIF_TRANSFER_COUNT_V_COUNT(config.height) | LCDIF_TRANSFER_COUNT_H_COUNT(config.width);
  
  LCDIF_VDCTRL0 = LCDIF_VDCTRL0_ENABLE_PRESENT | LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT | LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT | LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH(config.vsw) | config.vpolarity | config.hpolarity;
  LCDIF_VDCTRL1 = config.height + config.vfp + config.vsw + config.vbp;
  LCDIF_VDCTRL2 = LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH(config.hsw) | LCDIF_VDCTRL2_HSYNC_PERIOD(config.width + config.hfp + config.hsw + config.hbp);
  LCDIF_VDCTRL3 = LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT(config.hsw + config.hbp) | LCDIF_VDCTRL3_VERTICAL_WAIT_CNT(config.vsw + config.vbp);
  LCDIF_VDCTRL4 = LCDIF_VDCTRL4_SYNC_SIGNALS_ON | LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT(config.width);
  
  Serial.printf("LCDIF_CTRL: 0x%x\nLCDIF_CTRL1: 0x%x\nLCDIF_TRANSFER_COUNT: 0x%x\nLCDIF_VDCTRL0: 0x%x\nLCDIF_VDCTRL1: 0x%x\nLCDIF_VDCTRL2: 0x%x\nLCDIF_VDCTRL3: 0x%x\nLCDIF_VDCTRL4: 0x%x\n", 
                LCDIF_CTRL, LCDIF_CTRL1, LCDIF_TRANSFER_COUNT, LCDIF_VDCTRL0, LCDIF_VDCTRL1, LCDIF_VDCTRL2, LCDIF_VDCTRL3, LCDIF_VDCTRL4);
  Serial.println("done.");
};

eLCDIF_t4::CBF eLCDIF_t4::_callback = nullptr;

FASTRUN void eLCDIF_t4::LCDIF_ISR(void) {
  uint32_t intStatus = LCDIF_CTRL1 & (LCDIF_CTRL1_BM_ERROR_IRQ | LCDIF_CTRL1_OVERFLOW_IRQ | LCDIF_CTRL1_UNDERFLOW_IRQ | LCDIF_CTRL1_CUR_FRAME_DONE_IRQ | LCDIF_CTRL1_VSYNC_EDGE_IRQ);
  // clear all pending LCD interrupts
  LCDIF_CTRL1_CLR = intStatus;
  if (intStatus & (LCDIF_CTRL1_CUR_FRAME_DONE_IRQ | LCDIF_CTRL1_VSYNC_EDGE_IRQ)) {
    if (_callback){
    _callback();
    }
  }
  asm volatile("dsb");
};