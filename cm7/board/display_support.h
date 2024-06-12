/*
 * Copyright 2019-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DISPLAY_SUPPORT_H_
#define _DISPLAY_SUPPORT_H_

#include "fsl_dc_fb.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */

#define DEMO_PANEL_RK055AHD091 0 /* 720 * 1280 */
#define DEMO_PANEL_RK055IQH091 1 /* 540 * 960  */
#define DEMO_PANEL_RK055MHD091 2 /* 720 * 1280 */
#define DEMO_PANEL_RM67162     3

#ifndef DEMO_PANEL
#define DEMO_PANEL DEMO_PANEL_RM67162
#endif

#if (DEMO_PANEL == DEMO_PANEL_RM67162)
/*
 * Use the MIPI smart panel
 *
 * MIPI_DSI support three pixel formats:
 *
 * 1. RGB565: frame buffer format is RGB565, MIPI DSI send out data format is RGB565,
 *    to use this, set DEMO_RM67162_USE_RGB565=1
 *
 * 2. XRGB8888: frame buffer format is XRGB888, Code helps drop the useless byte
 *    and MIPI DSI send out data format is RGB888. Using GPU needs this format.
 *    to use this, set DEMO_RM67162_USE_RGB565=0, DEMO_RM67162_USE_XRGB8888=1
 */

#ifndef DEMO_RM67162_USE_RGB565
#define DEMO_RM67162_USE_RGB565 1
#endif

#ifndef DEMO_RM67162_USE_XRGB8888
#define DEMO_RM67162_USE_XRGB8888 0
#endif

/* Pixel format macro mapping. */
#define DEMO_RM67162_BUFFER_RGB565   0
#define DEMO_RM67162_BUFFER_RGB888   1
#define DEMO_RM67162_BUFFER_XRGB8888 2

#if DEMO_RM67162_USE_RGB565
#define DEMO_RM67162_BUFFER_FORMAT DEMO_RM67162_BUFFER_RGB565
#else
#if DEMO_RM67162_USE_XRGB8888
#define DEMO_RM67162_BUFFER_FORMAT DEMO_RM67162_BUFFER_XRGB8888
#else
#define DEMO_RM67162_BUFFER_FORMAT DEMO_RM67162_BUFFER_RGB888
#endif

#endif

/* Use auxiliary core to send image to panel */
#ifndef DEMO_RM67162_USE_DSI_AUX
#define DEMO_RM67162_USE_DSI_AUX 1
#endif

#define DEMO_BUFFER_FIXED_ADDRESS 0

#if DEMO_BUFFER_FIXED_ADDRESS
#define DEMO_BUFFER0_ADDR 0x80000000
#define DEMO_BUFFER1_ADDR 0x80200000
#endif

/* Definitions for the frame buffer. */
/* 1 is enough, use 2 could render background buffer while display the foreground buffer. */
#define DEMO_BUFFER_COUNT         2

#if (DEMO_RM67162_BUFFER_FORMAT == DEMO_RM67162_BUFFER_RGB565)

#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatRGB565
#define DEMO_BUFFER_BYTE_PER_PIXEL 2
   
#define BOARD_MIPI_CLK_HZ       350000000U /*320MHz*/

#elif (DEMO_RM67162_BUFFER_FORMAT == DEMO_RM67162_BUFFER_RGB888)

#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatRGB888
#define DEMO_BUFFER_BYTE_PER_PIXEL 3
#define BOARD_MIPI_CLK_HZ       500000000U /*500MHz*/
#else

#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatXRGB8888
#define DEMO_BUFFER_BYTE_PER_PIXEL 4
#define BOARD_MIPI_CLK_HZ       500000000U /*500MHz*/
#endif /* DEMO_RM67162_BUFFER_FORMAT */

#define DEMO_PANEL_WIDTH  (400U)
#define DEMO_PANEL_HEIGHT (392U)

#define DEMO_BUFFER_WIDTH   (400U)
#define DEMO_BUFFER_HEIGHT  (400U)

/* Where the frame buffer is shown in the screen. */
#define DEMO_BUFFER_START_X 4U
#define DEMO_BUFFER_START_Y 4U

#else
/*
 * Use the MIPI dumb panel
 */

#define DEMO_DISPLAY_CONTROLLER_ELCDIF  0
#define DEMO_DISPLAY_CONTROLLER_LCDIFV2 1

#ifndef DEMO_DISPLAY_CONTROLLER
/* Use LCDIFV2 by default, could use ELCDIF by changing this macro. */
#define DEMO_DISPLAY_CONTROLLER DEMO_DISPLAY_CONTROLLER_LCDIFV2
#endif

#define DEMO_BUFFER_FIXED_ADDRESS 0

#if DEMO_BUFFER_FIXED_ADDRESS
#define DEMO_BUFFER0_ADDR 0x80000000
#define DEMO_BUFFER1_ADDR 0x80200000
#endif


/* Definitions for the frame buffer. */
#define DEMO_BUFFER_COUNT 2 /* 2 is enough for DPI interface display. */

#ifndef DEMO_USE_XRGB8888
#define DEMO_USE_XRGB8888 0
#endif

#if DEMO_USE_XRGB8888
#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatXRGB8888
#define DEMO_BUFFER_BYTE_PER_PIXEL 4
#else
#define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatRGB565
#define DEMO_BUFFER_BYTE_PER_PIXEL 2
#endif

#if ((DEMO_PANEL_RK055AHD091 == DEMO_PANEL) || (DEMO_PANEL_RK055MHD091 == DEMO_PANEL))

#define DEMO_PANEL_WIDTH  (720)
#define DEMO_PANEL_HEIGHT (1280)

#elif (DEMO_PANEL_RK055IQH091 == DEMO_PANEL)

#define DEMO_PANEL_WIDTH  (540)
#define DEMO_PANEL_HEIGHT (960)

#endif

#define DEMO_BUFFER_WIDTH  DEMO_PANEL_WIDTH
#define DEMO_BUFFER_HEIGHT DEMO_PANEL_HEIGHT

/* Where the frame buffer is shown in the screen. */
#define DEMO_BUFFER_START_X 0U
#define DEMO_BUFFER_START_Y 0U

#endif /* (DEMO_PANEL == DEMO_PANEL_RM67162) */

#define DEMO_BUFFER_STRIDE_BYTE (DEMO_BUFFER_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL)
/* There is not frame buffer aligned requirement, consider the 64-bit AXI data
 * bus width and 32-byte cache line size, the frame buffer alignment is set to
 * 32 byte.
 */
#define FRAME_BUFFER_ALIGN 32

extern const dc_fb_t g_dc;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

status_t BOARD_PrepareDisplayController(void);
void BOARD_DisplayTEPinHandler(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _DISPLAY_SUPPORT_H_ */
