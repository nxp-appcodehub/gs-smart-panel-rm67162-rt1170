/*
 * Copyright 2019, 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "vglite_support.h"
#include "vglite_window.h"

#include "tiger_paths.h"

/*-----------------------------------------------------------*/
#include "vg_lite.h"

#include "fsl_soc_src.h"
#include "mcmgr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void vglite_task(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static vg_lite_display_t display;
static vg_lite_window_t window;

static int zoomOut    = 0;
static int scaleCount = 0;
static vg_lite_matrix_t matrix;

#if (CUSTOM_VGLITE_MEMORY_CONFIG != 1)
#error "Application must be compiled with CUSTOM_VGLITE_MEMORY_CONFIG=1"
#else
#define VGLITE_COMMAND_BUFFER_SZ (128 * 1024)
/* On RT595S */
#if defined(CPU_MIMXRT595SFFOC_cm33)
#define VGLITE_HEAP_SZ 0x400000 /* 4 MB */
/* On RT1170 */
#elif defined(CPU_MIMXRT1176DVMAA_cm7) || defined(CPU_MIMXRT1166DVM6A_cm7)
#define VGLITE_HEAP_SZ 8912896 /* 8.5 MB */
#else
#error "Unsupported CPU !"
#endif

#if (720 * 1280 == (DEMO_PANEL_WIDTH) * (DEMO_PANEL_HEIGHT))
#define TW 720
/* On RT595S */
#if defined(CPU_MIMXRT595SFFOC_cm33)
/* Tessellation window = 720 x 640 */
#define TH 640
/* On RT1170 */
#elif defined(CPU_MIMXRT1176DVMAA_cm7) || defined(CPU_MIMXRT1166DVM6A_cm7)
/* Tessellation window = 720 x 1280 */
#define TH 1280
#else
#error "Unsupported CPU !"
#endif
/* Panel RM67162. Supported only by platform RT595S. */
#elif (400 * 400 == (DEMO_PANEL_WIDTH) * (DEMO_PANEL_HEIGHT))
/* Tessellation window = 400 x 400 */
#define TW 400
#define TH 400
#else
/* Tessellation window = 400 x 400 */
#define TW 400
#define TH 400
#endif
/* Allocate the heap and set the command buffer(s) size */
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t vglite_heap[VGLITE_HEAP_SZ], 64);

void *vglite_heap_base        = &vglite_heap;
uint32_t vglite_heap_size     = VGLITE_HEAP_SZ;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    (void)MCMGR_EarlyInit();
}


#if (DEMO_PANEL_RM67162 == DEMO_PANEL)
void CM7_GPIO2_3_IRQHandler(void)
{
    uint32_t intStat;

    intStat = GPIO_PortGetInterruptFlags(BOARD_MIPI_PANEL_TE_GPIO);

    GPIO_PortClearInterruptFlags(BOARD_MIPI_PANEL_TE_GPIO, intStat);

    if (intStat & (1U << BOARD_MIPI_PANEL_TE_PIN))
    {
        BOARD_DisplayTEPinHandler();
    }
}
#endif


static void BOARD_ResetDisplayMix(void)
{
    /*
     * Reset the displaymix, otherwise during debugging, the
     * debugger may not reset the display, then the behavior
     * is not right.
     */
    SRC_AssertSliceSoftwareReset(SRC, kSRC_DisplaySlice);
    while (kSRC_SliceResetInProcess == SRC_GetSliceResetState(SRC, kSRC_DisplaySlice))
    {
    }
}


int main(void)
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_BootClockRUN();
    BOARD_ResetDisplayMix();
    BOARD_InitLpuartPins();
    BOARD_InitMipiPanelPins();
    BOARD_InitDebugConsole();

    if (xTaskCreate(vglite_task, "vglite_task", configMINIMAL_STACK_SIZE + 200, NULL, configMAX_PRIORITIES - 2, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    vTaskStartScheduler();

    for (;;)
        ;
}

static void cleanup(void)
{
    uint8_t i;
    for (i = 0; i < pathCount; i++)
    {
        vg_lite_clear_path(&path[i]);
    }

    vg_lite_close();
}

static vg_lite_error_t init_vg_lite(void)
{
    vg_lite_error_t error = VG_LITE_SUCCESS;
    int fb_width, fb_height;

    error = VGLITE_CreateDisplay(&display);
    if (error)
    {
        PRINTF("VGLITE_CreateDisplay failed: VGLITE_CreateDisplay() returned error %d\r\n", error);
        return error;
    }
    // Initialize the window.
    error = VGLITE_CreateWindow(&display, &window);
    if (error)
    {
        PRINTF("VGLITE_CreateWindow failed: VGLITE_CreateWindow() returned error %d\r\n", error);
        return error;
    }
    // Initialize the draw.
    error = vg_lite_init(TW, TH);
    if (error)
    {
        PRINTF("vg_lite engine init failed: vg_lite_init() returned error %d\r\n", error);
        cleanup();
        return error;
    }
    // Set GPU command buffer size for this drawing task.
    error = vg_lite_set_command_buffer_size(VGLITE_COMMAND_BUFFER_SZ);
    if (error)
    {
        PRINTF("vg_lite_set_command_buffer_size() returned error %d\r\n", error);
        cleanup();
        return error;
    }

//    vg_lite_set_scissor(200, 200, 300, 300);
//    vg_lite_enable_scissor();
    // Setup a scale at center of buffer.
    fb_width  = window.width;
    fb_height = window.height;
    vg_lite_identity(&matrix);
    vg_lite_translate(fb_width / 2 - 20 * fb_width / 640.0f, fb_height / 2 - 50 * fb_height / 480.0f, &matrix);
    vg_lite_scale(2.5f, 2.5f, &matrix);
    vg_lite_scale(fb_width / 640.0f, fb_height / 480.0f, &matrix);

    return error;
}

void animateTiger()
{
    if (zoomOut)
    {
        vg_lite_scale(1.25, 1.25, &matrix);
        if (0 == --scaleCount)
            zoomOut = 0;
    }
    else
    {
        vg_lite_scale(0.8, 0.8, &matrix);
        if (5 == ++scaleCount)
            zoomOut = 1;
    }

    vg_lite_rotate(5, &matrix);
}

static void redraw()
{
    vg_lite_error_t error = VG_LITE_SUCCESS;
    uint8_t count;
    vg_lite_buffer_t *rt = VGLITE_GetRenderTarget(&window);

    if (rt == NULL)
    {
        PRINTF("vg_lite_get_renderTarget error\r\n");
        while (1)
            ;
    }

    // Draw the path using the matrix.
    vg_lite_clear(rt, NULL, 0xFFFF0000);

    for (count = 0; count < pathCount; count++)
    {
        error = vg_lite_draw(rt, &path[count], VG_LITE_FILL_EVEN_ODD, &matrix, VG_LITE_BLEND_NONE, color_data[count]);
        if (error)
        {
            PRINTF("vg_lite_draw() returned error %d\r\n", error);
            cleanup();
            return;
        }
    }

    animateTiger();

    VGLITE_SwapBuffers(&window);

    return;
}

uint32_t getTime()
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void vglite_task(void *pvParameters)
{
    status_t status;
    vg_lite_error_t error;
    uint32_t startTime, time, n = 0, fps_x_1000;

    status = BOARD_PrepareVGLiteController();
    if (status != kStatus_Success)
    {
        PRINTF("Prepare VGlite contolor error\r\n");
        while (1)
            ;
    }

    error = init_vg_lite();
    if (error)
    {
        PRINTF("init_vg_lite failed: init_vg_lite() returned error %d\r\n", error);
        while (1)
            ;
    }

    startTime = getTime();

    while (1)
    {
        redraw();
        n++;
        if (n >= 60)
        {
            time       = getTime() - startTime;
            fps_x_1000 = (n * 1000 * 1000) / time;
            PRINTF("%d frames in %d mSec: %d.%d FPS\r\n", n, time, fps_x_1000 / 1000, fps_x_1000 % 1000);
            n         = 0;
            startTime = getTime();
        }
    }
}


