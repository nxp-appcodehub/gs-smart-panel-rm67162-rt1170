/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "mcmgr.h"

#include "mipi_dsi_aux.h"
#include "fsl_mipi_dsi_cmd.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*
 * Smart panel
 */

/* Definitions for MIPI. */
#define DEMO_MIPI_DSI_IRQn     MIPI_DSI_IRQn
#define DEMO_MIPI_DSI          (&g_mipiDsi)
#define DEMO_MIPI_DSI_LANE_NUM 1

/* The max APB transfer chunck size. */
#define FSL_DSI_TX_MAX_CHUNCK_BYTE (FSL_DSI_TX_MAX_PAYLOAD_BYTE)

typedef struct _dsi_mem_write_ctx
{
    volatile bool onGoing;
    const uint8_t *txData;
    uint32_t leftByteLen;
    uint32_t txDataSizeMax;
    uint16_t txFifoSize;
    uint8_t dscCmd;
} dsi_mem_write_ctx_t;

/*! @brief Structure for the data transfer. */
typedef struct _dsi_transfer_ext
{
    dsi_aux_input_pixel_format_t inputFormat;   /*!< Input format. */
    dsi_aux_output_pixel_format_t outputFormat; /*!< Output format. */
    uint8_t virtualChannel;        /*!< Virtual channel. */
    dsi_tx_data_type_t txDataType; /*!< TX data type. */
    uint8_t flags;                 /*!< Flags to control the transfer, see _dsi_transfer_flags. */
    const uint8_t *txData;         /*!< The TX data buffer. */
    uint8_t *rxData;               /*!< The TX data buffer. */
    uint16_t txDataSize;           /*!< Size of the TX data. */
    uint16_t rxDataSize;           /*!< Size of the RX data. */
    bool sendDscCmd;               /*!< If set to true, the DSC command is specified by @ref dscCmd, otherwise
                                        the DSC command is included in the @ref txData. */
    uint8_t dscCmd;                /*!< The DSC command to send, only valid when @ref sendDscCmd is true. */
    uint32_t txDataSizeMax;
    uint16_t txFifoSize;
} dsi_transfer_ext_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static dsi_mem_write_ctx_t s_dsiMemWriteCtx;
static dsi_transfer_ext_t s_dsiMemWriteXfer = {0};
static dsi_aux_write_mem_transfer_t *g_xfer;

static const MIPI_DSI_Type g_mipiDsi = {
    .host = DSI_HOST,
    .apb  = DSI_HOST_APB_PKT_IF,
    .dpi  = DSI_HOST_DPI_INTFC,
    .dphy = DSI_HOST_DPHY_INTFC,
};

volatile uint32_t armCM4_Hz;
volatile float time;
/*******************************************************************************
 * Code
 ******************************************************************************/
static uint16_t DSI_WriteDataStep1_RGB565(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint32_t firstWord;
    uint16_t i, payloadSize;
    const uint8_t *payload;

    DSI_HOST_APB_PKT_IF_Type *apb = base->apb;

    if (xfer->txDataSize > xfer->txFifoSize)
    {
        payloadSize = xfer->txFifoSize + 1;
    }
    else
    {
        payloadSize = xfer->txDataSize + 1;
    }

    payload = xfer->txData;
    firstWord = xfer->dscCmd;

    assert(payloadSize >= 3);

    if (payloadSize == 3)
    {
        firstWord |= ((uint32_t)payload[0] << 16U) | ((uint32_t)payload[1] << 8U);

        apb->TX_PAYLOAD = firstWord;
        xfer->txData += 2;
        xfer->txDataSize -= 2;
        return payloadSize;
    }
    else
    {
        firstWord |= ((uint32_t)payload[3] << 24U) | ((uint32_t)payload[0] << 16U) | ((uint32_t)payload[1] << 8U);

        apb->TX_PAYLOAD = firstWord;
        payload += 4;
        payloadSize -= 4;
    }

    for (i = 0; i < payloadSize >> 2; i++)
    {
        apb->TX_PAYLOAD = ((uint32_t)payload[3] << 24U) | ((uint32_t)payload[0] << 16U) | ((uint32_t)payload[1] << 8U) | payload[-2];

        payload += 4;
    }

    /* Only write the remaining data while the txDataSize <= max available FIFO size */
    if (xfer->txDataSize <= xfer->txFifoSize)
    {
        switch (payloadSize & 0x03U)
        {
            case 3:
                apb->TX_PAYLOAD = ((uint32_t)payload[0] << 16U) | ((uint32_t)payload[1] << 8U) | payload[-2];
                break;
            case 1:
                apb->TX_PAYLOAD = payload[-2];
                break;
            default:
                /* For MISRA 2012 16.4 */
                break;
        }
    }
    else
    {
        payloadSize &= ~0x03U;
    }

    xfer->txData += (payloadSize + 4);
    xfer->txDataSize -= (payloadSize + 3);

    return payloadSize + 4;
}

static uint16_t DSI_WriteDataStep1_RGB888(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint32_t firstWord;
    uint16_t i, payloadSize;
    const uint8_t *payload;

    DSI_HOST_APB_PKT_IF_Type *apb = base->apb;

    if (xfer->txDataSize > xfer->txFifoSize)
    {
        payloadSize = xfer->txFifoSize + 1;
    }
    else
    {
        payloadSize = xfer->txDataSize + 1;
    }

    payload = xfer->txData;
    firstWord = xfer->dscCmd;

    assert(payloadSize >= 4);

    if (payloadSize == 4)
    {
        firstWord |= ((uint32_t)payload[0] << 24U) | ((uint32_t)payload[1] << 16U) | ((uint32_t)payload[2] << 8U);

        apb->TX_PAYLOAD = firstWord;
        xfer->txData += 3;
        xfer->txDataSize -= 3;
        return payloadSize;
    }
    else
    {
        firstWord |= ((uint32_t)payload[0] << 24U) | ((uint32_t)payload[1] << 16U) | ((uint32_t)payload[2] << 8U);

        apb->TX_PAYLOAD = (uint32_t)firstWord;
        payload += 3;
        payloadSize -= 4;
    }

    for (i = 0; i < payloadSize >> 2; i++)
    {
        payload += 12;

        apb->TX_PAYLOAD = ((uint32_t)payload[-7] << 24U) | ((uint32_t)payload[-12] << 16U) | ((uint32_t)payload[-11] << 8U) | payload[-10];

        apb->TX_PAYLOAD = ((uint32_t)payload[-5] << 24U) | ((uint32_t)payload[-4]  << 16U) | ((uint32_t)payload[-9]  << 8U) | payload[-8];

        apb->TX_PAYLOAD = ((uint32_t)payload[-3] << 24U) | ((uint32_t)payload[-2]  << 16U) | ((uint32_t)payload[-1]  << 8U) | payload[-6];
    }

    /* Only write the remaining data while the txDataSize <= max available FIFO size */
    if (xfer->txDataSize <= xfer->txFifoSize)
    {
        switch (payloadSize & 0x0FU)
        {
            case 10:
                payload += 9;
                apb->TX_PAYLOAD = ((uint32_t)payload[-4] << 24U) | ((uint32_t)payload[-9] << 16U) | ((uint32_t)payload[-8] << 8U) | payload[-7];
                apb->TX_PAYLOAD = ((uint32_t)payload[-2] << 24U) | ((uint32_t)payload[-1]  << 16U) | ((uint32_t)payload[-6]  << 8U) | payload[-5];
                apb->TX_PAYLOAD = payload[-3];
                break;
            case 7:
                payload += 6;
                apb->TX_PAYLOAD = ((uint32_t)payload[-1] << 24U) | ((uint32_t)payload[-6] << 16U) | ((uint32_t)payload[-5] << 8U) | payload[-4];
                apb->TX_PAYLOAD = ((uint32_t)payload[-3]  << 8U) | payload[-2];
                break;
            case 4:
                payload += 3;
                apb->TX_PAYLOAD = ((uint32_t)payload[-3] << 16U) | ((uint32_t)payload[-2] << 8U) | payload[-1];
                break;
            case 1:
                break;
            default:
                /* For MISRA 2012 16.4 */
                break;
        }
    }
    else
    {
        payloadSize &= ~0x03U;
    }

    xfer->txData += (payloadSize + 4);
    xfer->txDataSize -= (payloadSize + 3);

    return payloadSize + 4;
}

static uint16_t DSI_WriteDataStep1_XRGB8888(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint32_t firstWord;
    uint16_t i, payloadSize;
    const uint8_t *payload;

    DSI_HOST_APB_PKT_IF_Type *apb = base->apb;

    if (xfer->txDataSize > xfer->txFifoSize)
    {
        payloadSize = xfer->txFifoSize + 1;
    }
    else
    {
        payloadSize = xfer->txDataSize + 1;
    }

    payload = xfer->txData;
    firstWord = xfer->dscCmd;

    assert(payloadSize >= 5);

    if (payloadSize == 5)
    {
        firstWord |= ((uint32_t)payload[0] << 24U) | ((uint32_t)payload[1] << 16U) | ((uint32_t)payload[2] << 8U);

        apb->TX_PAYLOAD = firstWord;
        xfer->txData += 4;
        xfer->txDataSize -= 4;
        return payloadSize;
    }
    else
    {
        firstWord |= ((uint32_t)payload[0] << 24U) | ((uint32_t)payload[1] << 16U) | ((uint32_t)payload[2] << 8U);
        apb->TX_PAYLOAD = (uint32_t)firstWord;
        payload += 4;
        payloadSize -= 4;
    }

    for (i = 0; i < payloadSize >> 4; i++)
    {
        payload += 16;

        apb->TX_PAYLOAD = ((uint32_t)payload[-10] << 24U) | ((uint32_t)payload[-16] << 16U) | ((uint32_t)payload[-15] << 8U) | payload[-14];

        apb->TX_PAYLOAD = ((uint32_t)payload[-7]  << 24U) | ((uint32_t)payload[-6]  << 16U) | ((uint32_t)payload[-12] << 8U) | payload[-11];

        apb->TX_PAYLOAD = ((uint32_t)payload[-4]  << 24U) | ((uint32_t)payload[-3]  << 16U) | ((uint32_t)payload[-2]  << 8U) | payload[-8];
    }
    /* Only write the remaining data while the txDataSize <= max available FIFO size */
    if (xfer->txDataSize <= xfer->txFifoSize)
    {
        switch (payloadSize & 0x0FU)
        {
            case 13:
                payload += 12;
                apb->TX_PAYLOAD = ((uint32_t)payload[-6] << 24U) | ((uint32_t)payload[-12] << 16U) | ((uint32_t)payload[-11] << 8U) | payload[-14];
                apb->TX_PAYLOAD = ((uint32_t)payload[-3]  << 24U) | ((uint32_t)payload[-2]  << 16U) | ((uint32_t)payload[-8] << 8U) | payload[-7];
                apb->TX_PAYLOAD = payload[-4];
                break;
            case 9:
                payload += 8;
                apb->TX_PAYLOAD = ((uint32_t)payload[-2] << 24U) | ((uint32_t)payload[-8] << 16U) | ((uint32_t)payload[-7] << 8U) | payload[-6];
                apb->TX_PAYLOAD = ((uint32_t)payload[-4] << 8U) | payload[-3];
                break;
            case 5:
                payload += 4;
                apb->TX_PAYLOAD = ((uint32_t)payload[-4] << 16U) | ((uint32_t)payload[-3] << 8U) | payload[-2];
                break;
            case 1:
                break;
            default:
                /* For MISRA 2012 16.4 */
                break;
        }
    }
    else
    {
        payloadSize &= ~0x03U;
    }

    xfer->txData += (payloadSize + 4);
    xfer->txDataSize -= (payloadSize + 3);

    return ((payloadSize + 3) >> 2) * 3 + 1;
}

static status_t DSI_WriteDataStep1(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint16_t wordCount, payloadSize;
    uint32_t intFlags1, intFlags2;

    /* ========================== Prepare TX. ========================== */
    /* If xfer->sendDscCmd is true, then the DSC command is not included in the
       xfer->txData, but specified by xfer->dscCmd.
     */
    assert(xfer->sendDscCmd);
    assert(xfer->txDataSize <= xfer->txDataSizeMax);

    payloadSize = xfer->txDataSize + 1;

    if (xfer->inputFormat == kDSI_AUX_InputPixelFormatRGB565)
    {
        wordCount = DSI_WriteDataStep1_RGB565(base, xfer);
    }
    else if (xfer->inputFormat == kDSI_AUX_InputPixelFormatRGB888)
    {
        wordCount = DSI_WriteDataStep1_RGB888(base, xfer);
    }
    else
    {
        wordCount = DSI_WriteDataStep1_XRGB8888(base, xfer);
        payloadSize = ((payloadSize - 1) >> 2) * 3 + 1;
    }

    assert(wordCount <= payloadSize);
    DSI_SetApbPacketControl(base, payloadSize, xfer->virtualChannel, xfer->txDataType, xfer->flags);

    /* Clear the interrupt flags set by previous transfer. */
    DSI_GetAndClearInterruptStatus(base, &intFlags1, &intFlags2);

    return kStatus_Success;
}

static void DSI_WriteDataStep2_RGB565(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint16_t i, payloadSize;
    const uint8_t *payload;

    DSI_HOST_APB_PKT_IF_Type *apb = base->apb;

    payload = xfer->txData;
    payloadSize = xfer->txDataSize;

    for (i = 0; i < payloadSize / 4U; i++)
    {
        apb->TX_PAYLOAD = ((uint32_t)payload[3] << 24U) | ((uint32_t)payload[0] << 16U) | ((uint32_t)payload[1] << 8U) | payload[-2];
        payload += 4;
    }

    /* Write the remaining data. */
    switch (payloadSize & 0x03U)
    {
        case 3:
            apb->TX_PAYLOAD =  ((uint32_t)payload[0] << 16U) | ((uint32_t)payload[1] << 8U) | payload[-2];
            break;
        case 1:
            apb->TX_PAYLOAD = payload[-2];
            break;
        default:
            /* For MISRA 2012 16.4 */
            break;
    }
}

static void DSI_WriteDataStep2_RGB888(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint16_t i, payloadSize;
    const uint8_t *payload;

    DSI_HOST_APB_PKT_IF_Type *apb = base->apb;

    payload = xfer->txData;
    payloadSize = xfer->txDataSize;

    for (i = 0; i < payloadSize >> 2; i++)
    {
        payload += 12;

        apb->TX_PAYLOAD = ((uint32_t)payload[-7] << 24U) | ((uint32_t)payload[-12] << 16U) | ((uint32_t)payload[-11] << 8U) | payload[-10];

        apb->TX_PAYLOAD = ((uint32_t)payload[-5] << 24U) | ((uint32_t)payload[-4]  << 16U) | ((uint32_t)payload[-9]  << 8U) | payload[-8];

        apb->TX_PAYLOAD = ((uint32_t)payload[-3] << 24U) | ((uint32_t)payload[-2]  << 16U) | ((uint32_t)payload[-1]  << 8U) | payload[-6];
    }

    /* Write the remaining data. */
    switch (payloadSize & 0x0FU)
    {
        case 10:
            payload += 9;
            apb->TX_PAYLOAD = ((uint32_t)payload[-4] << 24U) | ((uint32_t)payload[-9] << 16U) | ((uint32_t)payload[-8] << 8U) | payload[-7];
            apb->TX_PAYLOAD = ((uint32_t)payload[-2] << 24U) | ((uint32_t)payload[-1]  << 16U) | ((uint32_t)payload[-6]  << 8U) | payload[-5];
            apb->TX_PAYLOAD = payload[-3];
            break;
        case 7:
            payload += 6;
            apb->TX_PAYLOAD = ((uint32_t)payload[-1] << 24U) | ((uint32_t)payload[-6] << 16U) | ((uint32_t)payload[-5] << 8U) | payload[-4];
            apb->TX_PAYLOAD = ((uint32_t)payload[-3]  << 8U) | payload[-2];
            break;
        case 4:
            payload += 3;
            apb->TX_PAYLOAD = ((uint32_t)payload[-3] << 16U) | ((uint32_t)payload[-2] << 8U) | payload[-1];
            break;
        case 1:
            break;
        default:
            /* For MISRA 2012 16.4 */
            break;
    }
}

static void DSI_WriteDataStep2_XRGB8888(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    uint16_t i, payloadSize;
    const uint8_t *payload;

    DSI_HOST_APB_PKT_IF_Type *apb = base->apb;

    payload = xfer->txData;
    payloadSize = xfer->txDataSize;

    for (i = 0; i < payloadSize >> 4; i++)
    {
        payload += 16;

        apb->TX_PAYLOAD = ((uint32_t)payload[-10] << 24U) | ((uint32_t)payload[-16] << 16U) | ((uint32_t)payload[-15] << 8U) | payload[-14];

        apb->TX_PAYLOAD = ((uint32_t)payload[-7]  << 24U) | ((uint32_t)payload[-6]  << 16U) | ((uint32_t)payload[-12] << 8U) | payload[-11];

        apb->TX_PAYLOAD = ((uint32_t)payload[-4]  << 24U) | ((uint32_t)payload[-3]  << 16U) | ((uint32_t)payload[-2]  << 8U) | payload[-8];
    }

    switch (payloadSize & 0x0FU)
    {
        case 13:
            apb->TX_PAYLOAD = ((uint32_t)payload[-6] << 24U) | ((uint32_t)payload[-12] << 16U) | ((uint32_t)payload[-11] << 8U) | payload[-14];
            apb->TX_PAYLOAD = ((uint32_t)payload[-3]  << 24U) | ((uint32_t)payload[-2]  << 16U) | ((uint32_t)payload[-8] << 8U) | payload[-7];
            apb->TX_PAYLOAD = payload[-4];
            break;
        case 9:
            apb->TX_PAYLOAD = ((uint32_t)payload[-2] << 24U) | ((uint32_t)payload[-8] << 16U) | ((uint32_t)payload[-7] << 8U) | payload[-6];
            apb->TX_PAYLOAD = ((uint32_t)payload[-4] << 8U) | payload[-3];
            break;
        case 5:
            apb->TX_PAYLOAD = ((uint32_t)payload[-4] << 16U) | ((uint32_t)payload[-3] << 8U) | payload[-2];
            break;
        case 1:
            break;
        default:
            /* For MISRA 2012 16.4 */
            break;
    }
}

static status_t DSI_WriteDataStep2(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    if (xfer->inputFormat == kDSI_AUX_InputPixelFormatRGB565)
    {
        DSI_WriteDataStep2_RGB565(base, xfer);
    }
    else if (xfer->inputFormat == kDSI_AUX_InputPixelFormatRGB888)
    {
        DSI_WriteDataStep2_RGB888(base, xfer);
    }
    else
    {
        DSI_WriteDataStep2_XRGB8888(base, xfer);
    }
    return kStatus_Success;
}

static status_t DSI_DCSTransfer(const MIPI_DSI_Type *base, dsi_transfer_ext_t *xfer)
{
    status_t status;

    while (0U == (base->apb->PKT_STATUS & (uint32_t)kDSI_ApbTxDone)) ;

    status = DSI_WriteDataStep1(base, xfer);

    if (kStatus_Success != status)
    {
        return status;
    }

    DSI_SendApbPacket(base);

    if (xfer->txDataSize > 0)
    {
        status = DSI_WriteDataStep2(base, xfer);
    }

    if (kStatus_Success != status)
    {
        return status;
    }

    return kStatus_Success;
}

static status_t BOARD_DsiMemWriteSendChunck(dsi_aux_write_mem_transfer_t *x_fer)
{
    uint32_t curSendLen;

    curSendLen =
        s_dsiMemWriteCtx.txDataSizeMax > s_dsiMemWriteCtx.leftByteLen ? s_dsiMemWriteCtx.leftByteLen : s_dsiMemWriteCtx.txDataSizeMax;

    s_dsiMemWriteXfer.txDataType    = kDSI_TxDataDcsLongWr;
    s_dsiMemWriteXfer.dscCmd        = s_dsiMemWriteCtx.dscCmd;
    s_dsiMemWriteXfer.txData        = s_dsiMemWriteCtx.txData;
    s_dsiMemWriteXfer.txDataSize    = curSendLen;
    s_dsiMemWriteXfer.txDataSizeMax = s_dsiMemWriteCtx.txDataSizeMax;
    s_dsiMemWriteXfer.txFifoSize = s_dsiMemWriteCtx.txFifoSize;

    s_dsiMemWriteCtx.txData += curSendLen;
    s_dsiMemWriteCtx.leftByteLen -= (curSendLen);
    s_dsiMemWriteCtx.dscCmd = kMIPI_DCS_WriteMemoryContinue;

    return DSI_DCSTransfer(DEMO_MIPI_DSI, &s_dsiMemWriteXfer);
}

static status_t DSI_MemWrite(dsi_aux_write_mem_transfer_t *x_fer)
{
    status_t status;

    if (s_dsiMemWriteCtx.onGoing)
    {
        return kStatus_Fail;
    }

    if (x_fer->inputFormat == kDSI_AUX_InputPixelFormatRGB565)
    {
        s_dsiMemWriteCtx.txDataSizeMax =  (FSL_DSI_TX_MAX_CHUNCK_BYTE - 1U) / 2 * 2;
        s_dsiMemWriteCtx.txFifoSize = (FSL_DSI_TX_MAX_PAYLOAD_BYTE - 1U) / 2 * 2;
    }
    else if (x_fer->inputFormat == kDSI_AUX_InputPixelFormatRGB888)
    {
        s_dsiMemWriteCtx.txDataSizeMax =  (FSL_DSI_TX_MAX_CHUNCK_BYTE - 1U) / 3 * 3;
        s_dsiMemWriteCtx.txFifoSize = (FSL_DSI_TX_MAX_PAYLOAD_BYTE - 1U) / 3 * 3;
    }
    else
    {
        s_dsiMemWriteCtx.txDataSizeMax =  (FSL_DSI_TX_MAX_CHUNCK_BYTE - 1U) / 3 * 4;
        s_dsiMemWriteCtx.txFifoSize = (FSL_DSI_TX_MAX_PAYLOAD_BYTE - 1U) / 3 * 4;
    }

    s_dsiMemWriteXfer.inputFormat    = x_fer->inputFormat;
    s_dsiMemWriteXfer.outputFormat   = x_fer->outputFormat;
    s_dsiMemWriteXfer.virtualChannel = x_fer->virtualChannel;
    s_dsiMemWriteXfer.flags          = kDSI_TransferUseHighSpeed;
    s_dsiMemWriteXfer.sendDscCmd     = true;

    s_dsiMemWriteCtx.onGoing     = true;
    s_dsiMemWriteCtx.txData      = x_fer->data;
    s_dsiMemWriteCtx.leftByteLen = x_fer->dataSize;
    s_dsiMemWriteCtx.dscCmd      = kMIPI_DCS_WriteMemoryStart;

    while (s_dsiMemWriteCtx.leftByteLen > 0)
    {
        status = BOARD_DsiMemWriteSendChunck(x_fer);
        s_dsiMemWriteCtx.onGoing     = false;
        if (status != kStatus_Success)
          break;
    }
    return status;
}

 static void RemoteReadyEventHandler(uint16_t eventData, void *context)
 {
     status_t status;
     uint32_t cycles = 0;


     if (kDSI_AUX_ImageDataReadyForSent == eventData)
     {
         CoreDebug->DEMCR |= (1 << CoreDebug_DEMCR_TRCENA_Pos);

         DWT->CYCCNT = 0;
         DWT->CTRL |= (1 << DWT_CTRL_CYCCNTENA_Pos);

         status = DSI_MemWrite((dsi_aux_write_mem_transfer_t *)context);

         cycles = DWT->CYCCNT;
         DWT->CTRL &= ~(1 << DWT_CTRL_CYCCNTENA_Pos);
         time =  cycles / (armCM4_Hz / 1000.0);

         if (kStatus_Success == status)
         {
             MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, (uint16_t)kDSI_AUX_IMageDataSentdDone);
         }
         else
         {
             MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, (uint16_t)kDSI_AUX_IMageDataSentdError);
         }
     }
 }
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

/*!
 * @brief Main function
 */
int main(void)
{
    mcmgr_status_t status;

    /* Init board hardware.*/
    BOARD_ConfigMPU();
    BOARD_InitPins();

    armCM4_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_M4);

    /* Initialize MCMGR, install generic event handlers */
    (void)MCMGR_Init();

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData((uint32_t*)&g_xfer);
    } while (status != kStatus_MCMGR_Success);

    /* Register the application event before getting sending ready signal */
    MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RemoteReadyEventHandler, (void *)g_xfer);

    /* Notify the other core we are ready */
    MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, (uint16_t) kDSI_AUX_RemoteCoreReady);

    for (;;)
    {
        SDK_DelayAtLeastUs(500000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    }
}
