/*
 * FreeRTOS+TCP V3.1.0
 * Copyright (C) 2022 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/*****************************************************************************
 * Note: This file is Not! to be used as is. The purpose of this file is to provide
 * a template for writing a network interface. Each network interface will have to provide
 * concrete implementations of the functions in this file.
 *
 * See the following URL for an explanation of this file and its functions:
 * https://freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/Embedded_Ethernet_Porting.html
 *
 *****************************************************************************/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "list.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"

/* Altera includes */
#include <altera_avalon_tse_system_info.h>
#include <altera_avalon_tse.h>
#include <altera_eth_tse_regs.h>
#include <altera_msgdma.h>
#include <sys/alt_cache.h>
#include <sys/alt_errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// us timeout for TSE IP reset polling
#define TSE_RESET_TIMEOUT 10

// Deferred ISR task parameters
#define RX_DEFERRED_ISR_STACKSIZE  ( configMINIMAL_STACK_SIZE )
#define RX_DEFERRED_ISR_PRIORITY  configMAX_PRIORITIES - 1

// MSGDMA Prefetcher descriptor oc memory parameters
#define MSGDMA_PREFETCHER_STANDARD_DESCRIPTOR_SIZE 0x20 // 256 bit = 32 bytes
#define MSGDMA_DESCRIPTOR_SIZE MSGDMA_PREFETCHER_STANDARD_DESCRIPTOR_SIZE // using standard descriptors

//#define RX_DESCRIPTOR_LIST_SIZE (DMA_DESC_RAM_SIZE_VALUE / MSGDMA_DESCRIPTOR_SIZE / 2)
#define RX_DESCRIPTOR_LIST_SIZE 6
#define TX_DESCRIPTOR_LIST_SIZE RX_DESCRIPTOR_LIST_SIZE
#define RX_DESCRIPTOR_LIST_START TSE_DMA_DESC_RAM_BASE //from system.h
#define TX_DESCRIPTOR_LIST_START (RX_DESCRIPTOR_LIST_START + RX_DESCRIPTOR_LIST_SIZE * MSGDMA_DESCRIPTOR_SIZE)
// using altera msgdma standard descriptors
typedef  alt_msgdma_prefetcher_standard_descriptor t_descr;

// static buffer allocation properties
#define BUFFER_SIZE ( ipTOTAL_ETHERNET_FRAME_SIZE + ipBUFFER_PADDING )
#define BUFFER_SIZE_ROUNDED_UP ( ( BUFFER_SIZE + 7 ) & ~0x07UL )

// filling altera tse system structure, names taken from system.h
alt_tse_system_info tse_mac_device[MAXNETS] = {
  TSE_SYSTEM_EXT_MEM_NO_SHARED_FIFO(TSE,              // tse_name
                                    0,                    // offset
                                    TSE_TX_MSGDMA,    // msgdma_tx_name
                                    TSE_RX_MSGDMA,    // msgdma_rx_name
                                    TSE_PHY_AUTO_ADDRESS, // phy_addr
                                    NULL,                 // phy_cfg_fp
                                    TSE_DMA_DESC_RAM          // desc_mem_name
                                    )
};

// additional configuration function for GPY115 
alt_32 GPY115_config(np_tse_mac *pmac)
{
  alt_u16 dat;

  dat = IORD(&pmac->mdio1.ADV, 0);
  dat &= 0xFFFF;
  dat &= ~(0x0380);
  IOWR(&pmac->mdio1.ADV, 0, dat);

  dat = IORD(&pmac->mdio1.reg9, 0);
  dat &= 0xFFFF;
  dat &= ~(0x0200);
  IOWR(&pmac->mdio1.reg9, 0, dat);

  dat = IORD(&pmac->mdio1.reg14, 0);
  dat |= 0x100;
  dat &= ~(0x6);
  IOWR(&pmac->mdio1.reg14, 0, dat);

  return 0;

}

// function to read out negotiated speed and mode
alt_u32 GPY115_link_status_read(np_tse_mac *pmac) {
  alt_u32 link_status = 0;
  usleep(1000000);
  alt_u32 reg = IORD(&pmac->mdio1.reg18, 0);
  alt_u32 speed = reg & 0x7;
  if(speed > 3)
    return (0x1 << 16);
  else if(speed == 3)
    return (0x1 << 19);
  else if(speed == 2)
    link_status = (0x1 << 1);
  else if(speed == 1)
    link_status = (0x1 << 2);
  else if(speed == 0)
    link_status = (0x1 << 3);

  link_status |= ((reg & 0x8) >> 3);
  return link_status;
}

// GPY115 init struct
alt_tse_phy_profile GPY115C0VI = {"MaxLinear GPY115C0VI",
                                  0x19f277,                   /* OUI                                                        */
                                  0x31,                 /* Vender Model Number                                        */
                                  0x0,                   /* Model Revision Number                                      */
                                  0,                              /* Location of Status Register (ignored)                      */
                                  0,                              /* Location of Speed Status    (ignored)                      */
                                  0,                              /* Location of Duplex Status   (ignored)                      */
                                  0,                              /* Location of Link Status     (ignored)                      */
                                  0,//&GPY115_config,                              /* No function pointer configure              */
                                  &GPY115_link_status_read      /* Function pointer to read from PHY specific status register */           
};

// shared struct to exchange DMA descriptor information between tasks
typedef struct NET_IF_INFO {
  alt_tse_system_info *pTseSystemInfo;
  np_tse_mac *pTseCsr;
  alt_msgdma_dev *rxMsgdma;
  alt_msgdma_dev *txMsgdma;
  t_descr *pRxDescrListStart;
  t_descr *pTxDescrListStart;
  t_descr *pRxHwDescr;
  t_descr *pRxSwDescr;
  uint8_t u8bRxHwDescrNum;
  t_descr *pTxSwDescr;
  t_descr *pTxHwDescr;
} NET_IF_INFO;
static NET_IF_INFO netIfInfo;

/* The deferred interrupt handler is a standard RTOS task.  FreeRTOS's centralised
   deferred interrupt handling capabilities can also be used. */
static TaskHandle_t pxDeferredIsrTask;
static void prvEMACDeferredInterruptHandlerTask( void *pvParameters )
{
  NetworkBufferDescriptor_t *pxDescriptor;
  NetworkBufferDescriptor_t *pxDescriptor_new;
  NetworkBufferDescriptor_t *pxHead = NULL;
  NetworkBufferDescriptor_t *pxTail = NULL;
  t_descr *descr;
  // Used to indicate that xSendEventStructToIPTask() is being called because
  // of an Ethernet receive event
  IPStackEvent_t xRxEvent;
  uint32_t ulNotifiedValue;

  NET_IF_INFO *pNetIfInfo = (NET_IF_INFO *) pvParameters;
  descr = pNetIfInfo->pRxDescrListStart;

  for( int j = 0;; ++j)
    {
      // Wait for the Ethernet MAC interrupt to indicate that another packet
      // has been received.  The task notification is used in a similar way to a
      // counting semaphore to count Rx events, but is a lot more efficient than
      // a semaphore.
      ulNotifiedValue = ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

      // get the descriptor pointer for the filled buffer by DMA
      pxDescriptor = *( ( NetworkBufferDescriptor_t ** )(descr->write_address - ipBUFFER_PADDING) );
      alt_dcache_flush((void*)descr->write_address, ipTOTAL_ETHERNET_FRAME_SIZE);
      // update bytes transferred field
      pxDescriptor->xDataLength = descr->bytes_transfered;

      // Allocate a new network buffer descriptor that references an Ethernet
      // frame large enough to hold the maximum network packet size (as defined
      // in the FreeRTOSIPConfig.h header file). 
      pxDescriptor_new = pxGetNetworkBufferWithDescriptor( ipTOTAL_ETHERNET_FRAME_SIZE, portMAX_DELAY );
      alt_dcache_flush(pxDescriptor_new->pucEthernetBuffer, pxDescriptor_new->xDataLength);
      // copy new buffer address into DMA descriptor
      descr->write_address = ((alt_u32) pxDescriptor_new->pucEthernetBuffer);

      taskENTER_CRITICAL();
      // move SW DMA descripter pointer to the next one
      pNetIfInfo->pRxSwDescr = (t_descr*)(descr->next_desc_ptr);
      // Give DMA descriptor to HW
      if ((t_descr*)(pNetIfInfo->pRxHwDescr) == descr) {
        pNetIfInfo->pRxHwDescr->control           |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;
      }
      taskEXIT_CRITICAL();
      descr = (t_descr*)(descr->next_desc_ptr);

      // See if the data contained in the received Ethernet frame needs
      // to be processed.  NOTE! It might be possible to do this in
      // the interrupt service routine itself, which would remove the need
      // to unblock this task for packets that don't need processing.
      if ( eConsiderFrameForProcessing( pxDescriptor->pucEthernetBuffer ) ) {
        xRxEvent.eEventType = eNetworkRxEvent;
        // pvData is used to point to the network buffer descriptor that
        // references the received data.
        xRxEvent.pvData = ( void * ) pxDescriptor;
        if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE ) {
          // The buffer could not be sent to the IP task so the buffer
          // must be released.
          vReleaseNetworkBufferAndDescriptor( pxDescriptor );
          // Make a call to the standard trace macro to log the
          // occurrence.
          iptraceETHERNET_RX_EVENT_LOST();
        }
        else {
          // The message was successfully sent to the TCP/IP stack.
          // Call the standard trace macro to log the occurrence. */
          iptraceNETWORK_INTERFACE_RECEIVE();
        }
      } else {
        vReleaseNetworkBufferAndDescriptor( pxDescriptor );
      }
    }
}

static void inline rx_msgdma_isr(NET_IF_INFO *pNetIfInfo) {
  BaseType_t xHigherPriorityTaskWoken;
  // Re_enable global interrupts so we don't miss one that occurs during the
  // processing of this ISR.
  alt_msgdma_dev *rx_msgdma = pNetIfInfo->rxMsgdma;
  alt_u32        reg_data_32;
  reg_data_32  = IORD_ALT_MSGDMA_PREFETCHER_CONTROL(rx_msgdma->prefetcher_base);
  reg_data_32 |= ALT_MSGDMA_PREFETCHER_CTRL_GLOBAL_INTR_EN_SET_MASK;
  IOWR_ALT_MSGDMA_PREFETCHER_CONTROL(rx_msgdma->prefetcher_base, reg_data_32);
  IOWR_ALT_MSGDMA_PREFETCHER_STATUS(rx_msgdma->prefetcher_base, ALT_MSGDMA_PREFETCHER_STATUS_IRQ_CLR_MASK);

  // move HW descriptor pointer to the next descriptor
  pNetIfInfo->pRxHwDescr = (t_descr*)(pNetIfInfo->pRxHwDescr->next_desc_ptr);

  // check that all previous DMA descriptors have been procesed by the SW
  if ((t_descr*)(pNetIfInfo->pRxHwDescr) != pNetIfInfo->pRxSwDescr) {
    // give next descriptor to HW
    pNetIfInfo->pRxHwDescr->control           |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;
  }
  /* xHigherPriorityTaskWoken must be initialised to pdFALSE.
     If calling vTaskNotifyGiveFromISR() unblocks the handling
     task, and the priority of the handling task is higher than
     the priority of the currently running task, then
     xHigherPriorityTaskWoken will be automatically set to pdTRUE. */
  xHigherPriorityTaskWoken = pdFALSE;
  /* Unblock the handling task so the task can perform
     any processing necessitated by the interrupt.  xHandlingTask
     is the task's handle, which was obtained when the task was
     created.  vTaskNotifyGiveFromISR() also increments
     the receiving task's notification value. */
  vTaskNotifyGiveFromISR( pxDeferredIsrTask, &xHigherPriorityTaskWoken );
  /* Force a context switch if xHigherPriorityTaskWoken is now
     set to pdTRUE. The macro used to do this is dependent on
     the port and may be called portEND_SWITCHING_ISR. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void inline tx_msgdma_isr(NET_IF_INFO *pNetIfInfo) {
  alt_u32        reg_data;
  reg_data = IORD_ALTERA_MSGDMA_CSR_STATUS(pNetIfInfo->txMsgdma->csr_base);
  if ((reg_data & ALTERA_MSGDMA_CSR_STOPPED_ON_ERROR_MASK) ||
      (reg_data & ALTERA_MSGDMA_CSR_STOPPED_ON_EARLY_TERMINATION_MASK)) {
    FreeRTOS_printf( ("Tx mSGDMA descriptor is in error.\n") );
    return;
  }
  // release network buffer after it has been sent
  vNetworkBufferReleaseFromISR( *( (NetworkBufferDescriptor_t**)(netIfInfo.pTxHwDescr->read_address - ipconfigBUFFER_PADDING) ) );
  netIfInfo.pTxHwDescr = (t_descr*)netIfInfo.pTxHwDescr->next_desc_ptr;
}

extern int alt_msgdma_start_prefetcher_with_list_addr (
                                                       alt_msgdma_dev *dev,
                                                       alt_u64  list_addr,
                                                       alt_u8 park_mode_en,
                                                       alt_u8 poll_en);

BaseType_t xNetworkInterfaceInitialise( void )
{
  // speed and duplex returned by getPHYSpeed
  uint32_t duplex;
  uint32_t speed;
  // temp
  uint32_t result;
  uint32_t reg_data;
  uint8_t count;


  // Create shared struct
  netIfInfo.pTseSystemInfo = &(tse_mac_device[0]);
  netIfInfo.pTseCsr = (np_tse_mac*)(netIfInfo.pTseSystemInfo->tse_mac_base);

  // Add PHY profie
  alt_tse_phy_add_profile(&GPY115C0VI);

  // Initialize PHY and PCS by calling getPHYSpeed()
  // uC-TCP-IP does that
  {
    FreeRTOS_printf( ("PHY and PCS Initialization: Starting.\n") );
    result = getPHYSpeed(netIfInfo.pTseCsr);
    if ((result >> 16) & 0xFF) {
      if (result & ALT_TSE_E_INVALID_SPEED)
        FreeRTOS_printf( ("getPHYSpeed: Invalid speed read from PHY.\n") );
      if (result & ALT_TSE_E_PROFILE_INCORRECT_DEFINED)
        FreeRTOS_printf( ("getPHYSpeed: PHY Profile is not defined correctly.\n") );
      if (result & ALT_TSE_E_NO_PHY_PROFILE)
        FreeRTOS_printf( ("getPHYSpeed: No PHY profile matched the detected PHY.\n") );
      if (result & ALT_TSE_E_AN_NOT_COMPLETE)
        FreeRTOS_printf( ("getPHYSpeed: Auto-negotiation not completed.\n") );
      if (result & ALT_TSE_E_NO_COMMON_SPEED)
        FreeRTOS_printf( ("getPHYSpeed: No common speed found for multi-port MAC.\n") );
      if (result & ALT_TSE_E_NO_PHY)
        FreeRTOS_printf( ("getPHYSpeed: No PHY detected.\n") );
      if (result & ALT_TSE_E_NO_MDIO)
        FreeRTOS_printf( ("getPHYSpeed: No MDIO used by the MAC.\n") );
      if (result & ALT_TSE_E_NO_PMAC_FOUND)
        FreeRTOS_printf( ("getPHYSpeed: Argument *pmac not found from the list of MAC detected during init.\n") );
      return pdFALSE;
    }
    speed  = (result >> 1) & 0x07;
    duplex = result & 0x01;
    FreeRTOS_printf( ("getPHYSpeed: speed = %d ; duplex = %d\n", (int)speed, (int)duplex) );
    FreeRTOS_printf( ("PHY and PCS Initialization: Success.\n") );
  }

  // TSE configuration
  alt_u32 *tse_base = (alt_u32*)netIfInfo.pTseCsr;
  alt_tse_system_info *tse_sys_info = netIfInfo.pTseSystemInfo;
  {
    FreeRTOS_printf( ("MAC Initialization: Starting.\n") );
    FreeRTOS_printf( ("MAC Software Reset: Starting.\n") );
    // Reset and disable TX and RX in TSE
    reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
    reg_data |= (ALTERA_TSEMAC_CMD_SW_RESET_MSK | ALTERA_TSEMAC_CMD_CNT_RESET_MSK);
    reg_data &= (~ALTERA_TSEMAC_CMD_RX_ENA_MSK);
    reg_data &= (~ALTERA_TSEMAC_CMD_TX_ENA_MSK);
    IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
    // wait for reset with timeout
    count = 0;
    while (1) {
      // delay 1 us
      vTaskDelay(0.001 / portTICK_PERIOD_MS);
      // read status
      reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
      // if reset finished -> break
      if ((reg_data & ALTERA_TSEMAC_CMD_SW_RESET_MSK) == 0) {
        break;
      }
      // Give up if reset takes too long.
      if (count > TSE_RESET_TIMEOUT) {
        FreeRTOS_printf( ("MAC reset: FAILED, Timed out!\n") );
        return pdFALSE;
      }
      ++count;
    }
    FreeRTOS_printf( ("MAC Software Reset: Success.\n") );
  }

  {
    FreeRTOS_printf( ("MAC FIFO Configuration: INFO: No shared fifo.\n") );
    // Tx_section_empty = Max FIFO size - 16
    IOWR_ALTERA_TSEMAC_TX_SECTION_EMPTY(tse_base, tse_sys_info->tse_tx_depth - 16);
    // Tx_almost_full = 3
    IOWR_ALTERA_TSEMAC_TX_ALMOST_FULL  (tse_base, 3);
    // Tx_almost_empty = 8
    IOWR_ALTERA_TSEMAC_TX_ALMOST_EMPTY (tse_base, 8);

    // Rx_section_empty = Max FIFO size - 16
    IOWR_ALTERA_TSEMAC_RX_SECTION_EMPTY(tse_base, tse_sys_info->tse_rx_depth - 16);
    // Rx_almost_full = 8
    IOWR_ALTERA_TSEMAC_RX_ALMOST_FULL  (tse_base, 8);
    // Rx_almost_empty = 8
    IOWR_ALTERA_TSEMAC_RX_ALMOST_EMPTY (tse_base, 8);

    // Tx_section_full = 16
    IOWR_ALTERA_TSEMAC_TX_SECTION_FULL (tse_base,  0);
    // Rx_section_full = 16
    IOWR_ALTERA_TSEMAC_RX_SECTION_FULL (tse_base,  0);
    FreeRTOS_printf( ("MAC FIFO Configuration: Success.\n") );
  }

  // setting mac
  // read MAC address provbided by the stack
  const uint8_t *hw_addr = FreeRTOS_GetMACAddress();
  {
    FreeRTOS_printf( ("MAC Address Configuration : Starting.\n") );
    {
      // configure TX_ADDR_SEL to select mac_0 mac_1 as MAC address source
      reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
      reg_data &= 0xFFF8FFFF;
      IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
      // configure MAC address
      reg_data = (((uint32_t)hw_addr[0]) |
                  ((uint32_t)hw_addr[1] << (1 * 8)) |
                  ((uint32_t)hw_addr[2] << (2 * 8)) |
                  ((uint32_t)hw_addr[3] << (3 * 8)));
      IOWR_ALTERA_TSEMAC_MAC_0(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_0_0(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_1_0(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_2_0(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_3_0(tse_base, reg_data);

      reg_data = (((uint32_t)hw_addr[4]) |
                  ((uint32_t)hw_addr[5] << (1 * 8)));
      IOWR_ALTERA_TSEMAC_MAC_1(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_0_1(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_1_1(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_2_1(tse_base, reg_data);
      IOWR_ALTERA_TSEMAC_SMAC_3_1(tse_base, reg_data);
    }
    FreeRTOS_printf( ("MAC Address Configuration: Success.\n") );
  }

  //MAC Configuration
  {
    FreeRTOS_printf( ("MAC Function Configuration: Starting.\n") );

    // Set maximum frame length.
    IOWR_ALTERA_TSEMAC_FRM_LENGTH(tse_base, ALTERA_TSE_MAC_MAX_FRAME_LENGTH);
    // Maximum inter packet gap is already set by default (to 12).
    // Set maximum pause quanta value for flow control.
    IOWR_ALTERA_TSEMAC_PAUSE_QUANT(tse_base, 0xFFFF);
    // TSE mac direct register configuration.
    {
      reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
      // Remove frames with CRC errors.
      reg_data |= ALTERA_TSEMAC_CMD_RX_ERR_DISC_MSK;
      reg_data |= ALTERA_TSEMAC_CMD_PAD_EN_MSK;
      reg_data &= ~ALTERA_TSEMAC_CMD_CRC_FWD_MSK;
      // TX MAC address insertion on transmit packet.
      reg_data |= ALTERA_TSEMAC_CMD_TX_ADDR_INS_MSK;
      IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
    }

    /* Setting tx_cmd_stat rx_cmd_stat registers */
    {
      FreeRTOS_printf( ("Turning off Tx/Rx header boundary 32-bit alignment: Starting.\n") );
      // Tx boundary alignment.
      reg_data  = IORD_ALTERA_TSEMAC_TX_CMD_STAT(tse_base);
      reg_data &= ~ALTERA_TSEMAC_TX_CMD_STAT_TXSHIFT16_MSK;
      //reg_data |= ALTERA_TSEMAC_TX_CMD_STAT_TXSHIFT16_MSK;
      IOWR_ALTERA_TSEMAC_TX_CMD_STAT(tse_base, reg_data);
      // Ensure Tx boundary alignment disabled.
      reg_data = IORD_ALTERA_TSEMAC_TX_CMD_STAT(tse_base);
      if (reg_data & ALTERA_TSEMAC_TX_CMD_STAT_TXSHIFT16_MSK) {
        //if ( ! (reg_data & ALTERA_TSEMAC_TX_CMD_STAT_TXSHIFT16_MSK) ) {
        FreeRTOS_printf( ("TX header should NOT be boundary aligned: FAILED.\n") );
        return pdFALSE;
      }
      // Rx boundary alignment.
      reg_data  = IORD_ALTERA_TSEMAC_RX_CMD_STAT(tse_base);
      reg_data &= ~ALTERA_TSEMAC_RX_CMD_STAT_RXSHIFT16_MSK;
      //reg_data |= ALTERA_TSEMAC_RX_CMD_STAT_RXSHIFT16_MSK;
      IOWR_ALTERA_TSEMAC_RX_CMD_STAT(tse_base, reg_data);
      // Ensure Tx boundary alignment disabled.
      reg_data = IORD_ALTERA_TSEMAC_RX_CMD_STAT(tse_base);
      if (reg_data & ALTERA_TSEMAC_RX_CMD_STAT_RXSHIFT16_MSK) {
        //if (! (reg_data & ALTERA_TSEMAC_RX_CMD_STAT_RXSHIFT16_MSK) ) {
        FreeRTOS_printf( ("RX header should NOT be boundary aligned: FAILED.\n") );
        return pdFALSE;
      }
      FreeRTOS_printf( ("Turning off Tx/Rx header boundary 32-bit alignment: Success.\n") );
    }
    // Speed and Duplex setting
    reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
    if (speed == 0x01) {
      // 1000 Mbps
      reg_data |= ALTERA_TSEMAC_CMD_ETH_SPEED_MSK;
      reg_data &= ~ALTERA_TSEMAC_CMD_ENA_10_MSK;
    } else if (speed == 0x02) {
      // 100 Mbps
      reg_data &= ~ALTERA_TSEMAC_CMD_ETH_SPEED_MSK;
      reg_data &= ~ALTERA_TSEMAC_CMD_ENA_10_MSK;
    } else if (speed == 0x04) {
      // 10 Mbps
      reg_data &= ~ALTERA_TSEMAC_CMD_ETH_SPEED_MSK;
      reg_data |= ALTERA_TSEMAC_CMD_ENA_10_MSK;
    } else {
      // 100 Mbps default if speed unknown/invalid
      reg_data &= ~ALTERA_TSEMAC_CMD_ETH_SPEED_MSK;
      reg_data &= ~ALTERA_TSEMAC_CMD_ENA_10_MSK;
    }

    if(duplex == TSE_PHY_DUPLEX_HALF) {
      // Half Duplex
      reg_data |= ALTERA_TSEMAC_CMD_HD_ENA_MSK;
    } else {
      // Full Duplex
      reg_data &= ~ALTERA_TSEMAC_CMD_HD_ENA_MSK;
    }
    IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);

    FreeRTOS_printf( ("MAC Configuration: Success.\n") );
  }
  FreeRTOS_printf( ("MAC Initialization: Success.\n") );

  // get pointers to mSGDMA modules
  alt_msgdma_dev *rx_msgdma = alt_msgdma_open(TSE_RX_MSGDMA_CSR_NAME);
  if (rx_msgdma == NULL || ALT_ERRNO == ENODEV) {
    FreeRTOS_printf( ("Rx mSGDMA Open '%s': FAILED.\n", TSE_RX_MSGDMA_CSR_NAME) );
    return pdFALSE;
  }
  netIfInfo.rxMsgdma = rx_msgdma;

  alt_msgdma_dev *tx_msgdma = alt_msgdma_open(TSE_TX_MSGDMA_CSR_NAME);
  if (tx_msgdma == NULL || ALT_ERRNO == ENODEV) {
    FreeRTOS_printf( ("Tx mSGDMA Open '%s': FAILED.\n", TSE_TX_MSGDMA_CSR_NAME) );
    return pdFALSE;
  }
  netIfInfo.txMsgdma = tx_msgdma;

  // Setting RX mSGDMA descriptors list
  t_descr *rx_descr_list = (t_descr *) RX_DESCRIPTOR_LIST_START;
  netIfInfo.pRxDescrListStart = (rx_descr_list);
  netIfInfo.pRxHwDescr = rx_descr_list;
  netIfInfo.pRxSwDescr = rx_descr_list;
  t_descr *tx_descr_list = (t_descr *) TX_DESCRIPTOR_LIST_START;
  netIfInfo.pTxDescrListStart = tx_descr_list;
  netIfInfo.pTxSwDescr = tx_descr_list;
  netIfInfo.pTxHwDescr = tx_descr_list;

  // Create Deferred ISR task
  xTaskCreate(prvEMACDeferredInterruptHandlerTask, "Deffered_RX_handler", RX_DEFERRED_ISR_STACKSIZE, &netIfInfo, RX_DEFERRED_ISR_PRIORITY, &pxDeferredIsrTask );
  

  // Register mSGDMA ISR.
  {
    FreeRTOS_printf( ("mSGDMA RX ISR Registering: Starting.\n") );
    alt_msgdma_register_callback(
                                 rx_msgdma,                        // mSGDMA device
                                 (alt_msgdma_callback) rx_msgdma_isr, // callback function
                                 0,                                          // control
                                 &netIfInfo);                                       // arguments/context
    FreeRTOS_printf( ("mSGDMA RX ISR Registering: Success.\n") );

    FreeRTOS_printf( ("mSGDMA TX ISR Registering: Starting.\n") );
    alt_msgdma_register_callback(
                                 tx_msgdma,                        // mSGDMA device
                                 (alt_msgdma_callback) tx_msgdma_isr, // callback function
                                 0,                                          // control
                                 &netIfInfo);                                       // arguments/context
    FreeRTOS_printf( ("mSGDMA TX ISR Registering: Success.\n") );
  }
  // Initialize mSGDMA descriptors (and descriptor lists).
  {
    FreeRTOS_printf( ("mSGDMA descriptor list initialization: Starting.\n") );
    FreeRTOS_printf( ("RX mSGDMA Descriptor Initialization: Starting.\n") );
    // init desc lists
    t_descr *descr;
    t_descr *llist;
    uint8_t *buf_addr;

    llist = NULL;
    reg_data = 0;
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_END_ON_EOP_MASK;
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_ERROR_IRQ_MASK;
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_EARLY_TERMINATION_IRQ_MASK;
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_TRANSFER_COMPLETE_IRQ_MASK;

    for (int i = 0; i < RX_DESCRIPTOR_LIST_SIZE; ++i) {
      descr = (rx_descr_list + i);
      descr->next_desc_ptr = descr;
      size_t bufferSize = ipTOTAL_ETHERNET_FRAME_SIZE;
      NetworkBufferDescriptor_t *pxDescriptor = pxGetNetworkBufferWithDescriptor( ipTOTAL_ETHERNET_FRAME_SIZE, 1000 );
      buf_addr = pxDescriptor->pucEthernetBuffer;
      alt_dcache_flush(buf_addr, bufferSize);
      result = alt_msgdma_construct_prefetcher_standard_st_to_mm_descriptor(rx_msgdma, descr, (alt_u32) buf_addr, ipTOTAL_ETHERNET_FRAME_SIZE, reg_data);
      if (result != 0) {
        FreeRTOS_printf( ("RX mSGDMA initializing descriptor %d: FAILED.\n", i) );
        return pdFALSE;
      }
      result = alt_msgdma_prefetcher_add_standard_desc_to_list(&llist, descr);
      if (result != 0) {
        FreeRTOS_printf( ("RX mSGDMA add descriptor %d to list: FAILED.\n", i) );
        return pdFALSE;
      }
    }
    // Give the first RX descriptor to HW (set hw_owned bit to 1)
    rx_descr_list->control |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;
    // Start prefetcher with the list pointing on the first descriptor
    result = alt_msgdma_start_prefetcher_with_list_addr(rx_msgdma, (alt_u32)rx_descr_list, 0, 1);
    if (result != 0) {
      FreeRTOS_printf( ("mSGDMA Initialization: Failed to start Rx list.\n") );
      return pdFALSE;
    }
    IOWR_ALT_MSGDMA_PREFETCHER_DESCRIPTOR_POLLING_FREQ(rx_msgdma->prefetcher_base, 0xA);

    FreeRTOS_printf( ("TX mSGDMA Descriptor Initialization: Starting.\n") );
    llist = NULL;
    reg_data  = 0;                                                            /* Set control for Avalon MM-ST transfer.     */
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_GENERATE_SOP_MASK;           /* Emit start of packet.                      */
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MASK;           /* Emit end of packet.                        */
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_EARLY_TERMINATION_IRQ_MASK;  /* Interrupt if EOP not seen and above size.  */
    reg_data |= ALTERA_MSGDMA_DESCRIPTOR_CONTROL_TRANSFER_COMPLETE_IRQ_MASK;  /* Interrupt when succesful completion.       */
    for (int i = 0; i < TX_DESCRIPTOR_LIST_SIZE; ++i) {
      descr = (tx_descr_list + i);
      result = alt_msgdma_construct_prefetcher_standard_mm_to_st_descriptor(tx_msgdma, descr, (alt_u32)NULL, 0x0, reg_data);
      if (result != 0) {
        FreeRTOS_printf( ("TX mSGDMA initializing descriptor %d: FAILED.\n", i) );
        return pdFALSE;
      }
      result = alt_msgdma_prefetcher_add_standard_desc_to_list(&llist, descr);
      if (result != 0) {
        FreeRTOS_printf( ("TX mSGDMA add descriptor %d to list: FAILED.\n", i) );
        return pdFALSE;
      }
    }
    // Start TX prefetcher waiting (polling) for first TX packet
    result = alt_msgdma_start_prefetcher_with_list_addr(tx_msgdma, (alt_u32)tx_descr_list, 0, 1);
    if (result != 0) {
      FreeRTOS_printf( ("mSGDMA Initialization: Failed to start Tx list.\n") );
      return pdFALSE;
    }
    IOWR_ALT_MSGDMA_PREFETCHER_DESCRIPTOR_POLLING_FREQ(tx_msgdma->prefetcher_base, 0xA);
    FreeRTOS_printf( ("mSGDMA descriptor list initialization: Success.\n") );
  }

  // Enable MAC Transmit and Receive
  {
    FreeRTOS_printf( ("Enable MAC Transmit and Receive: Starting.\n") );
    // Write RX and TX enable bits.
    reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
    reg_data |= ALTERA_TSEMAC_CMD_RX_ENA_MSK;
    reg_data |= ALTERA_TSEMAC_CMD_TX_ENA_MSK;
    IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
    // Ensure Rx is enabled.
    reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
    if ((reg_data & ALTERA_TSEMAC_CMD_RX_ENA_MSK) == 0) {
      FreeRTOS_printf( ("Enable MAC Receive: FAILED.\n") );
      return pdFALSE;
    }
    // Ensure Tx is enabled.
    if ((reg_data & ALTERA_TSEMAC_CMD_TX_ENA_MSK) == 0) {
      FreeRTOS_printf( ("Enable MAC Transmit: FAILED.\n") );
      return pdFALSE;
    }
    FreeRTOS_printf( ("Enable MAC Transmit and Receive: Success.\n") );
  }

  return pdTRUE;
}

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer,
                                    BaseType_t xReleaseAfterSend )
{

  // get current TX software descriptor address
  t_descr *descr = netIfInfo.pTxSwDescr;
  alt_dcache_flush(pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength);

  // check if descriptor is still HW owned
  taskENTER_CRITICAL();
  uint8_t isDescrHwOwned = ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_GET(descr->control);
  taskEXIT_CRITICAL();
  if ( isDescrHwOwned ) {
    FreeRTOS_printf( ("Dropped packet: no free TX DMA descriptors\n") );
    vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
    return pdFALSE;
  }

  descr->read_address = (alt_u32)(pxNetworkBuffer->pucEthernetBuffer);
  descr->transfer_length = pxNetworkBuffer->xDataLength;
  descr->control           |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;

  // move TX sw descriptor address to the next pointer in the list
  netIfInfo.pTxSwDescr = (t_descr*)(descr->next_desc_ptr);

  /* Call the standard trace macro to log the send event. */
  iptraceNETWORK_INTERFACE_TRANSMIT();


  return pdTRUE;
}

static uint8_t ucBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ][ BUFFER_SIZE_ROUNDED_UP ];

// function called by the stack during initialization to link network buffers to stack buffer descriptor structures
void vNetworkInterfaceAllocateRAMToBuffers(NetworkBufferDescriptor_t pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] ) {
  BaseType_t x;

  for( x = 0; x < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; x++ ) {
    // pucEthernetBuffer is set to point ipBUFFER_PADDING bytes in from the
    // beginning of the allocated buffer
    pxNetworkBuffers[ x ].pucEthernetBuffer = &( ucBuffers[ x ][ ipBUFFER_PADDING ] );

    // The following line is also required, but will not be required in
    // future versions.
    *( ( uint32_t * ) &ucBuffers[ x ][ 0 ] ) = ( uint32_t ) &( pxNetworkBuffers[ x ] );
  }
}
BaseType_t xGetPhyLinkStatus( void )
{
  /* FIX ME. */
  return pdFALSE;
}
