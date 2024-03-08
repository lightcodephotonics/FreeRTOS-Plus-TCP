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

#define TSE_RESET_TIMEOUT_MS 1000

#define RX_DEFERRED_ISR_STACKSIZE  ( 8 * configMINIMAL_STACK_SIZE )
#define RX_DEFERRED_ISR_PRIORITY  configMAX_PRIORITIES - 1
#define RX_LINKED_RX_MESSAGES_NUM 1

#define MSGDMA_PREFETCHER_STANDARD_DESCRIPTOR_SIZE 0x20 // 256 bit = 32 bytes
#define MSGDMA_DESCRIPTOR_SIZE MSGDMA_PREFETCHER_STANDARD_DESCRIPTOR_SIZE // using standard descriptors

#define RX_DESCRIPTOR_LIST_SIZE 5
#define TX_DESCRIPTOR_LIST_SIZE RX_DESCRIPTOR_LIST_SIZE
#define RX_DESCRIPTOR_LIST_START CPU_SUBSYSTEM_TSE_DMA_DESC_RAM_BASE
#define TX_DESCRIPTOR_LIST_START (RX_DESCRIPTOR_LIST_START + RX_DESCRIPTOR_LIST_SIZE * MSGDMA_DESCRIPTOR_SIZE)

typedef  alt_msgdma_prefetcher_standard_descriptor t_descr;

alt_tse_system_info tse_mac_device[MAXNETS] = {
    TSE_SYSTEM_EXT_MEM_NO_SHARED_FIFO(
        CPU_SUBSYSTEM_TSE,             // tse_name
        0,                             // offset
        CPU_SUBSYSTEM_TSE_TX_MSGDMA,   // msgdma_tx_name
        CPU_SUBSYSTEM_TSE_RX_MSGDMA,   // msgdma_rx_name
        TSE_PHY_AUTO_ADDRESS,          // phy_addr
        NULL,                          // phy_cfg_fp
        CPU_SUBSYSTEM_TSE_DMA_DESC_RAM // desc_mem_name
    )
};

// Texas Instruments DP83867 phy
void TIWR_extended(np_tse_mac *pmac, alt_u16 reg, alt_u16 val) {
    IOWR(&pmac->mdio1.regd, 0, 0x001f); // prep address write
    IOWR(&pmac->mdio1.rege, 0, reg);    // write reg address
    IOWR(&pmac->mdio1.regd, 0, 0x401f); // prep data write
    IOWR(&pmac->mdio1.rege, 0, val);    // write reg data
}
alt_u16 TIRD_extended(np_tse_mac *pmac, alt_u16 reg) {
    IOWR(&pmac->mdio1.regd, 0, 0x001f); // prep address write
    IOWR(&pmac->mdio1.rege, 0, reg);    // write reg address
    IOWR(&pmac->mdio1.regd, 0, 0x401f); // prep data read
    return IORD(&pmac->mdio1.rege, 0);  // read reg data
}

void reset_DP83867(np_tse_mac *pmac) {
    printf("reset_DP83867\n");
    IOWR(&pmac->mdio1.CONTROL, 0, 1<<15);
    do {
        vTaskDelay(10);
        printf("waiting reset to finish\n");
    } while(IORD(&pmac->mdio1.CONTROL, 0) & (1<<15));
}

// void print_all_DP83867(np_tse_mac *pmac) {
//     for (int i = 0; i < 32; i++) {
//         printf("0x%04x = 0x%04x\n", i, IORD(&pmac->mdio1.CONTROL, i));
//     }
//     printf("0x0025 = 0x%04x\n", TIRD_extended(pmac, 0x25));
//     printf("0x002c = 0x%04x\n", TIRD_extended(pmac, 0x2c));
//     printf("0x002d = 0x%04x\n", TIRD_extended(pmac, 0x2d));
//     printf("0x002e = 0x%04x\n", TIRD_extended(pmac, 0x2e));
//     printf("0x0031 = 0x%04x\n", TIRD_extended(pmac, 0x31));
//     printf("0x0032 = 0x%04x\n", TIRD_extended(pmac, 0x32));
//     printf("0x0033 = 0x%04x\n", TIRD_extended(pmac, 0x33));
//     printf("0x0043 = 0x%04x\n", TIRD_extended(pmac, 0x43));
//     printf("0x0053 = 0x%04x\n", TIRD_extended(pmac, 0x53));
//     printf("0x0055 = 0x%04x\n", TIRD_extended(pmac, 0x55));
//     printf("0x006e = 0x%04x\n", TIRD_extended(pmac, 0x6e));
//     printf("0x006f = 0x%04x\n", TIRD_extended(pmac, 0x6f));
//     printf("0x0071 = 0x%04x\n", TIRD_extended(pmac, 0x71));
//     printf("0x0072 = 0x%04x\n", TIRD_extended(pmac, 0x72));
//     printf("0x007b = 0x%04x\n", TIRD_extended(pmac, 0x7b));
//     printf("0x007c = 0x%04x\n", TIRD_extended(pmac, 0x7c));
//     printf("0x008a = 0x%04x\n", TIRD_extended(pmac, 0x8a));
//     printf("0x0086 = 0x%04x\n", TIRD_extended(pmac, 0x86));
//     printf("0x00B3 = 0x%04x\n", TIRD_extended(pmac, 0xB3));
//     printf("0x00C0 = 0x%04x\n", TIRD_extended(pmac, 0xC0));
//     printf("0x00C6 = 0x%04x\n", TIRD_extended(pmac, 0xC6));
//     printf("0x00e9 = 0x%04x\n", TIRD_extended(pmac, 0xe9));
//     printf("0x00fe = 0x%04x\n", TIRD_extended(pmac, 0xfe));
//     printf("0x0100 = 0x%04x\n", TIRD_extended(pmac, 0x100));
//     printf("0x012c = 0x%04x\n", TIRD_extended(pmac, 0x12c));
//     printf("0x0134 = 0x%04x\n", TIRD_extended(pmac, 0x134));
//     printf("0x0135 = 0x%04x\n", TIRD_extended(pmac, 0x135));
//     printf("0x0161 = 0x%04x\n", TIRD_extended(pmac, 0x161));
//     printf("0x0170 = 0x%04x\n", TIRD_extended(pmac, 0x170));
//     printf("0x0171 = 0x%04x\n", TIRD_extended(pmac, 0x171));
//     printf("0x0172 = 0x%04x\n", TIRD_extended(pmac, 0x172));
//     printf("0x0180 = 0x%04x\n", TIRD_extended(pmac, 0x180));
//     printf("0x01a4 = 0x%04x\n", TIRD_extended(pmac, 0x1a4));
// }

alt_32 DP83867_config(np_tse_mac *pmac) {
    /* Strap modes 1 and 2 are not applicable for RX_CTRL. The RX_CTRL strap must be configured for strap mode 3 or strap mode 4. If
    the RX_CTRL pin cannot be strapped to mode 3 or mode 4, bit[7] of Configuration Register 4 (address 0x0031) must be cleared to 0.
    Autoneg Disable should always be set to 0 when using gigabit Ethernet. */
    alt_u16 reg = TIRD_extended(pmac, 0x31);
    reg &= 0xFF7F;
    TIWR_extended(pmac, 0x31, reg);
    reset_DP83867(pmac);
    return 0;
}

alt_u32 DP83867_link_status_read(np_tse_mac *pmac) {
    alt_u32 speed;
    alt_u32 full_duplex;
    alt_u32 autoneg_done;
    alt_u32 link_is_up;
    alt_u32 sleep_mode;

    while (1) {
        alt_u32 reg = IORD(&pmac->mdio1.reg11, 0);
        speed = reg >> 14;
        full_duplex = (reg & (1<<13)) != 0;
        autoneg_done = (reg & (1<<11)) != 0;
        link_is_up = (reg & (1<<10)) != 0;
        sleep_mode = (reg & (1<<6)) != 0;
        if (sleep_mode) printf("DP83867 sleeps\n");
        if (!link_is_up) printf("DP83867 link is down\n");
        if (autoneg_done) {
            break;
        } else {
            printf("DP83867 autoneg ongoing\n");
            vTaskDelay(10);
            continue;
        }
    }

    while(IORD(&pmac->mdio1.rega, 0) & (1<<12) == 0) {
        printf("DP83867 Remote receiver is not OK\n");
        vTaskDelay(10);
    }

    // print_all_DP83867(pmac);

    return ((speed == 2 ? 0b001
           : speed == 1 ? 0b010
           : speed == 0 ? 0b100 : 0b000) << 1)
          | full_duplex;
}

alt_tse_phy_profile TI_DP83867 = {"TI DP83867",
    0x080028,                   // OUI
    0x23,                       // Vender Model Number
    0x1,                        // Model Revision Number
    0x11,                       // Location of Status Register (ignored)
    14,                         // Location of Speed Status    (ignored)
    13,                         // Location of Duplex Status   (ignored)
    10,                         // Location of Link Status     (ignored)
    &DP83867_config,            // No function pointer configure
    &DP83867_link_status_read   // Function pointer to read from PHY specific status register
};

typedef struct NET_IF_INFO {
    alt_tse_system_info *pTseSystemInfo;
    np_tse_mac *pTseCsr;
    alt_msgdma_dev *rxMsgdma;
    alt_msgdma_dev *txMsgdma;
    t_descr *pRxDescrListStart;
    t_descr *pTxDescrListStart;
    TaskHandle_t *pDeferredISRHandle;
    t_descr *pRxHwDescr;
    t_descr *pRxSwDescr;
    uint8_t u8bRxHwDescrNum;
    t_descr *pTxSwDescr;
} NET_IF_INFO;

static NET_IF_INFO netIfInfo;

/* The deferred interrupt handler is a standard RTOS task.  FreeRTOS's centralised
   deferred interrupt handling capabilities can also be used. */
static void prvEMACDeferredInterruptHandlerTask( void *pvParameters ) {
    NetworkBufferDescriptor_t *pxDescriptor;
    NetworkBufferDescriptor_t *pxHead = NULL;
    NetworkBufferDescriptor_t *pxTail = NULL;
    uint8_t *pucTemp;
    t_descr *descr;

    /* Used to indicate that xSendEventStructToIPTask() is being called because
    of an Ethernet receive event. */
    IPStackEvent_t xRxEvent;
    uint32_t ulNotifiedValue;

    NET_IF_INFO *pNetIfInfo = (NET_IF_INFO *) pvParameters;
    descr = pNetIfInfo->pRxDescrListStart;

    for (int j = 0;; ++j) {
        /* Wait for the Ethernet MAC interrupt to indicate that another packet
        has been received.  The task notification is used in a similar way to a
        counting semaphore to count Rx events, but is a lot more efficient than
        a semaphore. */
        ulNotifiedValue = ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

        /* Allocate a new network buffer descriptor that references an Ethernet
        frame large enough to hold the maximum network packet size (as defined
        in the FreeRTOSIPConfig.h header file). */
        pxDescriptor = pxGetNetworkBufferWithDescriptor( ipTOTAL_ETHERNET_FRAME_SIZE, 0 );
        alt_dcache_flush(pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength);
        alt_dcache_flush(descr->write_address, ipTOTAL_ETHERNET_FRAME_SIZE);

        /* Copy the pointer to the newly allocated Ethernet frame to a temporary
        variable. */
        pucTemp = pxDescriptor->pucEthernetBuffer;

        /* Update the newly allocated network buffer descriptor
        to point to the Ethernet buffer that contains the received data. */
        pxDescriptor->pucEthernetBuffer = (uint8_t*)descr->write_address;
        pxDescriptor->xDataLength = descr->bytes_transfered;

        /*
         * The network buffer descriptor now points to the Ethernet buffer that
         * contains the received data, and the Ethernet DMA descriptor now points
         * to a newly allocated (and empty) Ethernet buffer ready to receive more
         * data.  No data was copied.  Only pointers to data were swapped.
         */
        *( ( NetworkBufferDescriptor_t ** )
           ( pxDescriptor->pucEthernetBuffer - ipBUFFER_PADDING ) ) = pxDescriptor;

        /* Update the Ethernet Rx DMA descriptor to point to the newly allocated
           Ethernet buffer. */
        descr->write_address = (alt_u32) pucTemp;

        taskENTER_CRITICAL();
        pNetIfInfo->pRxSwDescr = (t_descr*)(descr->next_desc_ptr);
        // Give descriptor to HW
        if ((t_descr*)(pNetIfInfo->pRxHwDescr->next_desc_ptr) == descr) {
            pNetIfInfo->pRxHwDescr->control |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;
        }
        descr = (t_descr*)(descr->next_desc_ptr);
        taskEXIT_CRITICAL();

        if( pxHead == NULL ) {
            /* Remember the first packet. */
            pxHead = pxDescriptor;
        }
        if( pxTail != NULL ) {
            /* Make the link */
            pxTail->pxNextBuffer = pxDescriptor;
        }
        /* Remember the last packet. */
        pxTail = pxDescriptor;

        if ((j == RX_LINKED_RX_MESSAGES_NUM) || (pNetIfInfo->pRxHwDescr == pNetIfInfo->pRxSwDescr)) {
            j = 0;
            pxTail->pxNextBuffer = NULL;

            /* See if the data contained in the received Ethernet frame needs
            to be processed.  NOTE! It might be possible to do this in
            the interrupt service routine itself, which would remove the need
            to unblock this task for packets that don't need processing. */
            //if( eConsiderFrameForProcessing( pxDescriptor->pucEthernetBuffer ) == eProcessBuffer ) {
            /* The event about to be sent to the TCP/IP is an Rx event. */
            xRxEvent.eEventType = eNetworkRxEvent;

            /* pvData is used to point to the network buffer descriptor that
              references the received data. */
            xRxEvent.pvData = ( void * ) pxHead;

            /* Send the data to the TCP/IP stack. */
            if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE ) {
                /* The buffer could not be sent to the IP task so the buffer
                must be released. */
                //vReleaseNetworkBufferAndDescriptor( pxDescriptor );
                for(;pxHead != NULL;) {
                    pxTail = pxHead;
                    pxHead = pxHead->pxNextBuffer;
                    vReleaseNetworkBufferAndDescriptor( pxTail );
                }
                /* Make a call to the standard trace macro to log the
                  occurrence. */
                iptraceETHERNET_RX_EVENT_LOST();
            } else {
                /* The message was successfully sent to the TCP/IP stack.
                Call the standard trace macro to log the occurrence. */
                iptraceNETWORK_INTERFACE_RECEIVE();
            }
            pxHead = NULL;
            pxTail = NULL;
        }
    }
}

static void inline rx_msgdma_isr(NET_IF_INFO *pNetIfInfo) {
    BaseType_t xHigherPriorityTaskWoken;
    // Re_enable global interrupts so we don't miss one that occurs during the
    // processing of this ISR.
    alt_msgdma_dev *rx_msgdma = pNetIfInfo->rxMsgdma;
    alt_u32        reg_data_32;
    //reg_data_32  = IORD_ALT_MSGDMA_PREFETCHER_CONTROL(rx_msgdma->csr_base);
    reg_data_32  = IORD_ALT_MSGDMA_PREFETCHER_CONTROL(rx_msgdma->prefetcher_base);
    reg_data_32 |= ALT_MSGDMA_PREFETCHER_CTRL_GLOBAL_INTR_EN_SET_MASK;
    //IOWR_ALT_MSGDMA_PREFETCHER_CONTROL(rx_msgdma->csr_base, reg_data_32);
    IOWR_ALT_MSGDMA_PREFETCHER_CONTROL(rx_msgdma->prefetcher_base, reg_data_32);
    IOWR_ALT_MSGDMA_PREFETCHER_STATUS(rx_msgdma->prefetcher_base, ALT_MSGDMA_PREFETCHER_STATUS_IRQ_CLR_MASK);

    // move HW descriptor pointer to the next descriptor
    pNetIfInfo->pRxHwDescr = (t_descr*)(pNetIfInfo->pRxHwDescr->next_desc_ptr);

    // check if HW have already gone through all the descriptors in the list
    if ((t_descr*)(pNetIfInfo->pRxHwDescr->next_desc_ptr) != pNetIfInfo->pRxSwDescr) {
        // give next descriptor to HW
        pNetIfInfo->pRxHwDescr->control |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;
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
    vTaskNotifyGiveFromISR( *(pNetIfInfo->pDeferredISRHandle), &xHigherPriorityTaskWoken );
    /* Force a context switch if xHigherPriorityTaskWoken is now
    set to pdTRUE. The macro used to do this is dependent on
    the port and may be called portEND_SWITCHING_ISR. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    //portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

static void inline tx_msgdma_isr(NET_IF_INFO *pNetIfInfo) {
    alt_u32 reg_data;
    reg_data = IORD_ALTERA_MSGDMA_CSR_STATUS(pNetIfInfo->txMsgdma->csr_base);
    if ((reg_data & ALTERA_MSGDMA_CSR_STOPPED_ON_ERROR_MASK) ||
        (reg_data & ALTERA_MSGDMA_CSR_STOPPED_ON_EARLY_TERMINATION_MASK)) {
        printf("Tx mSGDMA descriptor is in error.\n");
        return;
    }
}

extern int alt_msgdma_start_prefetcher_with_list_addr (
    alt_msgdma_dev *dev,
    alt_u64  list_addr,
    alt_u8 park_mode_en,
    alt_u8 poll_en);

BaseType_t xNetworkInterfaceInitialise( void ) {
    // speed and duplex returned by getPHYSpeed
    uint32_t duplex;
    uint32_t speed;
    // temp
    uint32_t result;
    uint32_t reg_data;
    uint32_t count;

    // Create shared struct
    netIfInfo.pTseSystemInfo = &(tse_mac_device[0]);
    netIfInfo.pTseCsr = (np_tse_mac*)(netIfInfo.pTseSystemInfo->tse_mac_base);

    // Add PHY profiles
    alt_tse_phy_add_profile(&TI_DP83867);

    // Initialize PHY and PCS by calling getPHYSpeed()
    // uC-TCP-IP does that
    while (1) {
        result = getPHYSpeed(netIfInfo.pTseCsr);
        // setup_phy_loopback(netIfInfo.pTseCsr);
        if ((result >> 16) & 0xFF) {
            if (result & ALT_TSE_E_INVALID_SPEED)
                printf("getPHYSpeed: Invalid speed read from PHY.\n");
            if (result & ALT_TSE_E_PROFILE_INCORRECT_DEFINED)
                printf("getPHYSpeed: PHY Profile is not defined correctly.\n");
            if (result & ALT_TSE_E_NO_PHY_PROFILE)
                printf("getPHYSpeed: No PHY profile matched the detected PHY.\n");
            if (result & ALT_TSE_E_AN_NOT_COMPLETE)
                printf("getPHYSpeed: Auto-negotiation not completed.\n");
            if (result & ALT_TSE_E_NO_COMMON_SPEED)
                printf("getPHYSpeed: No common speed found for multi-port MAC.\n");
            if (result & ALT_TSE_E_NO_PHY)
                printf("getPHYSpeed: No PHY detected.\n");
            if (result & ALT_TSE_E_NO_MDIO)
                printf("getPHYSpeed: No MDIO used by the MAC.\n");
            if (result & ALT_TSE_E_NO_PMAC_FOUND)
                printf("getPHYSpeed: Argument *pmac not found from the list of MAC detected during init.\n");
            continue;
        }
        uint32_t *pcs_ptr = (0xc0002000 + 0x80*4); //&pmac->mdio0.CONTROL
        uint16_t pcs_status = IORD(pcs_ptr, 1);
        uint16_t dev_ability = IORD(pcs_ptr, 4);
        uint16_t partner_ability = IORD(pcs_ptr, 5);
        uint16_t an_expansion = IORD(pcs_ptr, 6);
        uint16_t if_mode = IORD(pcs_ptr, 0x14);
        uint16_t link_ok = (pcs_status&(1<<2))!=0;
        uint16_t autoneg_ok = (pcs_status&(1<<5))!=0;
        if (!link_ok || !autoneg_ok) {
            printf("PCS link_ok=%d or autoneg_ok=%d (status=0x%x dev_ability=0x%x partner_ability=0x%x an_expansion=0x%x if_mode=0x%x)\n",
                link_ok, autoneg_ok, pcs_status, dev_ability, partner_ability, an_expansion, if_mode);
            // uint16_t pcs_control = IORD(pcs_ptr, 0);
            // printf("PCS control (rst) 0x%x -> ", pcs_control);
            // pcs_control |= 1<<15;
            // IOWR(pcs_ptr, 0, pcs_control);
            // printf("0x%x\n", pcs_control);
            // vTaskDelay(3000);
            // continue;
        }

        speed  = (result >> 1) & 0x07;
        duplex = result & 0x01;
        printf("PHY speed %s Mbps %s-duplex\n",
            speed == 1 ? "1000" :
            speed == 2 ? "100" :
            speed == 4 ? "10" : "Unknown",
            duplex == 1 ? "Full" : "Half");
        printf("PHY and PCS Initialization: Success.\n");
        break;
    }

    // TSE configuration
    alt_u32 *tse_base = (alt_u32*)netIfInfo.pTseCsr;
    alt_tse_system_info *tse_sys_info = netIfInfo.pTseSystemInfo;
    {
        // Reset and disable TX and RX in TSE
        reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
        reg_data |= (ALTERA_TSEMAC_CMD_SW_RESET_MSK | ALTERA_TSEMAC_CMD_CNT_RESET_MSK);
        reg_data &= (~ALTERA_TSEMAC_CMD_RX_ENA_MSK);
        reg_data &= (~ALTERA_TSEMAC_CMD_TX_ENA_MSK);
        IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
        // wait for reset with timeout
        count = 0;
        while (1) {
            vTaskDelay(10);
            count += 10;
            // read status
            reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
            // if reset finished -> break
            if ((reg_data & ALTERA_TSEMAC_CMD_SW_RESET_MSK) == 0) {
                break;
            }
            // Give up if reset takes too long.
            if (count > TSE_RESET_TIMEOUT_MS) {
                printf("MAC reset: FAILED, Timed out!\n");
                return pdFALSE;
            }
        }
        printf("MAC Software Reset: Success.\n");
    }

    {
        printf("MAC FIFO Configuration: INFO: No shared fifo.\n");
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
        printf("MAC FIFO Configuration: Success.\n");
  }

    // setting mac
    // read MAC address provbided by the stack
    const uint8_t *hw_addr = FreeRTOS_GetMACAddress();
    {
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
        printf("MAC Address Configuration: Success.\n");
    }

    //MAC Configuration
    {

        // Set maximum frame length.
        IOWR_ALTERA_TSEMAC_FRM_LENGTH(tse_base, ALTERA_TSE_MAC_MAX_FRAME_LENGTH);
        // Maximum inter packet gap is already set by default (to 12).
        // Set maximum pause quanta value for flow control.
        IOWR_ALTERA_TSEMAC_PAUSE_QUANT(tse_base, 0xFFFF); // 0x0f said in https://www.intel.com/content/www/us/en/support/programmable/articles/000076478.html
        // TSE mac direct register configuration.
        {
            reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
            // Remove frames with CRC errors.
            reg_data |= ALTERA_TSEMAC_CMD_RX_ERR_DISC_MSK;;
            // TX MAC address insertion on transmit packet.
            reg_data |= ALTERA_TSEMAC_CMD_TX_ADDR_INS_MSK;
            IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
        }

        /* Setting tx_cmd_stat rx_cmd_stat registers */
        {
            // Tx boundary alignment.
            reg_data  = IORD_ALTERA_TSEMAC_TX_CMD_STAT(tse_base);
            reg_data &= ~ALTERA_TSEMAC_TX_CMD_STAT_TXSHIFT16_MSK;
            IOWR_ALTERA_TSEMAC_TX_CMD_STAT(tse_base, reg_data);
            // Ensure Tx boundary alignment disabled.
            reg_data = IORD_ALTERA_TSEMAC_TX_CMD_STAT(tse_base);
            if (reg_data & ALTERA_TSEMAC_TX_CMD_STAT_TXSHIFT16_MSK) {
                printf("TX header should NOT be boundary aligned: FAILED.\n");
                return pdFALSE;
            }
            // Rx boundary alignment.
            reg_data  = IORD_ALTERA_TSEMAC_RX_CMD_STAT(tse_base);
            reg_data &= ~ALTERA_TSEMAC_RX_CMD_STAT_RXSHIFT16_MSK;
            IOWR_ALTERA_TSEMAC_RX_CMD_STAT(tse_base, reg_data);
            // Ensure Tx boundary alignment disabled.
            reg_data = IORD_ALTERA_TSEMAC_RX_CMD_STAT(tse_base);
            if (reg_data & ALTERA_TSEMAC_RX_CMD_STAT_RXSHIFT16_MSK) {
                printf("RX header should NOT be boundary aligned: FAILED.\n");
                return pdFALSE;
            }
            printf("Turning off Tx/Rx header boundary 32-bit alignment: Success.\n");
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

        printf("MAC Configuration: Success.\n");
    }
    printf("MAC Initialization: Success.\n");

    // get pointers to mSGDMA modules
    alt_msgdma_dev *rx_msgdma = alt_msgdma_open(CPU_SUBSYSTEM_TSE_RX_MSGDMA_CSR_NAME);
    if (rx_msgdma == NULL || ALT_ERRNO == ENODEV) {
        printf("Rx mSGDMA Open '%s': FAILED.\n", CPU_SUBSYSTEM_TSE_RX_MSGDMA_CSR_NAME);
        return pdFALSE;
    }
    netIfInfo.rxMsgdma = rx_msgdma;

    alt_msgdma_dev *tx_msgdma = alt_msgdma_open(CPU_SUBSYSTEM_TSE_TX_MSGDMA_CSR_NAME);
    if (tx_msgdma == NULL || ALT_ERRNO == ENODEV) {
        printf("Tx mSGDMA Open '%s': FAILED.\n", CPU_SUBSYSTEM_TSE_TX_MSGDMA_CSR_NAME);
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

    // Create Deferred ISR task
    TaskHandle_t *xHandlingTask = pvPortMalloc(sizeof(TaskHandle_t));
    xTaskCreate(prvEMACDeferredInterruptHandlerTask, "Deffered_RX_handler",
        RX_DEFERRED_ISR_STACKSIZE, &netIfInfo,
        RX_DEFERRED_ISR_PRIORITY, xHandlingTask );
    netIfInfo.pDeferredISRHandle = xHandlingTask;

    // Register mSGDMA ISR.
    {
        alt_msgdma_register_callback(
            rx_msgdma,                           // mSGDMA device
            (alt_msgdma_callback) rx_msgdma_isr, // callback function
            0,                                   // control
            &netIfInfo);                         // arguments/context
        printf("mSGDMA RX ISR Registering: Success.\n");

        alt_msgdma_register_callback(
            tx_msgdma,                           // mSGDMA device
            (alt_msgdma_callback) tx_msgdma_isr, // callback function
            0,                                   // control
            &netIfInfo);                         // arguments/context
        printf("mSGDMA TX ISR Registering: Success.\n");
    }
    // Initialize mSGDMA descriptors (and descriptor lists).
    {
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
            size_t bufferSize = ipTOTAL_ETHERNET_FRAME_SIZE;
            //NetworkBufferDescriptor_t *pxDescriptor = pxGetNetworkBufferWithDescriptor( ipTOTAL_ETHERNET_FRAME_SIZE, 0 );
            buf_addr = pucGetNetworkBuffer(&bufferSize);
            //buf_addr = pxDescriptor->pucEthernetBuffer;
            alt_dcache_flush(buf_addr, bufferSize);
            result = alt_msgdma_construct_prefetcher_standard_st_to_mm_descriptor(
                rx_msgdma, descr, (alt_u32) buf_addr, ipTOTAL_ETHERNET_FRAME_SIZE, reg_data);
            if (result != 0) {
                printf("RX mSGDMA initializing descriptor %d: FAILED.\n", i);
                return pdFALSE;
            }
            result = alt_msgdma_prefetcher_add_standard_desc_to_list(&llist, descr);
            if (result != 0) {
                printf("RX mSGDMA add descriptor %d to list: FAILED.\n", i);
                return pdFALSE;
            }
        }
        // Give the first RX descriptor to HW (set hw_owned bit to 1)
        rx_descr_list->control |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;
        // Start prefetcher with the list pointing on the first descriptor
        result = alt_msgdma_start_prefetcher_with_list_addr(rx_msgdma, (alt_u32)rx_descr_list, 0, 1);
        if (result != 0) {
            printf("mSGDMA Initialization: Failed to start Rx list.\n");
            return pdFALSE;
        }
        IOWR_ALT_MSGDMA_PREFETCHER_DESCRIPTOR_POLLING_FREQ(rx_msgdma->prefetcher_base, 0xA);

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
                printf("TX mSGDMA initializing descriptor %d: FAILED.\n", i);
                return pdFALSE;
            }
            result = alt_msgdma_prefetcher_add_standard_desc_to_list(&llist, descr);
            if (result != 0) {
                printf("TX mSGDMA add descriptor %d to list: FAILED.\n", i);
                return pdFALSE;
            }
        }
        // Start TX prefetcher waiting (polling) for first TX packet
        result = alt_msgdma_start_prefetcher_with_list_addr(tx_msgdma, (alt_u32)tx_descr_list, 0, 1);
        if (result != 0) {
            printf("mSGDMA Initialization: Failed to start Tx list.\n");
            return pdFALSE;
        }
        IOWR_ALT_MSGDMA_PREFETCHER_DESCRIPTOR_POLLING_FREQ(tx_msgdma->prefetcher_base, 0xA);
        printf("mSGDMA descriptor list initialization: Success.\n");
    }

    // Enable MAC Transmit and Receive
    {
        // Write RX and TX enable bits.
        reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
        reg_data |= ALTERA_TSEMAC_CMD_RX_ENA_MSK;
        reg_data |= ALTERA_TSEMAC_CMD_TX_ENA_MSK;
        // reg_data |= ALTERA_TSEMAC_CMD_LOOPBACK_MSK;
        IOWR_ALTERA_TSEMAC_CMD_CONFIG(tse_base, reg_data);
        // Ensure Rx is enabled.
        reg_data = IORD_ALTERA_TSEMAC_CMD_CONFIG(tse_base);
        if ((reg_data & ALTERA_TSEMAC_CMD_RX_ENA_MSK) == 0) {
            printf("Enable MAC Receive: FAILED.\n");
            return pdFALSE;
        }
        // Ensure Tx is enabled.
        if ((reg_data & ALTERA_TSEMAC_CMD_TX_ENA_MSK) == 0) {
            printf("Enable MAC TX: FAILED.\n");
            return pdFALSE;
        }
        // Ensure Rx is enabled.
        if ((reg_data & ALTERA_TSEMAC_CMD_RX_ENA_MSK) == 0) {
            printf("Enable MAC RX: FAILED.\n");
            return pdFALSE;
        }
        // printf("MAC loopback 0x%x\n", reg_data & ALTERA_TSEMAC_CMD_LOOPBACK_MSK);
        printf("Enable MAC Transmit and Receive: Success.\n");
        printf("[NetworkInterface] Finish\n");
    }

    return pdTRUE;
}

BaseType_t xNetworkInterfaceOutput(
    NetworkBufferDescriptor_t * const pxNetworkBuffer,
    BaseType_t xReleaseAfterSend
) {
    // get current TX software descriptor address
    t_descr *descr = netIfInfo.pTxSwDescr;
    alt_dcache_flush(pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength);

    // check if descriptor is still HW owned
    taskENTER_CRITICAL();
    uint8_t isDescrHwOwned = ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_GET(descr->control);
    taskEXIT_CRITICAL();
    if ( isDescrHwOwned ) {
        printf("Dropped packet: no free TX DMA descriptors\n");
        vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
        return pdFALSE;
    }

    // if descriptor is free
    /* If BufferAllocation_2.c is being used then the DMA descriptor may
    still be pointing to the buffer it last transmitted.  If this is the case
    then the old buffer must be released (returned to the TCP/IP stack) before
    descriptor is updated to point to the new data waiting to be transmitted. */
    if( (uint8_t *)(descr->read_address) != NULL ) {
        /* Note this is releasing just an Ethernet buffer, not a network buffer
        descriptor as the descriptor has already been released. */
        vReleaseNetworkBuffer( (uint8_t *)(descr->read_address) );
    }

    /* Configure the DMA descriptor to send the data referenced by the network buffer
        descriptor */
    descr->read_address = (alt_u32)(pxNetworkBuffer->pucEthernetBuffer);
    descr->transfer_length = pxNetworkBuffer->xDataLength;
    descr->control |= ALT_MSGDMA_PREFETCHER_DESCRIPTOR_CTRL_OWN_BY_HW_SET_MASK;

    // move TX sw descriptor address to the next pointer in the list
    netIfInfo.pTxSwDescr = (t_descr*)(descr->next_desc_ptr);

    /* Call the standard trace macro to log the send event. */
    iptraceNETWORK_INTERFACE_TRANSMIT();

    /* The network buffer descriptor must now be returned to the TCP/IP stack, but
    the Ethernet buffer referenced by the network buffer descriptor is still in
    use by the DMA.  Remove the reference to the Ethernet buffer from the network
    buffer descriptor so releasing the network buffer descriptor does not result
    in the Ethernet buffer also being released.  xReleaseAfterSend() should never
    equal pdFALSE when  ipconfigZERO_COPY_TX_DRIVER is set to 1 (as it should be
    if data is transmitted using a zero copy driver.*/
    if( xReleaseAfterSend != pdFALSE ) {
        pxNetworkBuffer->pucEthernetBuffer = NULL;
        vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
    }

    return pdTRUE;
}

void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] ) {
    /* FIX ME. */
}

BaseType_t xGetPhyLinkStatus( void ) {
    /* FIX ME. */
    return pdFALSE;
}