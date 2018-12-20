/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/ADC.h>

/* Example/Board Header files */
#include "COMMS_Board.h"

#include "satellite.h"
#include "devices.h"

#include "INA226.h"
#include "TMP100.h"

#include "parameters.h"
#include "OSAL.h"

extern UART_Handle uart_dbg_bus;
extern UART_Handle uart_pq9_bus;

bool start_flag = false;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    I2C_init();
    SPI_init();
    Timer_init();
    ADC_init();
    Watchdog_init();

    /* Turn on user LED */
    GPIO_write(PQ9_EN, 0);

    /*ECSS services start*/
    pkt_pool_INIT();
    OSAL_init();
    device_init();
    init_parameters();

    //sleep(10);

    pq_rx_addr_cnt = 0;
    pq_rx_byte_cnt = 0;

    uint8_t test_packet[] = { 0x07, 0x04, 0x01, 0x00, 0x00, 0x11, 0x01, 0x64, 0x19 };
//    for(uint16_t temp_i = 0; temp_i < 1000; temp_i++) {
//        GPIO_write(PQ9_EN, 1);
//        UART_writePolling(uart_pq9_bus, test_packet, 9);
//        //UART_write(uart_pq9_bus, test_packet, 7);
//        GPIO_write(PQ9_EN, 0);
//        usleep(500000);
//
//        //usleep(1000);
//        sprintf(msg,"COMMS Received: %d, %d\n", pq_rx_addr_cnt, pq_rx_byte_cnt);
//        UART_write(uart_dbg_bus, msg, strlen(msg));
//    }
//
//    sprintf(msg,"Received: %d, %d\n", pq_rx_addr_cnt, pq_rx_byte_cnt);
//    UART_write(uart_dbg_bus, msg, strlen(msg));


    uint16_t size;
    uint8_t buf[4];

    start_flag = true;

    uint32_t sen_loop = 100000;

    /* Loop forever echoing */
    while (1) {

        set_parameter(SBSYS_reset_clr_int_wdg_param_id, NULL);

        update_device(COMMS_TEMP_DEV_ID);
        usleep(1);

        update_device(COMMS_ADC_DEV_ID);
        usleep(1);

        get_parameter(SBSYS_sensor_loop_param_id, &sen_loop, buf, &size);
        usleep(sen_loop);

        sprintf(msg,"COMMS Received: %d, %d\n", pq_rx_addr_cnt, pq_rx_byte_cnt);
        UART_write(uart_dbg_bus, msg, strlen(msg));

    }
}

/*  ======== ecssThread ========
 *  This thread runs on a higher priority, since wdg pin
 *  has to be ready for master.
 */
void *pqReceiveThread(void *arg0)
{

    while(!start_flag) {
        usleep(1000);
    }

    /* Loop forever */
    while (1) {
         PQ9_beta();
         usleep(1);
    }

    return (NULL);
}

void *pqTransmitThread(void *arg0)
{

    while(!start_flag) {
        usleep(1000);
    }

    /* Loop forever */
    while (1) {
         //export_pkt();
         sleep(100);
    }

    return (NULL);
}


char msg[100];

/*  ======== senThread ========
 *  This a dbg thread for outputing sensor readings
 */

#define HLDLC_START_FLAG        0x7E
#define HLDLC_CONTROL_FLAG      0x7D
#define HLDLC_STOP_FLAG         0x7C

void HLDLC_deframe(uint8_t *buf_in,
                              uint8_t *buf_out,
                              const uint16_t size_in,
                              uint16_t *size_out) {


    uint16_t cnt = 0;

    for(uint16_t i = 0; i < size_in; i++) {

        uint8_t c = buf_in[i];

        if(c == HLDLC_START_FLAG) {
            cnt = 0;
        } else if(c == HLDLC_STOP_FLAG) {
            *size_out = cnt;
            return ;
        } else if(c == HLDLC_CONTROL_FLAG) {
            i++;
            c = buf_in[i];

            if(c == 0x5E) {
              buf_out[cnt++] = 0x7E;
            } else if(c == 0x5D) {
              buf_out[cnt++] = 0x7D;
            } else if(c== 0x5C) {
              buf_out[cnt++] = 0x7C;
            } else {
              return ;
            }
        } else {
            buf_out[cnt++] = c;
        }

    }
    return ;
}

uint8_t tx_count, tx_size, tx_buf[255];

void *senThread(void *arg0)
{

    sprintf(msg, "Reset\n");
    UART_write(uart_dbg_bus, msg, strlen(msg));

    while(!start_flag) {
        sleep(1);
    }

    bool ctrl_flag, strt_flag = false;

    /* Loop forever */
    while (1) {

        uint8_t resp54[3];
        uint8_t res;

        usleep(10);

        res = UART_read(uart_dbg_bus, resp54, 1);
              if(res > 0) {
                if(resp54[0] == HLDLC_START_FLAG) {
                  strt_flag = true;

                } else if(resp54[0] == HLDLC_CONTROL_FLAG) {
                  ctrl_flag = true;
                } else if(strt_flag) {
                   strt_flag = false;
                   tx_buf[0] = resp54[0];
                   tx_count = 1;
                } else if(ctrl_flag) {
                   ctrl_flag = false;
                   if(resp54[0] == 0x5D) {
                     tx_buf[tx_count] = 0x7D;
                     tx_count++;
                   } else if(resp54[0] == 0x5E) {
                     tx_buf[tx_count] = 0x7E;
                     tx_count++;
                   }
                } else if(tx_count == 1) {
                  tx_buf[tx_count] = resp54[0];
                  tx_size = resp54[0] + 5;
                  tx_count++;
                } else if(tx_count > 0 && tx_count < tx_size - 1) {
                  tx_buf[tx_count] = resp54[0];
                  tx_count++;
                } else if(tx_count > 0 && tx_count == tx_size - 1) {
                  tx_buf[tx_count] = resp54[0];
                  tx_count++;

                  {
                    pq9_pkt *pkt;

                    pkt = get_pkt(tx_count);

                    bool res_unpack_PQ = unpack_PQ9_BUS(tx_buf,
                                                        tx_count,
                                                        pkt);
                    pkt->dest_id &= 0x7F;
                    if(res_unpack_PQ == true) {
                        queuePush(pkt, RF_POOL_ID);
                    } else {
                      free_pkt(pkt);
                    }
                  }
                }
              }

    }

    return (NULL);
}
