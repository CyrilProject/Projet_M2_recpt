/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @brief Example project on UART usage to communicate with PC.
 * @defgroup uart_example UART example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief Example of basic UART usage.
 *
 * Simple UART example that transmits and receives through the configured pins as serial device.
 * The configure pins needs to be redirected to a COM port (for some terminal program like putty which
 * can listen to this COM port through a terminal session)
 * When the program start it will transmit "START: " through this serial device using @ref simple_uart_putstring
 * and this should be visible on the terminal.
 * All typed characters on this terminal will be transmitted to this program through @ref simple_uart_get and
 * when an exit character 'q' or 'Q' is typed this program will end into an infinite loop after transmitting 
 * "EXIT!" on the new line of the terminal.
 * @note This example is not just for COM ports but can be used for any UART connection, COM port redirection
 *       is for visual verification.
 * @note Setting the define ENABLE_LOOPBACK_TEST will assume that the TX_PIN_NUMBER is connected to RX_PIN_NUMBER
 *       and this example is used to test the loopback. In this case no com port can be used as the data flows
 *       from TX to RX and ERROR_PIN is set high for any loss of data.
 * @note Configure your terminal application for 38400 bauds, 8 data bits and 1 stop bit.
 *
 * @image html example_board_setup_a.png "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "simple_uart.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "radio_config.h"
//#define ENABLE_LOOPBACK_TEST           /*!< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback */

#define ERROR_PIN                (8)   /*!< gpio pin number to show error if loopback is enabled */
#define MAX_TEST_DATA_BYTES      (15U) /*!< max number of test bytes to be used for tx and rx */
#define MAX_PACKET_SIZE (50U)
#define CONVERSION_G 0.000244
static uint8_t volatile packet[MAX_PACKET_SIZE];  ///< Received packet buffer


/* Sends ' Exit!' string to UART.
Execution is blocked until UART peripheral detects all characters have been sent.
*/
//static __INLINE void uart_quit()
//{
//  simple_uart_putstring((const uint8_t *)" \n\rExit!\n\r");
//}

/** Sends 'Start: ' string to UART.
Execution is blocked until UART peripheral detects all characters have been sent.
 */
static __INLINE void uart_start()
{
  simple_uart_putstring((const uint8_t *)" \n\rStart: ");
}



/** Set @ref ERROR_PIN to one and enters an infinite loop. This function is called if any of the
 *  nRF6350 functions fail.
 */
//static void show_error(void)
//{
//  nrf_gpio_pin_write(ERROR_PIN, 1);
//  while(true)
//  {
//  }
//}


/** Transmits one char at a time as check if the loopback received data is same as transmitted
 *  Just used for testing with loopback setup (i.e, @ref TX_PIN_NUMBER connected to @ref RX_PIN_NUMBER)
 *  return true if test passed, else return false
 */
//static void uart_loopback_test()
//{
//  uint8_t *tx_data = (uint8_t *)("\n\rLOOPBACK_TEST");
//  uint8_t rx_data;
//
//  // Start sending one byte and see if you get the same
//  for(uint8_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
//  {
//    bool status;
//    simple_uart_put(tx_data[i]);
//    status = simple_uart_get_with_timeout(2, &rx_data);
//
//    if ((rx_data != tx_data[i]) || (!status))
//    {
//      show_error();
//    }
//  }
//  return; // Test passed
//}



/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  int16_t data1, data2, data3, data;
  simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);

  // ERROR_PIN configure as output
  nrf_gpio_cfg_output(ERROR_PIN);


//  uart_start();

    /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }
  
  simple_uart_putstring("\n\rStart :\n\r" );
//  // Set Port 1 as output
//  nrf_gpio_range_cfg_output(8, 15);

// Set radio configuration parameters
  radio_configure();

//  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x55);

  while(true)
  {
    // Set payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)packet;

    NRF_RADIO->EVENTS_READY = 0U;

    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;

    while(NRF_RADIO->EVENTS_READY == 0U)
    {
    }

    NRF_RADIO->EVENTS_END = 0U;

    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet
    while(NRF_RADIO->EVENTS_END == 0U)
    {
    }

    // Write received data to port 1 on CRC match
    if (NRF_RADIO->CRCSTATUS == 1U)
    {
//      nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, packet[2]);
//      uint8_t test[3];
//      char ok[3];
//      test[0]=packet[2];
//      test[1]=packet[3];
//      test[2]=packet[4];
//      simple_uart_putstring("ceci est un test: \n\r");
//      simple_uart_putstring((uint8_t *)itoa(100,ok,10));
//      simple_uart_putstring((uint8_t *)"\n\r" );
      data1 = (int16_t)packet[2];
      data2 = (int16_t)packet[3]<< 8;
//      data3 = packet[4]<< 16;
      data = data1+data2;
      simple_uart_putstring("X = " );
      itoa(data*CONVERSION_G,3);
      simple_uart_putstring(" , " );
      data1 = (int16_t)packet[4];
      data2 = (int16_t)packet[5]<< 8;
//      data3 = packet[7]<< 16;
      data = data1+data2;
      simple_uart_putstring("Y = " );
      itoa(data*CONVERSION_G,3);
      simple_uart_putstring(" , " );
      data1 = (int16_t)packet[6];
      data2 = (int16_t)packet[7]<< 8;
//      data3 = packet[10]<< 16;
      data = data1+data2;
      simple_uart_putstring("Z = " );
      itoa(data*CONVERSION_G,3);
      simple_uart_putstring("      \r" );
//      simple_uart_putstring("\033[2J" );
      
    }

    NRF_RADIO->EVENTS_DISABLED = 0U;

    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while(NRF_RADIO->EVENTS_DISABLED == 0U)
    {
    }
  }
  
}

//    uint8_t cr = simple_uart_get();
//    simple_uart_put(cr);
//
//    if(cr == 'q' || cr == 'Q')
//    {
//      uart_quit();
//      while(1){}
//    }
//  }
//
//#else
//  /* This part of the example is just for testing, can be removed if you do not have a loopback setup */
//
//  while(true)
//  {
//    uart_loopback_test();
//  }


  
