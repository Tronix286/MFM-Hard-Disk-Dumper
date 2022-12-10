/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include <tusb.h>
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/vreg.h"
#include "PwmIn.pio.h"
#include "sm_to_buf.pio.h"
#include "ymodem.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
#define abs(x) ({ __typeof__(x) _x = (x); _x >= 0 ? _x : -_x; })

// outputs
#define HEAD0 0
#define HEAD1 1
#define HEAD2 2
#define HEAD3 3
#define DIR   4
#define STEP  5

// inputs
#define SEEK      6
#define TRK0      7
#define INDEX     8
#define READY     9
#define W_FLT     10
#define SELECTED  11
#define MFM_DATA  12

//#define MAX_DELTAS (131072L-4700L)
#define MAX_DELTAS (131072L)

/*
 * Crc8_Dallas() - Polinom 0x31
 *
 */

//===8<==============Original message text===============
static const uint8_t Crc8Table_Dallas[256] = { 0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160,
        225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70,
        24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71,
        165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166,
        68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242,
        172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19,
        77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76,
        18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181,
        54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201,
        74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };

uint8_t RX_Buffer[10];
uint8_t TX_Buffer[40];
int RxCount =0;
int NbrOfDataToRead = 0;
int NbrOfDataToTransfer = 0;

char fname[13];
uint8_t deltas[MAX_DELTAS];

uint32_t direction = 0;
uint32_t cyl_num = 0;
uint32_t head_num = 0;
int timer_cnt = 0;
int led_flashes = 0;
struct repeating_timer timer;
uint offset0;
static uint8_t timer_errors=0;

int dma_chan;
// the pio instance
PIO pio;
// the state machine
uint sm;
// the array with the results
float pwm_reading[3];
// data about the PWM input measured in pio clock cycles
uint32_t pulsewidth, period;

uint32_t start_time_clocks = 0xF;

//------------------------------------------------------------------------------------------------|
// PIO 0 IRQ 0 Interrupt Service Routine
//------------------------------------------------------------------------------------------------|
void pio0_isr() {
  if(pio_interrupt_get(pio,0))
	{
    //timer_errors++;
		pio_interrupt_clear(pio, 0);
	}
  //irq_clear(PIO0_IRQ_0);
}

bool repeating_timer_callback(struct repeating_timer *t) {
    gpio_put(LED_PIN, gpio_get(LED_PIN)^1);
    timer_cnt++;
    if (timer_cnt > led_flashes)
    {
      timer_cnt = 0;
      cancel_repeating_timer(t);
    }
    return true;
}

void led_lights(int num)
{
  led_flashes = num;
  if (timer.alarm_id == 0)
    add_repeating_timer_ms(100, repeating_timer_callback, NULL, &timer);
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);

    // Can't measure clk_ref / xosc as it is the ref
}

void DumpHex(const void* data, size_t size) {
	char ascii[17];
	size_t i, j;
	ascii[16] = '\0';
	for (i = 0; i < size; ++i) {
		printf("%02X ", ((unsigned char*)data)[i]);
		if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char*)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i+1) % 8 == 0 || i+1 == size) {
			printf(" ");
			if ((i+1) % 16 == 0) {
				printf("|  %s \n", ascii);
			} else if (i+1 == size) {
				ascii[(i+1) % 16] = '\0';
				if ((i+1) % 16 <= 8) {
					printf(" ");
				}
				for (j = (i+1) % 16; j < 16; ++j) {
					printf("   ");
				}
				printf("|  %s \n", ascii);
			}
		}
	}
}

void CapInit(void)
{
    // pio 0 is used
    pio = pio0;
    // state machine 0
    uint sm0 = 1;

    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This should only be needed if you are pushing things up to
    // >16bits/clk here, i.e. if you need to saturate the bus completely.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // load the sm0 program into the pio memory
    offset0 = pio_add_program(pio, &sm_to_dma_to_buffer_program);
    // make a sm config
    pio_sm_config smc0 = sm_to_dma_to_buffer_program_get_default_config(offset0);
    // set the 'jmp' pin
    sm_config_set_jmp_pin(&smc0, MFM_DATA);
    // set the 'wait' pin (uses 'in' pins)
    sm_config_set_in_pins(&smc0, MFM_DATA);
    // set shift direction
    sm_config_set_in_shift(&smc0, false, false, 0);

    //sm_config_set_fifo_join(&smc0, PIO_FIFO_JOIN_RX);

    // init the pio sm0 with the config
    pio_sm_init(pio, sm0, offset0, &smc0);

    // Get a free DMA channel, panic() if there are none
    dma_chan = dma_claim_unused_channel(true);
}

void CapArm(void)
{
    // pio 0 is used
    pio = pio0;
    // state machine 0
    uint sm0 = 1;
    // disable the sm
    pio_sm_set_enabled(pio, sm0, false);
    // make sure the FIFOs are empty
    pio_sm_clear_fifos(pio, sm0);
    pio_sm_restart(pio, sm0);
    pio_sm_exec(pio, sm0, pio_encode_jmp(offset0));

    // make a default dma config
    dma_channel_config dma_conf = dma_channel_get_default_config(dma_chan);
    // transfer uint32_t
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);
    // a FIFO is read -> no increment of read pointer
    channel_config_set_read_increment(&dma_conf, false);
    // a buffer in memory is written to -> increment write pointer
    channel_config_set_write_increment(&dma_conf, true);
    // let the sm0 of pio determine the speed
    channel_config_set_dreq(&dma_conf, pio_get_dreq(pio, sm0, false));
    // configure the dma channel to read 100 uint32_t from sm0 to the buffer
    dma_channel_configure(dma_chan, &dma_conf,
                          deltas,         // Destinatinon pointer: the buffer in memory
                          &pio->rxf[sm0], // Source pointer: the output of sm0
                          MAX_DELTAS,            // Number of transfers
                          true            // Start immediately
    );
    //pio_sm_exec(pio, sm0, pio_encode_wait_gpio(false, INDEX));
    //pio_sm_exec(pio, sm0, pio_encode_wait_gpio(true, INDEX));
    
    timer_errors = 0;

    //send START_TIME_CLOCKS
    pio_sm_put(pio,sm0,start_time_clocks);
    // enable the sm
    pio_sm_set_enabled(pio, sm0, true);

}
// input = pin that receives the PWM pulses.
void PwmIn(uint input)
{
        // pio 0 is used
        pio = pio0;
        // state machine 0
        sm = 0;
        // configure the used pins
        pio_gpio_init(pio, input);
        // load the pio program into the pio memory
        uint offset = pio_add_program(pio, &PwmIn_program);
        // make a sm config
        pio_sm_config c = PwmIn_program_get_default_config(offset);
        // set the 'jmp' pin
        sm_config_set_jmp_pin(&c, input);
        // set the 'wait' pin (uses 'in' pins)
        sm_config_set_in_pins(&c, input);
        // set shift direction
        sm_config_set_in_shift(&c, false, false, 0);
        // init the pio sm with the config
        pio_sm_init(pio, sm, offset, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
}

    // read the period and pulsewidth
    void read(void)
    {
        // clear the FIFO: do a new measurement
        pio_sm_clear_fifos(pio, sm);
        // wait for the FIFO to contain two data items: pulsewidth and period
        while (pio_sm_get_rx_fifo_level(pio, sm) < 2)
            ;
        // read pulse width from the FIFO
        pulsewidth = pio_sm_get(pio, sm);
        // read period from the FIFO
        period = pio_sm_get(pio, sm) + pulsewidth;
        // the measurements are taken with 2 clock cycles per timer tick
        pulsewidth = 2 * pulsewidth;
        // calculate the period in clock cycles:
        period = 2 * period;
    }

   // read_period (in seconds)
    float read_period(void)
    {
        read();
        // one clock cycle is 1/125000000 seconds
        return (period * 0.000000004);
    }

    // read_pulsewidth (in seconds)
    float read_pulsewidth(void)
    {
        read();
        // one clock cycle is 1/125000000 seconds
        return (pulsewidth * 0.000000004);
    }

    // read_dutycycle (between 0 and 1)
    float read_dutycycle(void)
    {
        read();
        return ((float)pulsewidth / (float)period);
    }

    // read pulsewidth and period for one pulse
    void read_PWM(float *readings)
    {
        read();
        *(readings + 0) = (float)pulsewidth * 0.000000004;
        *(readings + 1) = (float)period * 0.000000004;
        *(readings + 2) = ((float)pulsewidth / (float)period);
    }

uint8_t Crc8_Dallas(uint8_t * _pBuff, uint32_t _len)
{
    uint8_t Crc8 = 0;

    for (uint32_t i = 0; i < _len; i++) {
        Crc8 = Crc8Table_Dallas[Crc8 ^ _pBuff[i]];
    }

    return Crc8;
}

void proceed_uart(void)
#define NBROFDATATOREAD		1			// minimal packet size
#define CMD_REQ 0x1e
#define CMD_ANSW 0x2e
#define CMD_ANSW_ERROR_CRC 0x42
#define CMD_GET_VERSION 0x01
#define CMD_SET_HEAD 0x02
#define CMD_GET_STATUS 0x03
#define CMD_SEEK 0x04
#define CMD_READ_TRACK 0x05
#define CMD_SEEK_TRACK0 0x06
#define CMD_START_TIME_CLOCKS 0x07
#define CMD_GET_RPM 0x08
{
  uint8_t str[2];
  uint8_t str4[4];
  int SEEK_WAIT;
  int SEEK_PW;
  int16_t steps;

  int c = getchar_timeout_us(0);
  if (c != PICO_ERROR_TIMEOUT)
    {
      RX_Buffer[RxCount] = c;
      RxCount++;
      if (RX_Buffer[0] != CMD_REQ)
        {
          RX_Buffer[0] = 0x00;
          RX_Buffer[1] = 0x00;
          NbrOfDataToRead = 0;
          RxCount = 0;
          return;
        }

      if(RxCount <= NBROFDATATOREAD )
			return;

		if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_GET_VERSION))   
			NbrOfDataToRead = 3;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_SET_HEAD))
			NbrOfDataToRead = 4;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_GET_STATUS))
			NbrOfDataToRead = 3;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_SEEK)) // 1e 04 [fast/slow] [steps16bit] crc
			NbrOfDataToRead = 6;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_READ_TRACK))
			NbrOfDataToRead = 3;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_SEEK_TRACK0))
			NbrOfDataToRead = 3;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_START_TIME_CLOCKS)) // 1e 07 [1 2 3 4] crc
			NbrOfDataToRead = 7;
    else if (( RX_Buffer[0] == CMD_REQ) &&( RX_Buffer[1] == CMD_GET_RPM))
			NbrOfDataToRead = 3;
    else {
			RX_Buffer[0] = 0x00;
			RX_Buffer[1] = 0x00;
			NbrOfDataToRead = 0;
			RxCount = 0;
      return;
		}

		if( RxCount == NbrOfDataToRead ) 
    {
      led_lights(3);
      RxCount = 0;
      if (RX_Buffer[0] == CMD_REQ) 
      {
        if (RX_Buffer[NbrOfDataToRead - 1] != Crc8_Dallas(RX_Buffer, (NbrOfDataToRead - 1))) {
            TX_Buffer[0] = CMD_ANSW;
            TX_Buffer[1] = CMD_ANSW_ERROR_CRC;
            TX_Buffer[2] = Crc8_Dallas(TX_Buffer, 2);
            NbrOfDataToTransfer = 3;
            memset(RX_Buffer, 0, sizeof(RX_Buffer));
            return;
        }
		    
        switch (RX_Buffer[1])              
        {
          case CMD_GET_VERSION:
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_GET_VERSION;
                      TX_Buffer[2] = 0x10;
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      break;
          case CMD_SET_HEAD:
                      head_num = RX_Buffer[2];
                      gpio_put_masked(1UL << HEAD0 | 1UL << HEAD1 | 1UL << HEAD2 | 1UL << HEAD3,head_num);
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_SET_HEAD;
                      TX_Buffer[2] = head_num;
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      break;
          case CMD_GET_STATUS:
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_GET_STATUS;
                      TX_Buffer[2] = 0;
                      if (gpio_get(TRK0))     TX_Buffer[2] |= 1;
                      if (gpio_get(SELECTED)) TX_Buffer[2] |= 2;
                      if (gpio_get(SEEK))     TX_Buffer[2] |= 4;
                      if (gpio_get(W_FLT))    TX_Buffer[2] |= 8;
                      if (gpio_get(READY))    TX_Buffer[2] |= 16;
                      if (gpio_get(INDEX))    TX_Buffer[2] |= 32;
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      break;
          case CMD_SEEK:
                      str[0] = RX_Buffer[3];
                      str[1] = RX_Buffer[4];
                      steps = *((int16_t*) str);
                      if (steps<0) direction = 0;
                        else direction = 1;
                      // set direction
                      gpio_put(DIR,direction);
                      sleep_ms(10);
                      SEEK_WAIT = 60;   // 30 microseconds (fast)
                      SEEK_PW = 10;      // 5 microseconds (fast)
                      if (RX_Buffer[2] = 5) // CMD_SEEK_SLOW   5
                      {
                        SEEK_WAIT = 8000;   // 4 milliseconds
                        SEEK_PW = 80;       //  40 microseconds
                      }
                      // produce pulses
                      for (int i = 0; i < abs(steps); i++)
                      {
                        gpio_put(STEP,1);
                        sleep_us(SEEK_PW);
                        gpio_put(STEP,0);
                        sleep_us(SEEK_WAIT);
                      }
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_SEEK;
                      // wait seek complete
                      TX_Buffer[2] = 0xFF;
                      for (int i=0; i < 65535; i++)
                      {
                        if ((!gpio_get(SEEK)) && (!gpio_get(READY)) && (!gpio_get(SELECTED)))
                          {
                            TX_Buffer[2] = 0;
                            break;
                          }
                        sleep_ms(100);
                      }
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      break;
          case CMD_READ_TRACK:
                        CapArm();
                        // let the dma channel do its stuff
                        dma_channel_wait_for_finish_blocking(dma_chan);

                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_READ_TRACK;
                      TX_Buffer[2] = timer_errors;
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      for (int i = 0; i < NbrOfDataToTransfer; i++) putchar_raw(TX_Buffer[i]);
                      memset(TX_Buffer, 0, sizeof(TX_Buffer));
                      NbrOfDataToTransfer = 0;
                      for (int i = 0; i < MAX_DELTAS; i++) 
                      {
                        putchar_raw(deltas[i]);
                        //uint16_t tmp = deltas[i];
                        //putchar_raw((uint8_t)tmp);
                        //putchar_raw((uint8_t)tmp>>8);
                      }
                      break;
          case CMD_SEEK_TRACK0:
                      // set direction
                      gpio_put(DIR,0);
                      sleep_ms(10);
                      SEEK_WAIT = 8000;   // 30 microseconds (fast)
                      SEEK_PW = 80;      // 5 microseconds (fast)
                      // produce pulses
                      for (int i = 0; i < 4096; i++)
                      {
                        gpio_put(STEP,1);
                        sleep_us(300);
                        gpio_put(STEP,0);
                        sleep_us(1000);
                        if (!gpio_get(TRK0))
                          break;
                      }
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_SEEK_TRACK0;
                      // wait seek complete
                      TX_Buffer[2] = 0xFF;
                      for (int i=0; i < 65535; i++)
                      {
                        if ((!gpio_get(SEEK)) && (!gpio_get(READY)) && (!gpio_get(SELECTED)))
                          {
                            TX_Buffer[2] = 0;
                            break;
                          }
                        sleep_ms(100);
                      }
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      break;
          case CMD_START_TIME_CLOCKS:
                      str4[0] = RX_Buffer[2];
                      str4[1] = RX_Buffer[3];
                      str4[2] = RX_Buffer[4];
                      str4[3] = RX_Buffer[5];
                      start_time_clocks = *((uint32_t*) str4);
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_START_TIME_CLOCKS;
                      TX_Buffer[2] = start_time_clocks;
                      TX_Buffer[3] = Crc8_Dallas(TX_Buffer, 3);
                      NbrOfDataToTransfer = 4;
                      break;
          case CMD_GET_RPM:
                      read();
                      TX_Buffer[0] = CMD_ANSW;
                      TX_Buffer[1] = CMD_GET_RPM;
                      TX_Buffer[2] = (uint8_t) pulsewidth;
                      TX_Buffer[3] = (uint8_t) (pulsewidth >> 8);
                      TX_Buffer[4] = (uint8_t) (pulsewidth >> 16);
                      TX_Buffer[5] = (uint8_t) (pulsewidth >> 24);
                      TX_Buffer[6] = (uint8_t) period;
                      TX_Buffer[7] = (uint8_t) (period >> 8);
                      TX_Buffer[8] = (uint8_t) (period >> 16);
                      TX_Buffer[9] = (uint8_t) (period >> 24);
                      TX_Buffer[10] = Crc8_Dallas(TX_Buffer, 10);
                      NbrOfDataToTransfer = 11;
                      break;
          default:
                      break;
        }
      }

		}
    }
}
int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(400000, true);

    gpio_init_mask(1UL << LED_PIN | 1UL << HEAD0 | 1UL << HEAD1 | 1UL << HEAD2 | 1UL << HEAD3 | 1UL << DIR | 1UL << STEP | 1UL << SEEK | 1UL << TRK0 | 1UL << INDEX | 1UL << READY | 1UL << W_FLT | 1UL << SELECTED | 1UL << MFM_DATA);
    gpio_set_dir_in_masked(1UL << SEEK | 1UL << TRK0 | 1UL << INDEX | 1UL << READY | 1UL << W_FLT | 1UL << SELECTED | 1UL << MFM_DATA);
    gpio_set_dir_out_masked(1UL << LED_PIN | 1UL << HEAD0 | 1UL << HEAD1 | 1UL << HEAD2 | 1UL << HEAD3 | 1UL << DIR | 1UL << STEP);
    
    stdio_init_all();
    while (!tud_cdc_connected()) { sleep_ms(100);  }
    printf("tud_cdc_connected()\n");

    measure_freqs();

    PwmIn(INDEX);
    CapInit();

    printf("s - status\nd - direction\nt-seek\nr - rpm\nh - head\nx - ymodem\n");
    while (true) {
      proceed_uart();
        if (NbrOfDataToTransfer)
        {
          for (int i = 0; i < NbrOfDataToTransfer; i++) putchar_raw(TX_Buffer[i]);
          memset(TX_Buffer, 0, sizeof(TX_Buffer));
          NbrOfDataToTransfer = 0;

        }
/*        
        int c = getchar_timeout_us(0);
        if (c=='s')
          {
            printf("Status: ");
            if (!gpio_get(SEEK)) printf("SEEK_COMPLETED "); else printf("SEEK_NOT_COMPLETED ");
            if (!gpio_get(TRK0)) printf ("TRK0 "); else printf("NO_TRK0 ");
            if (!gpio_get(READY)) printf ("READY "); else printf("NOT_READY ");
            if (!gpio_get(W_FLT)) printf("WRITE_FAULT "); else printf("NO_WRT_FLT ");
            if (!gpio_get(SELECTED)) printf("SELECTED "); else printf("NO_SELECTED ");
            printf("\nDirection: %u ; Cyl_Num = %u ; Head_Num = %u",direction,cyl_num,head_num);
            printf("\n.\n");
          }
        if (c=='d')
          {
            direction ^= 1;
            printf("Set direction = %u\n.\n",direction);
          }
        if (c=='t')
          {
            gpio_put(DIR,direction);
            sleep_us(50);
            gpio_put(STEP,1);
            sleep_us(50);
            gpio_put(STEP,0);
            if (direction == 1)
               cyl_num++;
            else
               if(cyl_num >= 1) cyl_num--;
            printf("Step direction %u; cyl_num = %u ; head_num = %u\n.\n",direction,cyl_num,head_num);
          }
        if (c=='h')
         {
            if ((head_num) < 16) head_num++;
            gpio_put_masked(1UL << HEAD0 | 1UL << HEAD1 | 1UL << HEAD2 | 1UL << HEAD3,head_num);
            printf("Step direction %u; cyl_num = %u ; head_num = %u\n.\n",direction,cyl_num,head_num);
         }
        if (c=='r')
         {
            read_PWM(pwm_reading);
            if (pwm_reading[0] >= 0.)
            {
              printf("pw=%.8f \tp=%.8f \tdc=%.8f\n", pwm_reading[0], pwm_reading[1], pwm_reading[2]);
              printf("rpm = %.8f\n.\n", pwm_reading[1] * 60 * 60 * 60);
            }
            else
            {
              printf("rpm not detected\n.\n");
            }
         }
        if (c=='x')
         {
              CapArm();
              // let the dma channel do its stuff
              dma_channel_wait_for_finish_blocking(dma_chan);
              sprintf(fname,"pico%u.bin",cyl_num);
              printf("Start transfering %s via Y-Modem, fsize = %u, wait 10 sec...\n",fname,MAX_DELTAS*2);
              sleep_ms(10000);
              Ymodem_Transmit((char*)deltas,fname,MAX_DELTAS*2);
              //MFM_Decode(MAX_DELTAS);
              //DumpHex(&deltas,MAX_DELTAS);
         }
*/
/*
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
*/
    }
    return 0;
#endif
}
