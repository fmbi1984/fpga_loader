/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HARDWARE_H_
#define HARDWARE_H_

#include <stdint.h>
#include <stdbool.h>

#include "Arduino.h"
#include <SPI_Master.h>

/*
#define FPGA_BUS_PORT PORTB
#define FPGA_BUS_DDR DDRB
#define FPGA_BUS_PIN PINB
#define FPGA_BUS_MASK 0xFF
#define FPGA_BUS_OFFSET 0

#define ADC_BUS_PORT PORTB
#define ADC_BUS_DDR DDRB
#define ADC_BUS_PIN PINB
#define ADC_BUS_MASK 0xF0
#define ADC_BUS_OFFSET 4

#define CCLK_N 3
#define CS_FLASH_N 2
#define INIT_N 30
#define TX_BUSY_N 30
#define DONE_N 5
#define PROGRAM_N 13
*/
#include "Arduino.h"

#define CCLK          D11 // P0_12
#define CS_FLASH      D12 // P0_13
#define SCK           D16 // P0_25
#define INIT          D7  // P0_17
#define TX_BUSY       D7  // P0_17
#define DONE          D13 // P0_15
#define PROGRAM       D8  // P0_19
#define MISO          D17 // P0_22
#define MOSI          D18 // P0_20
#define SS            D10 // P0_14
#define DTR           D4  // P0_21

#define FPGA_BUS_D0         SS          // B0
#define FPGA_BUS_D0_MASK    0b00000001
#define FPGA_BUS_D1         SCK         // B1
#define FPGA_BUS_D1_MASK    0b00000010
#define FPGA_BUS_D2         MOSI        // B2
#define FPGA_BUS_D2_MASK    0b00000100
#define FPGA_BUS_D3         MISO        // B3
#define FPGA_BUS_D3_MASK    0b00001000

#define FPGA_BUS_D4         A3          // B4
#define FPGA_BUS_D4_MASK    0b00010000
#define FPGA_BUS_D5         A2          // B5
#define FPGA_BUS_D5_MASK    0b00100000
#define FPGA_BUS_D6         A1          // B6
#define FPGA_BUS_D6_MASK    0b01000000
#define FPGA_BUS_D7         A0          // B7
#define FPGA_BUS_D7_MASK    0b10000000

#define FPGA_BUS_DDR(dir) pinMode(FPGA_BUS_D0, (dir & FPGA_BUS_D0_MASK)>>0); pinMode(FPGA_BUS_D1, (dir & FPGA_BUS_D1_MASK)>>1); pinMode(FPGA_BUS_D2, (dir & FPGA_BUS_D2_MASK)>>2); pinMode(FPGA_BUS_D3, (dir & FPGA_BUS_D3_MASK)>>3); pinMode(FPGA_BUS_D4, (dir & FPGA_BUS_D4_MASK)>>4); pinMode(FPGA_BUS_D5, (dir & FPGA_BUS_D5_MASK)>>5); pinMode(FPGA_BUS_D6, (dir & FPGA_BUS_D6_MASK)>>6); pinMode(FPGA_BUS_D7, (dir & FPGA_BUS_D7_MASK)>>7);
#define FPGA_BUS_PORT(data) digitalWrite(FPGA_BUS_D0, (data & FPGA_BUS_D0_MASK)>>0); digitalWrite(FPGA_BUS_D1, (data & FPGA_BUS_D1_MASK)>>1); digitalWrite(FPGA_BUS_D2, (data & FPGA_BUS_D2_MASK)>>2); digitalWrite(FPGA_BUS_D3, (data & FPGA_BUS_D3_MASK)>>3); digitalWrite(FPGA_BUS_D4, (data & FPGA_BUS_D4_MASK)>>4); digitalWrite(FPGA_BUS_D5, (data & FPGA_BUS_D5_MASK)>>5); digitalWrite(FPGA_BUS_D6, (data & FPGA_BUS_D6_MASK)>>6); digitalWrite(FPGA_BUS_D7, (data & FPGA_BUS_D7_MASK)>>7);

#define TOGGLE(p) digitalWrite(p, ~digitalRead(p))
#define SET(p, v) digitalWrite(p, v)
#define VALUE(p) digitalRead(p)
#define OUT(p) pinMode(p, OUTPUT)
#define IN(p) pinMode(p, INPUT)

#define LINESTATE_DTR  1

#define LED_R D6
#define LED_G D9
#define LED_B D5

#endif /* HARDWARE_H_ */
