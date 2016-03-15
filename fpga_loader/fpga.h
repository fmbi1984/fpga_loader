#ifndef FPGA_H_
#define FPGA_H_

#include <stdint.h>
#include <stdbool.h>
#include "Arduino.h"
#include <SPI_Master.h>
#include "hardware.h"
#include "flash.h"

#define BUFFER_SIZE 1024

extern uint8_t loadBuffer[BUFFER_SIZE + 128];

void enableDataBus();
void initLoad();
void startLoad();
void sendByte(uint8_t b);
void sendExtraClocks();
void loadFromFlash();

#endif /* FPGA_H_ */
