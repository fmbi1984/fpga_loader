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

#include "flash.h"
#include <SPI_Master.h>

void waitBusy() {
  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x05); //read status register
  while (SPI_Master.transfer(0) & 0x01);
  SET(CS_FLASH, HIGH);
}

void SPI_Setup() {
  OUT(SS); // prevent SPI from entering slave mode
  SET(SS, HIGH);
  IN(MISO);
  SPI_Master.begin();
  SPI_Master.setFrequency(SPI_8M);
  SPI_Master.setSPIMode(SPI_MODE0);
  SPI_Master.setBitORDER(MSBFIRST);
}

void eraseFlash() {
  SPI_Setup();

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x06); // Write mode
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x01); // Write status reg
  SPI_Master.transfer(0x00); // Disable protection
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x06); // Write mode
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x60); // Full chip erase
  SET(CS_FLASH, HIGH);

  waitBusy(); // Wait for operation to finish

  SPI_Master.endTransfer();
}

void writeByteFlash(uint32_t address, uint8_t b) {
  SPI_Setup();

  waitBusy(); // wait for other writes

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x06); // Write mode
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x02); // Single write
  SPI_Master.transfer(address >> 16);
  SPI_Master.transfer(address >> 8);
  SPI_Master.transfer(address);
  SPI_Master.transfer(b);
  SET(CS_FLASH, HIGH);

  SPI_Master.endTransfer();
}

void waitHardwareBusy() {
  SET(CS_FLASH, LOW);
  delayMicroseconds(1); // change to 1
  while (digitalRead(MISO) == 0);
  SET(CS_FLASH, HIGH);
}

void writeFlash(uint32_t startAddress, uint8_t *data, uint16_t length) {
  if (length < 2) {
    if (length == 1)
      writeByteFlash(startAddress, data[0]);
    return;
  }

  uint16_t pos;

  SPI_Setup();

  waitBusy();

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x70); // Hardware EOW detection
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x06); // Write Enable
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0xAD); // Auto-increment write
  SPI_Master.transfer(startAddress >> 16);
  SPI_Master.transfer(startAddress >> 8);
  SPI_Master.transfer(startAddress);
  SPI_Master.transfer(data[0]);
  SPI_Master.transfer(data[1]);
  SET(CS_FLASH, HIGH);

  waitHardwareBusy();

  for (pos = 2; pos <= length - 2; pos += 2) {
    SET(CS_FLASH, LOW);
    SPI_Master.transfer(0xAD); // Auto-increment write
    SPI_Master.transfer(data[pos]);
    SPI_Master.transfer(data[pos + 1]);
    SET(CS_FLASH, HIGH);

    waitHardwareBusy();
  }

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x04); // Write Disable
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x80); // Disable hardware EOW detection
  SET(CS_FLASH, HIGH);

  if (pos < length)
    writeByteFlash(startAddress + pos, data[pos]);
}

void readFlash(volatile uint8_t* buffer, uint32_t address, uint16_t count) {
  SPI_Setup();

  SET(CS_FLASH, LOW);
  SPI_Master.transfer(0x03); // Read Array (Low Frequency)
  SPI_Master.transfer(address >> 16);
  SPI_Master.transfer(address >> 8);
  SPI_Master.transfer(address);

  for (uint16_t c = 0; c < count; c++)
    buffer[c] = SPI_Master.transfer(0);
  SET(CS_FLASH, HIGH);

  SPI_Master.endTransfer();
}

// enable writing
void writeEnable ()
{
  waitBusy();

  SET(CS_FLASH, LOW);
  SPI_Master.transfer (0x06);       
  SET(CS_FLASH, HIGH);  
}  // end of writeEnable

// read device status
byte readStatus (void)
{
  SET(CS_FLASH, LOW);
  SPI_Master.transfer (0x05);       
  byte status = SPI_Master.transfer (status);       
  SET(CS_FLASH, HIGH);  
  
  return status;
}  // end of readStatus

// write status register
void writeStatus (const byte status)
{
   writeEnable ();
   waitBusy();  // wait until ready
   
   SET(CS_FLASH, LOW);
   SPI_Master.transfer (0x01);       
   SPI_Master.transfer (status);       
   SET(CS_FLASH, HIGH);  
}  // end of writeStatus

void info ()
{
  
  waitBusy(); // wait until ready
  
  SET(CS_FLASH, LOW);
  SPI_Master.transfer (0x9F);       
  
  Serial.print ("Manufacturer: ");
  Serial.println (SPI_Master.transfer (0), HEX);
  Serial.print ("Device ID Part 1: ");
  Serial.println (SPI_Master.transfer (0), HEX);
  Serial.print ("Device ID Part 2: ");
  Serial.println (SPI_Master.transfer (0), HEX);
  Serial.print ("Extended Information Length: ");
  Serial.println (SPI_Master.transfer (0),HEX);

  SET(CS_FLASH, HIGH);
  
  Serial.print ("Status: ");
  Serial.println (readStatus (), HEX);

} // end of info


