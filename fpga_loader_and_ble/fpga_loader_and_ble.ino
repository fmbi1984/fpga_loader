#include "Arduino.h"
#include "hardware.h"
#include "ring_buffer.h"
#include "fpga.h"
#include "flash.h"
#include "SPI_Master.h"
#include <BLE_API.h>

typedef enum {
  IDLE,
  READ_SIZE,
  WRITE_TO_FLASH,
  WRITE_TO_FPGA,
  VERIFY_FLASH,
  LOAD_FROM_FLASH
}
loaderState_t;

typedef enum {
  WAIT, START_LOAD, LOAD, SERVICE
}
taskState_t;

#define BUFFER_SIZE 1024 // Mojo V3

uint8_t loadBuffer[BUFFER_SIZE + 128];

volatile taskState_t taskState = SERVICE;

static uint32_t byteCountLoop=0;
static uint32_t transferSizeLoop=0;

#define TXRX_BUF_LEN                      20

BLE                                       ble;
Timeout                                   timeout;

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state=0;

// The Nordic UART Service
static const uint8_t service1_uuid[]                = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]             = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]             = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]           = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};

GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));


void disconnectionCallBack(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    ble.startAdvertising();
    Serial.print("\\l0\n");
}

void onConnectionCallback(const Gap::ConnectionCallbackParams_t *p_conn_param)
{
  Serial.print("\\l1\n");
}

void writtenHandle(const GattWriteCallbackParams *Handler)
{
    uint8_t buf[TXRX_BUF_LEN];
    uint16_t bytesRead, index;

    if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
        ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
        for(byte index=0; index<bytesRead; index++) {
            Serial.write(buf[index]);
        }
    }
}

void m_uart_rx_handle()
{   //update characteristic data
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);
    memset(rx_buf, 0x00,20);
    rx_state = 0;
}



/* this is used to undo any setup you did in initPostLoad */
void disablePostLoad() {

  SPI_Master.endTransfer();  // disable SPI
  SET(CCLK, LOW);
  OUT(PROGRAM);
  SET(PROGRAM, LOW); // reset the FPGA
  IN(INIT);
  SET(INIT, HIGH); // pullup on INIT
}

/* Here you can do some setup before entering the userLoop loop */
void initPostLoad() {
  //Serial.flush();

  // Setup all the SPI pins
  OUT(CS_FLASH);
  SET(CS_FLASH, HIGH);
  OUT(SS);
  SET(SS, HIGH);
  SPI_Setup(); // enable the SPI Port

  // This pin is used to signal the serial buffer is almost full
  OUT(TX_BUSY);
  SET(TX_BUSY, LOW);

  // set progam as an input so that it's possible to use a JTAG programmer with the Mojo
  IN(PROGRAM);

  // the FPGA looks for CCLK to be high to know the AVR is ready for data
  SET(CCLK, HIGH);
  IN(CCLK); // set as pull up so JTAG can work

  userSetup();
}

void setup() {
  OUT(LED_R);
  OUT(LED_G);
  OUT(LED_B);
  SET(LED_R, HIGH);
  SET(LED_G, HIGH);
  SET(LED_B, HIGH);

  OUT(CS_FLASH);
  SET(CS_FLASH, HIGH);
  OUT(CCLK);
  OUT(PROGRAM);

  IN(INIT);
  SET(INIT, HIGH); // pullup on INIT

  IN(DONE);

  //Serial.begin(115200); // Baud rate does nothing
  Serial.begin(9600);
  Serial.attach(uart_handle);

  loadFromFlash(); // load on power up
  initPostLoad();

  // set progam as an input so that it's possible to use a JTAG programmer with the Mojo
  IN(PROGRAM);
  IN(DONE);
}

void loop() {
  static loaderState_t state = IDLE;
  static int8_t destination;
  static int8_t verify;
  static uint32_t byteCount;
  static uint32_t transferSize;

  int16_t w;
  uint8_t bt;
  uint8_t buffIdx;

  switch (taskState) {
    case WAIT:
      break;
    case START_LOAD: // command to enter loader mode
      disablePostLoad(); // setup peripherals
      taskState = LOAD; // enter loader mode
      state = IDLE; // in idle state
      break;
    case LOAD:
      w = Serial.read();
      bt = (uint8_t) w;
      if (w >= 0) { // if we have data
        switch (state) {
          case IDLE: // in IDLE we are waiting for a command from the PC
            byteCount = 0;
            transferSize = 0;
            if (bt == 'F') { // write to flash
              destination = 0; // flash
              verify = 0; // don't verify
              state = READ_SIZE;
              Serial.write('R'); // signal we are ready
            }
            if (bt == 'V') { // write to flash and verify
              destination = 0; // flash
              verify = 1; // verify
              state = READ_SIZE;
              Serial.write('R'); // signal we are ready
            }
            if (bt == 'R') { // write to RAM
              destination = 1; // ram
              state = READ_SIZE;
              Serial.write('R'); // signal we are ready
            }
            if (bt == 'E') { //erase
              eraseFlash();
              Serial.write('D'); // signal we are done
            }
            //Serial.flush();
            break;
          case READ_SIZE: // we need to read in how many bytes the config data is
            transferSize |= ((uint32_t) bt << (byteCount++ * 8));
            transferSizeLoop = transferSize;
            if (byteCount > 3) {
              byteCount = 0;
              if (destination) {
                state = WRITE_TO_FPGA;
                initLoad(); // get the FPGA read for a load
                startLoad(); // start the load
              }
              else {
                state = WRITE_TO_FLASH;
                eraseFlash();
              }
              Serial.write('O'); // signal the size was read
              //Serial.flush();
            }
            break;
          case WRITE_TO_FLASH:
            // we can only use the batch write for even addresses
            // so address 5 is written as a single byte
            if (byteCount == 0)
              writeByteFlash(5, bt);

            buffIdx = (byteCount++ - 1) % 256;
            loadBuffer[buffIdx] = bt;

            if (buffIdx == 255 && byteCount != 0)
              writeFlash(byteCount + 5 - 256, loadBuffer, 256); // write blocks of 256 bytes at a time for speed

            if (byteCount == transferSize) { // the last block to write

              if (buffIdx != 255) // finish the partial block write
                writeFlash(byteCount + 5 - (buffIdx + 1), loadBuffer,
                           buffIdx + 1);
              delayMicroseconds(50); // these are necciary to get reliable writes
              
              uint32_t size = byteCount + 5;
              for (uint8_t k = 0; k < 4; k++) {
                writeByteFlash(k + 1, (size >> (k * 8)) & 0xFF); // write the size of the config data to the flash
                delayMicroseconds(50);
              }
              
              writeByteFlash(0, 0xAA); // 0xAA is used to signal the flash has valid data
              delayMicroseconds(50);

              Serial.write('D'); // signal we are done
              //Serial.flush(); // make sure it sends

              if (verify) {
                state = VERIFY_FLASH;
                
              }
              else {
                state = LOAD_FROM_FLASH;

              }
            }
            break;
          case WRITE_TO_FPGA:
            
            sendByte(bt); // just send the byte!
            if (++byteCount == transferSize) { // if we are done
              sendExtraClocks(); // send some extra clocks to make sure the FPGA is happy
              state = IDLE;
              //byteCountLoop = byteCount;
              taskState = SERVICE; // enter user mode
              initPostLoad();
              Serial.write('D'); // signal we are done
              //Serial.flush();
            }
            break;
          case VERIFY_FLASH:
            if (bt == 'S') {
              byteCount += 5;
              for (uint32_t k = 0; k < byteCount; k += 256) { // dump all the flash data
                uint16_t s;
                if (k + 256 <= byteCount) {
                  s = 256;
                }
                else {
                  s = byteCount - k;
                }
                readFlash(loadBuffer, k, s); // read blocks of 256
                uint16_t br = Serial.write((uint8_t*) loadBuffer, s); // dump them to the serial port
                //k -= (256 - br); // if all the bytes weren't sent, resend them next round
                //Serial.flush();
                //delay(10); // needed to prevent errors in some computers running Windows (give it time to process the data?)
                //setRGB(0,255,0);
                //delay(5);
                //setRGB(0,0,0);
                //delay(5);
              }
              
              state = LOAD_FROM_FLASH;
            }
            break;
          case LOAD_FROM_FLASH:
            
            if (bt == 'L') {
              loadFromFlash(); // load 'er up!
              Serial.write('D'); // loading done
              //Serial.flush();
              state = IDLE;
              taskState = SERVICE;
              initPostLoad();
              
            }
            break;
        }
      }

      break;
    case SERVICE:
      userLoop(); // loop the user code
      break;
  }
}

void userSetup() {
  // put your setup code here, to run once
    //Serial.begin(9600);
    //Serial.attach(uart_handle);

    ble.init();
    ble.onDisconnection(disconnectionCallBack);
    ble.onConnection(onConnectionCallback);
    ble.onDataWritten(writtenHandle);

    // setup adv_data and srp_data
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"TXRX", sizeof("TXRX") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));

    // set adv_type
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    // add service
    ble.addService(uartService);
    // set device name
    ble.setDeviceName((const uint8_t *)"VendBox");
    // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
    ble.setTxPower(4);
    // set adv_interval, 100ms in multiples of 0.625ms.
    ble.setAdvertisingInterval(160);
    // set adv_timeout, in seconds
    ble.setAdvertisingTimeout(0);
    // start advertising
    ble.startAdvertising();

    pinMode(D3, INPUT);
    digitalWrite(D3, HIGH);
}
/* This is where you should add your own code! Feel free to edit anything here.
   This function will work just like the Arduino loop() function in that it will
   be called over and over. You should try to not delay too long in the loop to
   allow the Mojo to enter loading mode when requested. */
void userLoop() {
  setRGB(0, 0, 255);
  delay(100);
  setRGB(0, 0, 0);
  delay(100);
  ble.waitForEvent();

  /*
  Serial.print("Transfer Size: ");
  Serial.println(transferSizeLoop);
  Serial.print("Byte Count: ");
  Serial.println(byteCountLoop);
  */
}

void uart_handle(uint32_t id, SerialIrq event)
{ 
  /* Serial rx IRQ */
  /*
  if (event == RxIrq)
  {
    if (taskState == SERVICE)
    {
      if (Serial.available() > 0)
      {

        char c = Serial.peek();

        if (c == 'r')
        {

          Serial.read();
          //Serial.flush();
          taskState = START_LOAD;

        }
        
        //if (c == '@')
        //{
        //Serial.read();
        ////Serial.flush();
        //taskState = SERVICE;
        //}
      }

      
    }
  }
  */

  /* Serial rx IRQ */
    if(event == RxIrq) {
      
        if (rx_state == 0) {
            rx_state = 1;
            timeout.attach_us(m_uart_rx_handle, 100000);
            rx_buf_num=0;
        }
        while(Serial.available()) {
            if(rx_buf_num < 20) {
                rx_buf[rx_buf_num] = Serial.read();
                rx_buf_num++;
            }
            else {
                Serial.read();
            }
        }
    }
}

void setRGB(int r, int g, int b)
{
  analogWrite(LED_R, 255 - r);
  analogWrite(LED_G, 255 - g);
  analogWrite(LED_B, 255 - b);
}



