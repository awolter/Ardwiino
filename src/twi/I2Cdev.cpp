#include "I2Cdev.h"
#include "../mpu6050/inv_mpu.h"
#include "../mpu6050/inv_mpu_dmp_motion_driver.h"
#include "../wii/internal/NXC_Identity.h"
#define BIT_FIFO_OVERFLOW (0x10)
TWIInfoStruct I2Cdev::TWIInfo;
int I2Cdev::RXBuffLen;
int I2Cdev::TXBuffLen;
int I2Cdev::RXBuffIndex;
uint8_t I2Cdev::andCheck;
uint8_t I2Cdev::orCheck;
volatile int I2Cdev::TXBuffIndex;
volatile uint8_t I2Cdev::TWIReceiveBuffer[RXMAXBUFLEN];
uint8_t I2Cdev::TWITransmitBuffer[TXMAXBUFLEN];
uint16_t I2Cdev::mpuCount;
AccelDataUnion I2Cdev::accelData;
int I2Cdev::pollDevice = 0;
volatile bool I2Cdev::needsData;

ExtensionPort I2Cdev::port;
/** Default constructor.
 */
I2Cdev::I2Cdev() {}
void I2Cdev::startPolling() {
  pollDevice = 1;
  nextPollingReq();
}
void I2Cdev::nextPollingReq() {
  if (pollDevice == 0)
    return;
  uint8_t addr;
  uint8_t value;
  uint8_t dev = WII_I2C_ADDR;
  needsData = false;
  switch (pollDevice) {
  case 1:
    addr = 0xFA;
    needsData = true;
    break;
  case 2:
    orCheck = 0x00;
    andCheck = 0xFF;
    addr = 0x00;
    needsData = true;
    break;
  case 3:
    addr = 0xF0;
    value = 0x55;
    break;
  case 4:
    addr = 0xFB;
    value = 0x00;
    break;
  case 5:
    dev = MPU_ADDR;
    addr = 0x72;
    needsData = true;
    break;
  case 6:
    dev = MPU_ADDR;
    addr = 0x3A;
    needsData = true;
    break;
  case 7:
    dev = MPU_ADDR;
    addr = 0x74;
    needsData = true;
    break;
  }
  uint8_t msg[] = {(dev << 1) & 0xFE, addr, value};
  TWITransmitData(msg, needsData ? 2 : 3, 0);
}
void I2Cdev::nextPollingData() {
  _delay_us(200);
  if (pollDevice == 0)
    return;
  if (needsData) {
    needsData = false;
    switch (pollDevice) {
    case 1:
      TWIReadData(WII_I2C_ADDR, WII_ID_SIZE, false);
      break;
    case 2:
      TWIReadData(WII_I2C_ADDR, 6, false);
      break;
    case 5:
      TWIReadData(MPU_ADDR, 2, false);
      break;
    case 6:
      TWIReadData(MPU_ADDR, 1, false);
      break;
    case 7:
      TWIReadData(MPU_ADDR, 16, false);
      break;
    }
  } else {
    switch (pollDevice) {
    case 1:
      port.data.connectedType =
          NintendoExtensionCtrl::identifyController(TWIReceiveBuffer);
      pollDevice = 2;
      break;
    case 2:
      for (int i = 0; i < RXBuffLen; i++) {
        port.data.controlData[i] = TWIReceiveBuffer[i];
      }
      if (orCheck == 0x00 || andCheck == 0xFF) {
        pollDevice = 3;
      } else {
        pollDevice = 1;
      }
      break;
    case 3:
      pollDevice = 4;
      break;
    case 4:
      pollDevice = 5;
      break;
    case 5:
      mpuCount = TWIReceiveBuffer[0] << 8 | TWIReceiveBuffer[1];
      if (mpuCount > 512) {
        pollDevice = 6;
      } else if (mpuCount > 16) {
        pollDevice = 7;
      } else {
        pollDevice = 1;
      }
      break;
    case 6:
      if (TWIReceiveBuffer[0] & BIT_FIFO_OVERFLOW) {
        mpu_reset_fifo();
      }
      pollDevice = 7;
      break;
    case 7:
      accelData._l[0] = ((long)TWIReceiveBuffer[0] << 24) |
                        ((long)TWIReceiveBuffer[1] << 16) |
                        ((long)TWIReceiveBuffer[2] << 8) | TWIReceiveBuffer[3];
      accelData._l[1] = ((long)TWIReceiveBuffer[4] << 24) |
                        ((long)TWIReceiveBuffer[5] << 16) |
                        ((long)TWIReceiveBuffer[6] << 8) | TWIReceiveBuffer[7];
      accelData._l[2] = ((long)TWIReceiveBuffer[8] << 24) |
                        ((long)TWIReceiveBuffer[9] << 16) |
                        ((long)TWIReceiveBuffer[10] << 8) |
                        TWIReceiveBuffer[11];
      accelData._l[3] = ((long)TWIReceiveBuffer[12] << 24) |
                        ((long)TWIReceiveBuffer[13] << 16) |
                        ((long)TWIReceiveBuffer[14] << 8) |
                        TWIReceiveBuffer[15];
      accelData._f.w = (float)accelData._l[0] / (float)QUAT_SENS;
      accelData._f.x = (float)accelData._l[1] / (float)QUAT_SENS;
      accelData._f.y = (float)accelData._l[2] / (float)QUAT_SENS;
      accelData._f.z = (float)accelData._l[3] / (float)QUAT_SENS;
      pollDevice = 1;
      break;
    }
    nextPollingReq();
  }
}
void I2Cdev::TWIInit() {
  TWIInfo.mode = Ready;
  TWIInfo.errorCode = 0xFF;
  TWIInfo.repStart = 0;
  // Set pre-scalers (no pre-scaling)
  TWSR = 0;
  // Set bit rate
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
  // Enable TWI and interrupt
  TWCR = (1 << TWIE) | (1 << TWEN);
  // Serial.println("TWIInit");
}

uint8_t I2Cdev::isTWIReady() {
  return (TWIInfo.mode == Ready) || (TWIInfo.mode == RepeatedStartSent);
}
uint8_t I2Cdev::TWIWriteRegister(uint8_t device, uint8_t addr, uint8_t value) {
  uint8_t msg[] = {(device << 1) & 0xFE, addr, value};
  TWITransmitData(msg, 3, 0);
  WAIT_TWI;
  return 1;
}
uint8_t I2Cdev::TWIWriteRegisterMultiple(uint8_t device, uint8_t addr,
                                         uint8_t *value, uint8_t bytesToWrite) {
  uint8_t msg[2 + bytesToWrite];
  msg[0] = (device << 1) & 0xFE;
  msg[1] = addr;
  for (int i = 0; i < bytesToWrite; i++) {
    msg[i + 2] = value[i];
  }

  TWITransmitData(msg, bytesToWrite + 2, 0);
  WAIT_TWI;
  return 1;
}
uint8_t I2Cdev::TWIReadRegister(uint8_t device, uint8_t address,
                                uint8_t bytesToRead) {
  uint8_t msg[] = {(device << 1) & 0xFE, address};
  TWITransmitData(msg, 2, 0);
  WAIT_TWI;
  TWIReadData(device, bytesToRead, 0);
  WAIT_TWI;
  return 1;
}
uint8_t I2Cdev::TWITransmitData(void *const TXdata, uint8_t dataLen,
                                uint8_t repStart) {
  if (dataLen <= TXMAXBUFLEN) {
    // Wait until ready
    WAIT_TWI;
    // Set repeated start mode
    TWIInfo.repStart = repStart;
    // Copy data into the transmit buffer
    uint8_t *data = (uint8_t *)TXdata;
    for (int i = 0; i < dataLen; i++) {
      TWITransmitBuffer[i] = data[i];
    }
    // Copy transmit info to global variables
    TXBuffLen = dataLen;
    TXBuffIndex = 0;

    // If a repeated start has been sent, then devices are already listening
    // for an address and another start does not need to be sent.
    if (TWIInfo.mode == RepeatedStartSent) {
      TWIInfo.mode = Initializing;
      TWDR = TWITransmitBuffer[TXBuffIndex++]; // Load data to transmit buffer
      TWISendTransmit();                       // Send the data
    } else // Otherwise, just send the normal start signal to begin
           // transmission.
    {
      TWIInfo.mode = Initializing;
      TWISendStart();
    }
  } else {
    return 1; // return an error if data length is longer than buffer
  }
  return 0;
}

uint8_t I2Cdev::TWIReadData(uint8_t TWIaddr, uint8_t bytesToRead,
                            uint8_t repStart) {
  // Check if number of bytes to read can fit in the RXbuffer
  if (bytesToRead < RXMAXBUFLEN) {
    // Reset buffer index and set RXBuffLen to the number of bytes to read
    RXBuffIndex = 0;
    RXBuffLen = bytesToRead;
    // Create the one value array for the address to be transmitted
    uint8_t TXdata[1];
    // Shift the address and AND a 1 into the read write bit (set to write
    // mode)
    TXdata[0] = (TWIaddr << 1) | 0x01;
    // Use the TWITransmitData function to initialize the transfer and
    // address the slave
    TWITransmitData(TXdata, 1, repStart);
  } else {
    return 0;
  }
  return 1;
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave
 * off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                         uint8_t *data, uint16_t timeout) {
  TWIReadRegister(devAddr, regAddr, length);
  for (int i = 0; i < length; i++) {
    data[i] = TWIReceiveBuffer[i];
  }
  return length;
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint8_t *data) {
  return TWIWriteRegisterMultiple(devAddr, regAddr, data, length);
}

void I2Cdev::interrupt() {
  switch (TWI_STATUS) {
  // ----\/ ---- MASTER TRANSMITTER OR WRITING ADDRESS ----\/ ----  //
  case TWI_MT_SLAW_ACK: // SLA+W transmitted and ACK received
    // Set mode to Master Transmitter
    TWIInfo.mode = MasterTransmitter;
  case TWI_START_SENT:           // Start condition has been transmitted
  case TWI_MT_DATA_ACK:          // Data byte has been transmitted, ACK received
    if (TXBuffIndex < TXBuffLen) // If there is more data to send
    {
      TWDR = TWITransmitBuffer[TXBuffIndex++]; // Load data to transmit buffer
      TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
      TWISendTransmit(); // Send the data
    }
    // This transmission is complete however do not release bus yet
    else if (TWIInfo.repStart) {
      TWIInfo.errorCode = 0xFF;
      TWISendStart();
    }
    // All transmissions are complete, exit
    else {
      TWIInfo.mode = Ready;
      TWIInfo.errorCode = 0xFF;
      TWISendStop();
      nextPollingData();
    }
    break;

    // ----\/ ---- MASTER RECEIVER ----\/ ----  //

  case TWI_MR_SLAR_ACK: // SLA+R has been transmitted, ACK has been received
    // Switch to Master Receiver mode
    TWIInfo.mode = MasterReceiver;
    // If there is more than one byte to be read, receive data byte and
    // return an ACK
    if (RXBuffIndex < RXBuffLen - 1) {
      TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
      TWISendACK();
    }
    // Otherwise when a data byte (the only data byte) is received, return
    // NACK
    else {
      TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
      TWISendNACK();
    }
    break;

  case TWI_MR_DATA_ACK: // Data has been received, ACK has been transmitted.

    /// -- HANDLE DATA BYTE --- ///
    TWIReceiveBuffer[RXBuffIndex++] = TWDR;
    // If there is more than one byte to be read, receive data byte and
    // return an ACK
    if (RXBuffIndex < RXBuffLen - 1) {
      TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
      TWISendACK();
    }
    // Otherwise when a data byte (the only data byte) is received, return
    // NACK
    else {
      TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
      TWISendNACK();
    }
    break;

  case TWI_MR_DATA_NACK: // Data byte has been received, NACK has been
                         // transmitted. End of transmission.

    /// -- HANDLE DATA BYTE --- ///
    TWIReceiveBuffer[RXBuffIndex++] = TWDR;
    if (pollDevice == 2) {
      orCheck |= TWIReceiveBuffer[RXBuffIndex - 1];
      andCheck &= TWIReceiveBuffer[RXBuffIndex - 1];
    }
    // This transmission is complete however do not release bus yet
    if (TWIInfo.repStart) {
      TWIInfo.errorCode = 0xFF;
      TWISendStart();
    }
    // All transmissions are complete, exit
    else {
      TWIInfo.mode = Ready;
      TWIInfo.errorCode = 0xFF;
      TWISendStop();
      nextPollingData();
    }
    break;

    // ----\/ ---- MT and MR common ----\/ ---- //

  case TWI_MR_SLAR_NACK: // SLA+R transmitted, NACK received
  case TWI_MT_SLAW_NACK: // SLA+W transmitted, NACK received
  case TWI_MT_DATA_NACK: // Data byte has been transmitted, NACK received
  case TWI_LOST_ARBIT:   // Arbitration has been lost
    // Return error and send stop and set mode to ready
    if (TWIInfo.repStart) {
      TWIInfo.errorCode = TWI_STATUS;
      TWISendStart();
    }
    // All transmissions are complete, exit
    else {
      TWIInfo.mode = Ready;
      TWIInfo.errorCode = TWI_STATUS;
      TWISendStop();
      nextPollingData();
    }
    break;
  case TWI_REP_START_SENT: // Repeated start has been transmitted
    // Set the mode but DO NOT clear TWINT as the next data is not yet ready
    TWIInfo.mode = RepeatedStartSent;
    break;

  // ----\/ ---- SLAVE RECEIVER ----\/ ----  //

  // TODO  IMPLEMENT SLAVE RECEIVER FUNCTIONALITY

  // ----\/ ---- SLAVE TRANSMITTER ----\/ ----  //

  // TODO  IMPLEMENT SLAVE TRANSMITTER FUNCTIONALITY

  // ----\/ ---- MISCELLANEOUS STATES ----\/ ----  //
  case TWI_NO_RELEVANT_INFO: // It is not really possible to get into this
                             // ISR on this condition Rather, it is there to
                             // be manually set between operations
    break;
  case TWI_ILLEGAL_START_STOP: // Illegal START/STOP, abort and return error
    TWIInfo.errorCode = TWI_ILLEGAL_START_STOP;
    TWIInfo.mode = Ready;
    TWISendStop();
    break;
  }
}
/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
ISR(TWI_vect) { I2Cdev::interrupt(); }
