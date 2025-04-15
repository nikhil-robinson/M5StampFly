#include "driver/i2c.h"
#include "i2c.hpp"

i2c_port_t _i2c_port = 1;
i2c_config_t i2c_conf;
const size_t bufferSize=256;
uint8_t rxBuffer[bufferSize];
size_t rxIndex;
size_t rxLength;

uint8_t txBuffer[bufferSize];
size_t txLength;
uint16_t txAddress;
uint32_t _timeOutMillis;
bool nonStop;


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t i2c_read(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(_i2c_port, device_address, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t i2c_write_byte(uint8_t device_address, uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(_i2c_port, device_address, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}


/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
  i2c_conf.mode = I2C_MODE_MASTER;
  i2c_conf.sda_io_num = I2C_MASTER_SDA_IO;         // select GPIO specific to your project
  i2c_conf.scl_io_num = I2C_MASTER_SCL_IO;         // select GPIO specific to your project
  i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(_i2c_port, &i2c_conf);
  return i2c_driver_install(_i2c_port, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// TODO
void i2c_beginTransmission(uint16_t address)
{
    nonStop = false;
    txAddress = address;
    txLength = 0;
}

/*
https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
endTransmission() returns:
0: success.
1: data too long to fit in transmit buffer.
2: received NACK on transmit of address.
3: received NACK on transmit of data.
4: other error.
5: timeout
*/

// TODO
uint8_t i2c_endTransmission(void)
{
    esp_err_t err = ESP_OK;
    err = i2cWrite(_i2c_port, txAddress, txBuffer, txLength, _timeOutMillis);
    switch(err){
        case ESP_OK: return 0;
        case ESP_FAIL: return 2;
        case ESP_ERR_TIMEOUT: return 5;
        default: break;
    }
    return 4;
}

uint8_t i2c_scan(void)
{
  USBSerial.println ("I2C scanner. Scanning ...");
  delay(50);
  byte count = 0;
  for (uint8_t i = 1; i < 127; i++)
  {
    // i2c_beginTransmission(i);
    Wire1.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire1.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    // if(i2c_endTransmission() == 0)
      {
        USBSerial.print ("Found address: ");
        USBSerial.print (i, DEC);
        USBSerial.print (" (0x");
        USBSerial.print (i, HEX); 
        USBSerial.println (")");
        count++;
      }
  }
  USBSerial.print ("Found ");      
  USBSerial.print (count, DEC);        // numbers of devices
  USBSerial.println (" device(s).");
  return count;
}

