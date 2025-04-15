#include <sensor/imu/imu_bmi270.hpp>

void ImuBMI270::initialize()
{
  int8_t st;
  uint8_t data = 0;

  USBSerial.printf("Start IMU Initialize!\n\r");

  // BMI270 Init
  bmi270_dev_init();
  USBSerial.printf("#BMI270 dev init\n\r");
  st = bmi270_init(pBmi270_);
  USBSerial.printf("#INIT Status:%d\n\r", st);
  if (st != 0) {
    USBSerial.printf("BMI270 INIT Fail!\n\r");
    while (1);
  }
  USBSerial.printf("#Chip ID DEV:%02X\n\r", Bmi270_.chip_id);
  USBSerial.printf("#APP_STATUS:%02X\n\r", Bmi270_.aps_status);

  USBSerial.printf("#INIT_STATUS Read:%d\n\r", bmi2_get_regs(0x21, &data, 1, pBmi270_));
  USBSerial.printf("#INIT_STATUS:%02X\n\r", data);
  // IMU Config
  USBSerial.printf("#Config Status:%d\n\r", set_accel_gyro_config(pBmi270_));
  uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
  USBSerial.printf("#Sensor enable Status:%d\n\r", bmi2_sensor_enable(sensor_list, 2, pBmi270_));
}

void ImuBMI270::readImuData()
{
  imu_update();
  acc_data_(0) = imu_get_acc_y();
  acc_data_(1) = -imu_get_acc_x();
  acc_data_(2) = imu_get_acc_z();
  gyro_data_(0) = imu_get_gyro_y();
  gyro_data_(1) = -imu_get_gyro_x();
  gyro_data_(2) = imu_get_gyro_z();
}

void ImuBMI270::bmi270_dev_init(void)
{
  memset(&Bmi270_, 0, sizeof(Bmi270_));  // clear the struct. important
  Bmi270_.intf = BMI2_SPI_INTF;
  //Bmi270.chip_id = 0x24;
  Bmi270_.read =  bmi2_spi_read;
  Bmi270_.write = bmi2_spi_write;
  Bmi270_.delay_us = bmi2_delay_us;
  Bmi270_.dummy_byte = 1;
  Bmi270_.gyro_en = 1;
}

void ImuBMI270::bmi2_delay_us(uint32_t period, void *intf_ptr)
{
  //coines_delay_usec(period);
  ets_delay_us(period);
}

int8_t ImuBMI270::set_accel_gyro_config(struct bmi2_dev *bmi)
{
  /* Status of api are returned to this variable. */
  int8_t rslt;

  /* Structure to define accelerometer and gyro configuration. */
  struct bmi2_sens_config config[2];

  /* Configure the type of feature. */
  config[ACCEL].type = BMI2_ACCEL;
  config[GYRO].type = BMI2_GYRO;

  /* Get default configurations for the type of feature selected. */
  rslt = bmi2_get_sensor_config(config, 2, bmi);
  bmi2_error_codes_print_result(rslt);

  /* Map data ready interrupt to interrupt pin. */
  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
  bmi2_error_codes_print_result(rslt);

  if (rslt == BMI2_OK)
    {
      /* NOTE: The user can change the following configuration parameters according to their requirement. */
      /* Set Output Data Rate */
      config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_400HZ;

      /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
      config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_8G;

      /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
       * if it is set to 2, then 2^(bandwidth parameter) samples
       * are averaged, resulting in 4 averaged samples.
       * Note1 : For more information, refer the datasheet.
       * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
       * this has an adverse effect on the power consumed.
       */
      config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

      /* Enable the filter performance mode where averaging of samples
       * will be done based on above set bandwidth and ODR.
       * There are two modes
       *  0 -> Ultra low power mode
       *  1 -> High performance mode(Default)
       * For more info refer datasheet.
       */
      config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

      /* The user can change the following configuration parameters according to their requirement. */
      /* Set Output Data Rate */
      config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;

      /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
      config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;//BMI2_GYR_RANGE_2000

      /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
      config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;// BMI2_GYR_OSR4_MODE;

      /* Enable/Disable the noise performance mode for precision yaw rate sensing
       * There are two modes
       *  0 -> Ultra low power mode(Default)
       *  1 -> High performance mode
       */
      config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

      /* Enable/Disable the filter performance mode where averaging of samples
       * will be done based on above set bandwidth and ODR.
       * There are two modes
       *  0 -> Ultra low power mode
       *  1 -> High performance mode(Default)
       */
      config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

      /* Set the accel and gyro configurations. */
      rslt = bmi2_set_sensor_config(config, 2, bmi);
      bmi2_error_codes_print_result(rslt);
    }

  return rslt;
}

void ImuBMI270::bmi2_error_codes_print_result(int8_t rslt)
{
  switch (rslt)
    {
    case BMI2_OK:

      /* Do nothing */
      break;

    case BMI2_W_FIFO_EMPTY:
      printf("Warning [%d] : FIFO empty\r\n", rslt);
      break;
    case BMI2_W_PARTIAL_READ:
      printf("Warning [%d] : FIFO partial read\r\n", rslt);
      break;
    case BMI2_E_NULL_PTR:
      printf(
             "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
             rslt);
      break;

    case BMI2_E_COM_FAIL:
      printf(
             "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
             rslt);
      break;

    case BMI2_E_DEV_NOT_FOUND:
      printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
             rslt);
      break;

    case BMI2_E_INVALID_SENSOR:
      printf(
             "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
             rslt);
      break;

    case BMI2_E_SELF_TEST_FAIL:
      printf(
             "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
             rslt);
      break;

    case BMI2_E_INVALID_INT_PIN:
      printf(
             "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
             rslt);
      break;

    case BMI2_E_OUT_OF_RANGE:
      printf(
             "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
             rslt);
      break;

    case BMI2_E_ACC_INVALID_CFG:
      printf(
             "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
             rslt);
      break;

    case BMI2_E_GYRO_INVALID_CFG:
      printf(
             "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
             rslt);
      break;

    case BMI2_E_ACC_GYR_INVALID_CFG:
      printf(
             "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
             rslt);
      break;

    case BMI2_E_CONFIG_LOAD:
      printf(
             "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
             rslt);
      break;

    case BMI2_E_INVALID_PAGE:
      printf(
             "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
             rslt);
      break;

    case BMI2_E_SET_APS_FAIL:
      printf(
             "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
             rslt);
      break;

    case BMI2_E_AUX_INVALID_CFG:
      printf(
             "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
             rslt);
      break;

    case BMI2_E_AUX_BUSY:
      printf(
             "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
             rslt);
      break;

    case BMI2_E_REMAP_ERROR:
      printf(
             "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
             rslt);
      break;

    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      printf(
             "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
             rslt);
      break;

    case BMI2_E_SELF_TEST_NOT_DONE:
      printf(
             "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
             rslt);
      break;

    case BMI2_E_INVALID_INPUT:
      printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
      break;

    case BMI2_E_INVALID_STATUS:
      printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
      break;

    case BMI2_E_CRT_ERROR:
      printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
      break;

    case BMI2_E_ST_ALREADY_RUNNING:
      printf(
             "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
             rslt);
      break;

    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      printf(
             "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
             rslt);
      break;

    case BMI2_E_DL_ERROR:
      printf(
             "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
             rslt);
      break;

    case BMI2_E_PRECON_ERROR:
      printf(
             "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
             rslt);
      break;

    case BMI2_E_ABORT_ERROR:
      printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
      break;

    case BMI2_E_WRITE_CYCLE_ONGOING:
      printf(
             "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
             rslt);
      break;

    case BMI2_E_ST_NOT_RUNING:
      printf(
             "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
             rslt);
      break;

    case BMI2_E_DATA_RDY_INT_FAILED:
      printf(
             "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
             rslt);
      break;

    case BMI2_E_INVALID_FOC_POSITION:
      printf(
             "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
             rslt);
      break;

    default:
      printf("Error [%d] : Unknown error code\r\n", rslt);
      break;
    }
}


