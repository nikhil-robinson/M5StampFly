#include <sensor/mag/mag_bmm150.hpp>

void MagnetmeterBMM150::initialize()
{
  Magnetmeter::initialize();

  driver_ = std::make_shared<BMM150>();
  if(driver_->initialize() == BMM150_E_ID_NOT_CONFORM)
    {
      USBSerial.printf("#BMM150 Chip ID can not read!\n\r");
      while (1);
    }
  else
    USBSerial.printf("#BMM150 Initialize done!\n\r");
  // driver_->set_op_mode(BMM150_FORCED_MODE);
  driver_->set_op_mode(BMM150_NORMAL_MODE);
}

void MagnetmeterBMM150::readMagData()
{
  bmm150_mag_data magvalue;
  // driver_->set_op_mode(BMM150_FORCED_MODE);
  driver_->read_mag_data();
  raw_mag_data_(0) =  driver_->raw_mag_data.raw_datax;
  raw_mag_data_(1) =  driver_->raw_mag_data.raw_datay;
  raw_mag_data_(2) =  driver_->raw_mag_data.raw_dataz;
}

