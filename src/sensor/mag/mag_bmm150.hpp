#pragma once

#include <bmm150.h>
#include <sensor/mag/mag.hpp>
#include <memory>

class MagnetmeterBMM150 : public Magnetmeter
{
public:
  MagnetmeterBMM150() {};
  ~MagnetmeterBMM150() = default;
  void initialize() override;

private:
  void readMagData() override;

  std::shared_ptr<BMM150> driver_;
};

