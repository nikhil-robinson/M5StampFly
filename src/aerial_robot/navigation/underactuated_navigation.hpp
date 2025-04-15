#pragma once

#include <aerial_robot/navigation/navigation_base.hpp>
#include <communication/rc.hpp>
#include <config.h>

class UnderActuatedNavigator : public BaseNavigator
{
public:
  UnderActuatedNavigator(std::shared_ptr<Odometry> odom) : BaseNavigator(odom) {}

  ~UnderActuatedNavigator() = default;

  void initialize() override;
  void update() override;
};
