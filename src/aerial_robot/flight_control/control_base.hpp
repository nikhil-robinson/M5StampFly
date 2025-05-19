#pragma once

#include <aerial_robot/state_estimation/state_estimator.hpp>
#include <aerial_robot/navigation/navigation_base.hpp>
#include <memory>

class ControlBase {
   public:
    ControlBase(std::shared_ptr<Odometry> odom, std::shared_ptr<BaseNavigator> navigator)
        : odom_(odom), navigator_(navigator) {
    }

    virtual ~ControlBase() = default;

    virtual void update() = 0;

   protected:
    std::shared_ptr<Odometry> odom_;
    std::shared_ptr<BaseNavigator> navigator_;
};
