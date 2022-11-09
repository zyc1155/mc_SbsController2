#pragma once

#include <mc_control/mc_controller.h>


#include "api.h"

struct SbsControllerZyc_DLLAPI SbsControllerZyc : public mc_control::MCController
{
    SbsControllerZyc(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
};