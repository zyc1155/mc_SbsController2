#include "SbsControllerZyc.h"

SbsControllerZyc::SbsControllerZyc(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("SbsControllerZyc init done ");
}

bool SbsControllerZyc::run()
{
  return mc_control::MCController::run();
}

void SbsControllerZyc::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("SbsControllerZyc", SbsControllerZyc)
