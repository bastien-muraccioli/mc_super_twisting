/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "LpfThreshold.h"

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

#include <mc_rbdyn/ExternalTorqueSensor.h>

namespace mc_plugin
{

struct SuperTwisting : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  double sign(double x);
  Eigen::VectorXd Sign(Eigen::VectorXd x);
  void computeMomemtum(mc_control::MCGlobalController & controller);

  void addPlot(mc_control::MCGlobalController & ctl);
  void addLog(mc_control::MCGlobalController & ctl);
  void addGui(mc_control::MCGlobalController & ctl);

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~SuperTwisting() override;

private:

  // GUI
  double dt_;
  double counter_;

  int jointNumber;
  int jointShown = 0;
  bool activate_plot_ = false;
  bool plot_added_ = false;
  bool collision_stop_activated_ = false;
  bool obstacle_detected_ = false;
  bool activate_verbose = false;

  LpfThreshold lpf_threshold;
  Eigen::VectorXd threshold_offset;
  double threshold_filtering;
  Eigen::VectorXd threshold_high;
  Eigen::VectorXd threshold_low;

  std::string referenceFrame;
  std::string torqueSensorName;
  std::string FTSensorName;
  bool useFTSensor = true;
  bool plugin_active = false;

  rbd::Jacobian jac;
  rbd::Coriolis * coriolis;
  rbd::ForwardDynamics forwardDynamics;
  Eigen::VectorXd gamma;
  Eigen::MatrixXd inertiaMatrix;
  Eigen::MatrixXd jTranspose;
  Eigen::VectorXd p; //momentum
  Eigen::VectorXd p_hat; //Estimated momentum
  Eigen::VectorXd p_error; //Momentum error
  Eigen::VectorXd tau_m;
  Eigen::VectorXd tau_ext_hat; //Estimated external torque without the FT_sensor
  Eigen::VectorXd tau_ext_hat_dot; //Estimated external torque derivative without the FT_sensor
  Eigen::VectorXd tau_ext; //Estimated external torque of all the system
  double gamma2 = 100.0;
  double gamma1 = 21;
  double alpha2 = 100; //100 (adaptive)
  double alpha1 = 100; //100 (adaptive)

  Eigen::Vector6d externalForcesFT;
  mc_rbdyn::ExternalTorqueSensor * extTorqueSensor;
};

} // namespace mc_plugin
