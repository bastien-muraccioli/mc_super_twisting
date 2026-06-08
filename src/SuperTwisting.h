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

  void computeThirdOrder(mc_control::MCGlobalController & controller);

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
  bool activate_verbose = false;

  bool obstacle_detected_secondOrder_ = false;
  LpfThreshold lpf_threshold_secondOrder_;
  Eigen::VectorXd threshold_offset_secondOrder_;
  double threshold_filtering_secondOrder_;
  Eigen::VectorXd threshold_high_secondOrder_;
  Eigen::VectorXd threshold_low_secondOrder_;
  
  bool obstacle_detected_thirdOrder_ = false;
  LpfThreshold lpf_threshold_thirdOrder_;
  Eigen::VectorXd threshold_offset_thirdOrder_;
  double threshold_filtering_thirdOrder_;
  Eigen::VectorXd threshold_high_thirdOrder_;
  Eigen::VectorXd threshold_low_thirdOrder_;

  bool obstacle_detected_thirdOrder_tau_ext_dot_ = false;
  LpfThreshold lpf_threshold_thirdOrder_tau_ext_dot_;
  Eigen::VectorXd threshold_offset_thirdOrder_tau_ext_dot_;
  double threshold_filtering_thirdOrder_tau_ext_dot_;
  Eigen::VectorXd threshold_high_thirdOrder_tau_ext_dot_;
  Eigen::VectorXd threshold_low_thirdOrder_tau_ext_dot_;

  std::string referenceFrame;
  std::string torqueSensorName;
  std::string FTSensorName;
  bool useFTSensor = false;
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
  Eigen::VectorXd tau_ext_ft_sensor; // GroundTruth
  Eigen::VectorXd tau_ext_hat; //Estimated external torque without the FT_sensor
  Eigen::VectorXd tau_ext_hat_ft_sensor; //Estimated external torque with the FT_sensor
  Eigen::VectorXd tau_ext_hat_dot; //Estimated external torque derivative without the FT_sensor
  Eigen::VectorXd tau_ext; //Estimated external torque of all the system
  double gamma2 = 100.0;
  double gamma1 = 21;
  double alpha2 = 100; //100 (adaptive)
  double alpha1 = 100; //100 (adaptive)

  Eigen::Vector6d externalForcesFT;

  // Third order
  bool third_order = false;
  double c = 100.0; // Maximum value of the third order term (derivative of the external torque)
  double gamma3_third_order; // 1.1*c
  double gamma2_third_order; // (9/2)*c^(5/6)
  double gamma1_third_order; //3*c^(1/3)
  Eigen::VectorXd p_hat_third_order; //Estimated momentum
  Eigen::VectorXd p_error_third_order; //Momentum error
  Eigen::VectorXd tau_ext_hat_third_order;  //Estimated external torque without the FT_sensor
  Eigen::VectorXd tau_ext_hat_ft_sensor_third_order; //Estimated external torque with the FT_sensor
  Eigen::VectorXd tau_ext_hat_dot_third_order; // The derivative of the estimated external torque
  Eigen::VectorXd tau_ext_dot_hat_third_order;  // The estimated external torque derivative

};

} // namespace mc_plugin
