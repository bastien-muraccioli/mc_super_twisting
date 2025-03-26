#include "SuperTwisting.h"

#include <mc_control/GlobalPluginMacros.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_plugin
{

SuperTwisting::~SuperTwisting() = default;

void SuperTwisting::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
    auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

    auto & robot = ctl.robot(ctl.robots()[0].name());
    auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
    auto & rjo = robot.refJointOrder();
    
    dt_ = ctl.timestep();
    counter_ = 0.0;
    jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

    // Make sure to have obstacle detection
    if(!ctl.controller().datastore().has("Obstacle detected"))
    {
        ctl.controller().datastore().make<bool>("Obstacle detected", false);
    }
    
    auto plugin_config = config("super_twisting");

    referenceFrame = plugin_config("reference_frame", (std::string) "FT_sensor_wrench");
    torqueSensorName = plugin_config("torque_sensor_name", (std::string) "externalTorqueSensor");
    FTSensorName = plugin_config("ft_sensor_name", (std::string) "EEForceSensor");

    gamma2 = plugin_config("gamma2", 100.0);
    gamma1 = plugin_config("gamma1", 21.0);
    alpha2 = plugin_config("alpha2", 100.0);
    alpha1 = plugin_config("alpha1", 100.0);

    threshold_filtering = plugin_config("threshold_filtering", 0.05);
    threshold_offset = plugin_config("threshold_offset");
    if(threshold_offset.size() != jointNumber)
    {
        threshold_offset = Eigen::VectorXd::Constant(jointNumber, 10.0);
        mc_rtc::log::warning("[SuperTwisting] Threshold offset not set, using default value of 10.0");
    }
    lpf_threshold.setValues(threshold_offset, threshold_filtering, jointNumber);

    if(!robot.hasDevice<mc_rbdyn::ExternalTorqueSensor>(torqueSensorName))
    {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[SuperTwisting] No \"ExternalTorqueSensor\" with the name \"externalTorqueSensor\" found in "
            "the robot module, please add one to the robot's RobotModule.");
    }
    extTorqueSensor = &robot.device<mc_rbdyn::ExternalTorqueSensor>("externalTorqueSensor");

    ctl.setWrenches({{FTSensorName, sva::ForceVecd::Zero()}});

    tau_m.setZero(jointNumber);
    tau_ext_hat.setZero(jointNumber);
    tau_ext_hat_dot.setZero(jointNumber);
    inertiaMatrix.resize(jointNumber, jointNumber);
    tau_ext.setZero(jointNumber);

    Eigen::VectorXd qdot(jointNumber);
    for(size_t i = 0; i < jointNumber; i++)
    {
        qdot[i] = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
    }

    jac = rbd::Jacobian(robot.mb(), referenceFrame);
    jTranspose = jac.jacobian(robot.mb(), robot.mbc());
    jTranspose.transposeInPlace();
    coriolis = new rbd::Coriolis(robot.mb());
    forwardDynamics = rbd::ForwardDynamics(robot.mb());

    forwardDynamics.computeH(robot.mb(), robot.mbc());
    inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
    p = inertiaMatrix * qdot;
    p_hat = Eigen::VectorXd::Zero(jointNumber);
    p_error = p - p_hat;
    auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
    auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g
    gamma = tau_m + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm;

    externalForcesFT = Eigen::Vector6d::Zero();

    addGui(ctl);
    addLog(ctl);

    mc_rtc::log::info("SuperTwisting::init called with configuration:\n{}", config.dump(true, true));
}

void SuperTwisting::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("SuperTwisting::reset called");
}

void SuperTwisting::before(mc_control::MCGlobalController & controller)
{
    auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
    auto & robot = ctl.robot(ctl.robots()[0].name());
    auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
    counter_ += dt_;
    if(activate_plot_ && !plot_added_)
    {
        addPlot(controller);
        plot_added_ = true;
    }

    computeMomemtum(ctl);
    
    if(useFTSensor)
    {
        tau_ext = tau_ext_hat + jTranspose*externalForcesFT;
    }
    else
    {
        tau_ext = tau_ext_hat;
    }
    if(plugin_active)
    {
        extTorqueSensor->torques(tau_ext);
    }
    else
    {
        Eigen::VectorXd zero = Eigen::VectorXd::Zero(jointNumber);
        extTorqueSensor->torques(zero);
    }

    threshold_high = lpf_threshold.adaptiveThreshold(tau_ext, true);
    threshold_low = lpf_threshold.adaptiveThreshold(tau_ext, false);
    obstacle_detected_ = false;
    for (int i = 0; i < jointNumber; i++)
    {
        if (tau_ext[i] > threshold_high[i] || tau_ext[i] < threshold_low[i])
        {
            obstacle_detected_ = true;
            if(activate_verbose) mc_rtc::log::info("[SuperTwisting] Obstacle detected on joint {}", i);
            if (collision_stop_activated_) ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
            break;
        }
    }
    // mc_rtc::log::info("SuperTwisting::before");
}

void SuperTwisting::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("SuperTwisting::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration SuperTwisting::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void SuperTwisting::computeMomemtum(mc_control::MCGlobalController & controller)
{
    auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

    if(ctl.robot().encoderVelocities().empty())
    {
        return;
    }

    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

    auto & rjo = realRobot.refJointOrder();
    auto R = controller.robot().bodyPosW(referenceFrame).rotation();

    Eigen::VectorXd qdot(jointNumber);
    rbd::paramToVector(realRobot.alpha(), qdot);
    tau_m = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());
    // mc_rtc::log::info("[SuperTwisting] qdot: {}, tau_m: {}", qdot, tau_m);
    forwardDynamics.computeC(realRobot.mb(), realRobot.mbc());
    forwardDynamics.computeH(realRobot.mb(), realRobot.mbc());
    auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
    auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g
    jTranspose = jac.jacobian(realRobot.mb(), realRobot.mbc());
    jTranspose.transposeInPlace();
    auto sva_EF_FT = realRobot.forceSensor(FTSensorName).wrenchWithoutGravity(realRobot);
    Eigen::VectorXd sva_EF_FT_vec = sva_EF_FT.vector();
    externalForcesFT.head(3) = R.transpose() * sva_EF_FT_vec.head(3);
    externalForcesFT.tail(3) = R.transpose() * sva_EF_FT_vec.tail(3);
    gamma = tau_m + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm + jTranspose*externalForcesFT; //gamma = tau_m -g + C^T*qdot + J^T*F_ext
    inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
    // x_hat_dot = Gamma + γ1*sqrt(|x - x_hat|)*Sign(x - x_hat) + d_hat
    Eigen::VectorXd p_hat_dot = gamma + gamma1*(p_error).cwiseAbs().cwiseSqrt().cwiseProduct(Sign(p_error)) + alpha1*(p_error) + tau_ext_hat;
    p_hat += p_hat_dot*dt_;

    // d_hat_dot = γ2*Sign(x - x_hat)
    // tau_ext_hat_dot = (gamma_2+gamma_3*p_error.cwiseAbs())*Sign(p_error);
    tau_ext_hat_dot = (alpha2*Eigen::MatrixXd::Identity(jointNumber, jointNumber) + gamma2*p_error.cwiseAbs())*Sign(p_error);
    // integrate tau_ext_hat_dot to get tau_ext_hat
    tau_ext_hat += tau_ext_hat_dot*dt_;
    p = inertiaMatrix * qdot;
    p_error = p - p_hat;
}

double SuperTwisting::sign(double x)
{
    if (x>0) return 1;
    else if (x<0) return -1;
    else return 0;
}

Eigen::VectorXd SuperTwisting::Sign(Eigen::VectorXd x)
{
    Eigen::VectorXd y(x.size());
    for (int i=0; i<x.size(); i++)
    {
        y[i] = sign(x[i]);
    }
    return y;
}

void SuperTwisting::addPlot(mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    //Momentum
    gui.addPlot(
        "Momentum",
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "p(t)", [this]() { return p[jointShown]; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "p_hat(t)", [this]() { return p_hat[jointShown]; }, mc_rtc::gui::Color::Green)
        );
    //Momentum error
    gui.addPlot(
        "Momentum error",
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "p_error(t)", [this]() { return p_error[5]; }, mc_rtc::gui::Color::Red)
        );
    gui.addPlot(
        "Torque estimation",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("high_threshold(t)", [this]() { return threshold_high[jointShown]; }, mc_rtc::gui::Color::Gray),
        mc_rtc::gui::plot::Y("low_threshold(t)", [this]() { return threshold_low[jointShown]; }, mc_rtc::gui::Color::Gray),
        mc_rtc::gui::plot::Y("tau_ext_hat(t)", [this]() { return -tau_ext_hat[jointShown]; }, mc_rtc::gui::Color::Blue),
        mc_rtc::gui::plot::Y("tau_m(t)", [this]() { return tau_m[jointShown]; }, mc_rtc::gui::Color::Green),
        mc_rtc::gui::plot::Y("tau_ext(t)", [this]() { return -tau_ext[jointShown]; }, mc_rtc::gui::Color::Red)
    );
    gui.addPlot(
        "FT Sensor",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("FT Sensor norm", [this]() { return externalForcesFT.norm(); }, mc_rtc::gui::Color::Red)
    );
    gui.addPlot(
        "Gamma",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("gamma(t)", [this]() { return gamma[jointShown]; }, mc_rtc::gui::Color::Red)
    );
}

void SuperTwisting::addGui(mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::Checkbox(
            "Is estimation feedback active", plugin_active));

    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "jointShown", [this]() { return jointShown; },
            [this](int joint)
            {
                this->jointShown = joint;
            }),
        mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }),
        // Add checkbox to activate the collision stop
        mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
        mc_rtc::gui::Checkbox("Verbose", activate_verbose), 
        // Add Threshold offset input
        mc_rtc::gui::ArrayInput("Threshold offset", {"q_0", "q_1", "q_2", "q_3", "q_4", "q_5", "q_6"}, 
            [this](){return this->threshold_offset;},
            [this](const Eigen::VectorXd & offset)
            { 
            threshold_offset = offset;
            lpf_threshold.setOffset(threshold_offset); 
            }),
        // Add Threshold filtering input
        mc_rtc::gui::NumberInput("Threshold filtering", [this](){return this->threshold_filtering;},
            [this](double filtering)
            { 
            threshold_filtering = filtering;
            lpf_threshold.setFiltering(threshold_filtering); 
            })                                               
        );
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::Checkbox("Use FT Sensor", useFTSensor));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "gamma2", [this]() { return gamma2; },
            [this](double gain)
            {
                this->gamma2 = gain;
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "gamma1", [this]() { return gamma1; },
            [this](double gain)
            {
                this->gamma1 = gain;
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "alpha2", [this]() { return alpha2; },
            [this](double gain)
            {
                this->alpha2 = gain;
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "alpha1", [this]() { return alpha1; },
            [this](double gain)
            {
                this->alpha1 = gain;
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
}

void SuperTwisting::addLog(mc_control::MCGlobalController & ctl)
{
    ctl.controller().logger().addLogEntry("SuperTwisting_p", [this]() { return p; });
    ctl.controller().logger().addLogEntry("SuperTwisting_p_hat", [this]() { return p_hat; });
    ctl.controller().logger().addLogEntry("SuperTwisting_p_error", [this]() { return p_error; });
    ctl.controller().logger().addLogEntry("SuperTwisting_tau_ext_hat", [this]() { return tau_ext_hat; });
    ctl.controller().logger().addLogEntry("SuperTwisting_tau_ext_hat_dot", [this]() { return tau_ext_hat_dot; });
    ctl.controller().logger().addLogEntry("SuperTwisting_gamma", [this]() { return gamma; });
    ctl.controller().logger().addLogEntry("SuperTwisting_tau_ext", [this]() { return tau_ext; });
    ctl.controller().logger().addLogEntry("SuperTwisting_threshold_high", [this]() { return threshold_high; });
    ctl.controller().logger().addLogEntry("SuperTwisting_threshold_low", [this]() { return threshold_low; });
    ctl.controller().logger().addLogEntry("SuperTwisting_obstacle_detected", [this]() { return obstacle_detected_; });
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("SuperTwisting", mc_plugin::SuperTwisting)
