#include "SuperTwisting.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

SuperTwisting::~SuperTwisting() = default;

void SuperTwisting::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("SuperTwisting::init called with configuration:\n{}", config.dump(true, true));
}

void SuperTwisting::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("SuperTwisting::reset called");
}

void SuperTwisting::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("SuperTwisting::before");
}

void SuperTwisting::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("SuperTwisting::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration SuperTwisting::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("SuperTwisting", mc_plugin::SuperTwisting)
