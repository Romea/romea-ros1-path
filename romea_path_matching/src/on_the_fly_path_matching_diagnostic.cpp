#include "on_the_fly_path_matching_diagnostic.hpp"

namespace romea {

//-----------------------------------------------------------------------------
OnTheFlyPathMatchingDiagnostic::OnTheFlyPathMatchingDiagnostic():
  PathMatchingDiagnosticBase (),
  leader_odom_rate_diagnostic_("leader_odom",0)
{
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingDiagnostic::updateLeaderOdomRate(const romea::Duration & duration)
{
  matching_report_.diagnostics.clear();
  leader_odom_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
DiagnosticReport OnTheFlyPathMatchingDiagnostic::getReport() const
{
  DiagnosticReport report;
  report += odom_rate_diagnostic_.getReport();
  report += leader_odom_rate_diagnostic_.getReport();
  report += matching_report_;
  return report;
}


}
