#include "path_matching_diagnostic_base.hpp"

namespace
{

std::string booleanToString(const bool & flag)
{
  return flag ? "true" : "false";
}

}

namespace romea {

//-----------------------------------------------------------------------------
PathMatchingDiagnosticBase::PathMatchingDiagnosticBase():
  odom_rate_diagnostic_("odom",0),
  matching_report_()
{
  updateMatchingStatus(false);
}


//-----------------------------------------------------------------------------
void PathMatchingDiagnosticBase::updateOdomRate(const romea::Duration & duration)
{
  matching_report_.diagnostics.clear();
  odom_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnosticBase::updateMatchingStatus(const bool & status)
{
  if(status)
  {
    matching_report_.diagnostics.push_back(Diagnostic(DiagnosticStatus::OK, "Matching sucessed"));
  }
  else
  {
    matching_report_.diagnostics.push_back(Diagnostic(DiagnosticStatus::ERROR, "Matching failed"));
  }
  matching_report_.info[" matching "] = booleanToString(status);
}

}

