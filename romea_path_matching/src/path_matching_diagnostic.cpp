#include "path_matching_diagnostic.hpp"

namespace
{

std::string booleanToString(const bool & flag)
{
  return flag ? "true" : "false";
}

}

namespace romea {

//-----------------------------------------------------------------------------
PathMatchingDiagnostic::PathMatchingDiagnostic():
  PathMatchingDiagnosticBase ()
{
  setPathFilename("");
  updatePathStatus(false);
  updateLookupTransformStatus(false);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::setPathFilename(const std::string & filename)
{
  matching_report_.info[" filename "] =filename;
}


//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::updateLookupTransformStatus(const bool & status)
{
  if(!status)
  {
    matching_report_.diagnostics.push_back(Diagnostic(DiagnosticStatus::ERROR, "Transformation world to path not published"));
  }
  else
  {
    matching_report_.diagnostics.push_back(Diagnostic(DiagnosticStatus::OK,"Transformation world to path not published "));
  }
  matching_report_.info[" tf status "]= booleanToString(status);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::updatePathStatus(const bool & isOpened)
{
  if(isOpened)
  {
    matching_report_.diagnostics.push_back(Diagnostic(DiagnosticStatus::OK, "Path is loaded"));
  }
  else
  {
    matching_report_.diagnostics.push_back(Diagnostic(DiagnosticStatus::ERROR, "Unable to load path"));
  }
  matching_report_.info[" is opened "] = booleanToString(isOpened);
}


//-----------------------------------------------------------------------------
DiagnosticReport PathMatchingDiagnostic::getReport() const
{
  DiagnosticReport report;
  report += odom_rate_diagnostic_.getReport();
  report += matching_report_;
  return report;
}

}

