#include "path_matching_diagnostic.hpp"

namespace romea {

//-----------------------------------------------------------------------------
DiagnosticMatchingStatus::DiagnosticMatchingStatus(const std::string &name):
  DiagnosticTask(name),
  status_(false)
{
}

//-----------------------------------------------------------------------------
void DiagnosticMatchingStatus::setStatus(const bool &status)
{
  status_=status;
}

//-----------------------------------------------------------------------------
bool DiagnosticMatchingStatus::getStatus() const
{
  return status_;
}

//-----------------------------------------------------------------------------
void DiagnosticMatchingStatus::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if(status_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Matching sucessed");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Matching failed");
  }
  stat.add(" matching ", status_);
}


//-----------------------------------------------------------------------------
DiagnosticPathStatus::DiagnosticPathStatus(const std::string &name):
  DiagnosticTask(name),
  filename_(),
  status_(false)

{
}

//-----------------------------------------------------------------------------
void DiagnosticPathStatus::setPathFilename(const std::string & filename)
{
  filename_ = filename;
}

//-----------------------------------------------------------------------------
std::string DiagnosticPathStatus::getPathFilename()
{
  return filename_;
}

//-----------------------------------------------------------------------------
void DiagnosticPathStatus::setStatus(const bool &status)
{
  status_ =status;
}

//-----------------------------------------------------------------------------
bool DiagnosticPathStatus::getStatus() const
{
  return status_;
}

//-----------------------------------------------------------------------------
void DiagnosticPathStatus::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if(status_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Path is loaded");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unable to load path");
  }
  stat.add(" filename " , filename_ );
  stat.add(" is opened ", status_);
}


//-----------------------------------------------------------------------------
DiagnosticLookupTransformStatus::DiagnosticLookupTransformStatus(const std::string &name):
  DiagnosticTask(name),
  status_(false)
{

}

//-----------------------------------------------------------------------------
void DiagnosticLookupTransformStatus::setStatus(const bool &status)
{
  status_=status;
}

//-----------------------------------------------------------------------------
bool DiagnosticLookupTransformStatus::getStatus() const
{
  return status_;
}

//-----------------------------------------------------------------------------
void DiagnosticLookupTransformStatus::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

  if(!status_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Transformation world to path published");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Transformation world to path not published ");
  }
  stat.add(" tf status ", status_);
}


//-----------------------------------------------------------------------------
PathMatchingDiagnostic::PathMatchingDiagnostic():
  odom_rate_monitoring_(),
  odom_rate_diagnostic_("odom_rate",0),
  matching_status_diagnostic_("matching_status"),
  lookup_transform_status_diagnostic_("lookup_tf_status"),
  path_status_diagnostic_("path_status"),
  composite_diagnostic_("_path_matching"),
  diagnostics_updater_()
{
  odom_rate_monitoring_.initialize(10);
  composite_diagnostic_.addTask(&odom_rate_diagnostic_);
  composite_diagnostic_.addTask(&path_status_diagnostic_);
  composite_diagnostic_.addTask(&lookup_transform_status_diagnostic_);
  composite_diagnostic_.addTask(&matching_status_diagnostic_);
  diagnostics_updater_.add(composite_diagnostic_);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::updateOdomRate(const romea::Duration & duration)
{
  odom_rate_monitoring_.update(duration);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::updateLookupTransformStatus(const bool & status)
{
  lookup_transform_status_diagnostic_.setStatus(status);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::updateMatchingStatus(const bool & status)
{
  matching_status_diagnostic_.setStatus(status);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::updatePathStatus(const std::string & filename, const bool & isOpened)
{
 path_status_diagnostic_.setPathFilename(filename);
 path_status_diagnostic_.setStatus(isOpened);
}


//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::publish()
{
  odom_rate_diagnostic_.update(odom_rate_monitoring_.getRate());
  diagnostics_updater_.update();
}

}
