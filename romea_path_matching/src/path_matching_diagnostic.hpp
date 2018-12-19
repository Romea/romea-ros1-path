#ifndef __DiagnosticPathMatching_HPP__
#define __DiagnosticPathMatching_HPP__

//ros
#include <diagnostic_updater/diagnostic_updater.h>

////std
//#include <atomic>

//romea
#include <monitoring/RateMonitoring.hpp>
#include <ros/diagnostics/DiagnosticGreaterThan.hpp>

class DiagnosticMatchingStatus : public diagnostic_updater::DiagnosticTask
{
public:

  DiagnosticMatchingStatus(const std::string &name);

  void setStatus(const bool &status);

  bool getStatus() const;

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private :
  bool status_;
};

class DiagnosticLookupTransformStatus : public diagnostic_updater::DiagnosticTask
{
public:

  DiagnosticLookupTransformStatus(const std::string &name);

  void setStatus(const bool &status);

  bool getStatus() const;

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private :
  bool status_;
};

class DiagnosticPathStatus : public diagnostic_updater::DiagnosticTask
{
public:

  DiagnosticPathStatus(const std::string &name);

  void setPathFilename(const std::string & filename);

  std::string getPathFilename();

  void setStatus(const bool &status);

  bool getStatus() const;

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private :

  std::string filename_;
  bool status_;
};


class PathMatchingDiagnostic
{

public:

  PathMatchingDiagnostic();

  void updateOdomRate(const romea::Duration & duration);
  void updateLookupTransformStatus(const bool & status);
  void updateMatchingStatus(const bool & status);
  void updatePathStatus(const std::string & filename, const bool & isOpened);
  void publish();

private :

  romea::RateMonitoring odom_rate_monitoring_;
  romea::DiagnosticGreaterThan<double> odom_rate_diagnostic_;
  DiagnosticMatchingStatus matching_status_diagnostic_;
  DiagnosticLookupTransformStatus lookup_transform_status_diagnostic_;
  DiagnosticPathStatus path_status_diagnostic_;

  diagnostic_updater::CompositeDiagnosticTask composite_diagnostic_;
  diagnostic_updater::Updater diagnostics_updater_;

};

#endif
