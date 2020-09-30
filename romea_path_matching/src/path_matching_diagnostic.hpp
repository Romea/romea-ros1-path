#ifndef __DiagnosticPathMatching_HPP__
#define __DiagnosticPathMatching_HPP__

//romea
#include <romea_common/diagnostic/CheckupRate.hpp>

namespace romea{

class PathMatchingDiagnostic
{
public:

  PathMatchingDiagnostic();

  void setPathFilename(const std::string & filename);
  void updateOdomRate(const romea::Duration & duration);
  void updateLookupTransformStatus(const bool & status);
  void updateMatchingStatus(const bool & status);
  void updatePathStatus(const bool & isOpened);
  DiagnosticReport getReport()const;

private :

  CheckupRate odom_rate_diagnostic_;
  DiagnosticReport matching_report_;
};

}

#endif
