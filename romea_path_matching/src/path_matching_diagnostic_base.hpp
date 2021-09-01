#ifndef __DiagnosticPathMatchingBase_HPP__
#define __DiagnosticPathMatchingBase_HPP__

//romea
#include <romea_common/diagnostic/CheckupRate.hpp>

namespace romea{

class PathMatchingDiagnosticBase
{
public:

  PathMatchingDiagnosticBase();

  void updateOdomRate(const romea::Duration & duration);
  void updateMatchingStatus(const bool & status);

  virtual DiagnosticReport getReport() const =0;

protected :

  CheckupRate odom_rate_diagnostic_;
  DiagnosticReport matching_report_;
};


}

#endif
