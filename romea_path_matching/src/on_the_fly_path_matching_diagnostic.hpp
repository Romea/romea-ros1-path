#ifndef __DiagnosticOnTheFlyPathMatching_HPP__
#define __DiagnosticOnTheFlyPathMatching_HPP__

//romea
#include "path_matching_diagnostic_base.hpp"

namespace romea{

class OnTheFlyPathMatchingDiagnostic : public PathMatchingDiagnosticBase
{
public:

  OnTheFlyPathMatchingDiagnostic();

  virtual ~OnTheFlyPathMatchingDiagnostic()=default;

  void updateLeaderOdomRate(const romea::Duration & duration);

  virtual DiagnosticReport getReport()const override;


private :

   CheckupRate leader_odom_rate_diagnostic_;
};

}

#endif

