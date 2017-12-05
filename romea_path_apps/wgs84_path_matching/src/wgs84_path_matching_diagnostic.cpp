//#include "robot_to_human_localisation_diagnostic.hpp"


////-----------------------------------------------------------------------------
//DiagnosticLocalisationStatus::DiagnosticLocalisationStatus(const std::string &name):
//  DiagnosticTask(name),
//  status_(WAIT_FOR_PROPRIOCEPTIVE_DATA)
//{

//}

////-----------------------------------------------------------------------------
//void DiagnosticLocalisationStatus::setLocalisationStatus(const LocalisationStatus &status)
//{
//  status_=status;
//}

////-----------------------------------------------------------------------------
//LocalisationStatus DiagnosticLocalisationStatus::getLocalisationStatus()const
//{
//  return status_;
//}


////-----------------------------------------------------------------------------
//void DiagnosticLocalisationStatus::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
//{

//  if(status_ == WAIT_FOR_PROPRIOCEPTIVE_DATA)
//  {
//    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "WAIT_FOR_PROPRIOCEPTIVE_DATA");
//  }
//  else if(status_ == WAIT_FOR_RANGE_DATA)
//  {
//    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "WAIT_FOR_RANGE_DATA");
//  }
//  else
//  {
//    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "RUNNING");
//  }
//}


////-----------------------------------------------------------------------------
//R2HLocalisationDiagnostic::R2HLocalisationDiagnostic():
//  minimal_linear_speed_rate_(std::numeric_limits<double>::max()),
//  minimal_angular_speed_rate_(std::numeric_limits<double>::max()),
//  minimal_follower_tag_rate_(std::numeric_limits<double>::max()),
//  maximal_follower_tag_rate_(0),
//  linear_speed_rate_monitoring_(),
//  angular_speed_rate_monitoring_(),
//  follower_tag_rate_monitorings_(),
//  localisation_statuc_diagnostic_("status"),
//  linear_speed_rate_diagnostic_("linear_speed_rate"),
//  angular_speed_rate_diagnostic_("angular_speed_rate"),
//  follower_tag_rate_diagnotics_(),
//  composite_diagnostic_("r2hl"),
//  diagnostics_updater_()
//{
//  composite_diagnostic_.addTask(&angular_speed_rate_diagnostic_);
//  composite_diagnostic_.addTask(&linear_speed_rate_diagnostic_);
//  composite_diagnostic_.addTask(&localisation_statuc_diagnostic_);

//  diagnostics_updater_.add(composite_diagnostic_);
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::addFollowerTag(const unsigned long long & tagId)
//{
//  std::string diagnostic_name = std::string("/r2hl/tag")+std::to_string(tagId);
//  follower_tag_rate_monitorings_.emplace_back(romea::RateMonitoring(minimal_follower_tag_rate_));
//  follower_tag_rate_diagnotics_.emplace_back(romea::DiagnosticInsideRangeRate(diagnostic_name,minimal_follower_tag_rate_,maximal_follower_tag_rate_));
//  diagnostics_updater_.add(follower_tag_rate_diagnotics_.back());
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::setMinimalLinearSpeedRate(const double & minimal_linear_speed_rate)
//{
//  minimal_linear_speed_rate_ = minimal_linear_speed_rate;
//  linear_speed_rate_monitoring_.initialize(minimal_linear_speed_rate_);
//  linear_speed_rate_diagnostic_.setMinimalRate(minimal_linear_speed_rate_);
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::setMinimalAngularSpeedRate(const double & minimal_angular_speed_rate)
//{
//  minimal_angular_speed_rate_ = minimal_angular_speed_rate;
//  angular_speed_rate_monitoring_.initialize(minimal_angular_speed_rate_);
//  angular_speed_rate_diagnostic_.setMinimalRate(minimal_angular_speed_rate_);
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::setMinimalFollowerTagRate(const double & minimal_follower_tag_rate)
//{
//  minimal_follower_tag_rate_ = minimal_follower_tag_rate;
//  for(auto & tag_rate_diagnostic : follower_tag_rate_diagnotics_)
//  {
//    tag_rate_diagnostic.setMinimalRate(minimal_follower_tag_rate_);
//  }
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::setMaximalFollowerTagRate(const double & maximal_follower_tag_rate)
//{
//  maximal_follower_tag_rate_ = maximal_follower_tag_rate;
//  for(auto & tag_rate_diagnostic : follower_tag_rate_diagnotics_)
//  {
//    tag_rate_diagnostic.setMaximalRate(maximal_follower_tag_rate_);
//  }
//}

////-----------------------------------------------------------------------------
//const double & R2HLocalisationDiagnostic::getMinimalOdoRate()const
//{
//  return minimal_linear_speed_rate_;
//}

////-----------------------------------------------------------------------------
//const double & R2HLocalisationDiagnostic::setMinimalAngularSpeedRate()const
//{
//  return minimal_angular_speed_rate_;
//}

////-----------------------------------------------------------------------------
//const double & R2HLocalisationDiagnostic::getMinimalFollowerTagRate()const
//{
//  return minimal_follower_tag_rate_;
//}

////-----------------------------------------------------------------------------
//const double & R2HLocalisationDiagnostic::getMaximalFollowerTagRate()const
//{
//  return maximal_follower_tag_rate_;
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::updateLinearSpeedRate(const romea::Duration & duration)
//{
//  linear_speed_rate_monitoring_.update(duration);
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::updateAngularSpeedRate(const romea::Duration & duration)
//{
//  angular_speed_rate_monitoring_.update(duration);
//}


////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::updateFollowerTagRate(const romea::Duration & duration, const size_t & tagIndex)
//{
//  follower_tag_rate_monitorings_[tagIndex].update(duration);
//}

////-----------------------------------------------------------------------------
//void  R2HLocalisationDiagnostic::udapteLocalisationStatus(const LocalisationStatus & status)
//{
//  localisation_statuc_diagnostic_.setLocalisationStatus(status);
//}

////-----------------------------------------------------------------------------
//double R2HLocalisationDiagnostic::getLinearSpeedRate() const
//{
//  return linear_speed_rate_monitoring_.getRate();
//}

////-----------------------------------------------------------------------------
//double R2HLocalisationDiagnostic::getAngularSpeedRate() const
//{
//  return angular_speed_rate_monitoring_.getRate();
//}

////-----------------------------------------------------------------------------
//double R2HLocalisationDiagnostic::getFollowerTagRate(const size_t & tagIndex) const
//{
//  return follower_tag_rate_monitorings_[tagIndex].getRate();
//}

////-----------------------------------------------------------------------------
//LocalisationStatus R2HLocalisationDiagnostic::getLocalisationStatus() const
//{
//  return localisation_statuc_diagnostic_.getLocalisationStatus();
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isLinearSpeedRateOk() const
//{
//  return romea::RateMonitoring::isGreaterThanRate(getLinearSpeedRate(),minimal_linear_speed_rate_);
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isAngularSpeedRateOk() const
//{
//  return romea::RateMonitoring::isGreaterThanRate(getAngularSpeedRate(),minimal_angular_speed_rate_);
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isLinearSpeedOk(const romea::Duration & duration) const
//{
//  if(linear_speed_rate_monitoring_.timeout(duration))
//    return false;

//  return isLinearSpeedRateOk();
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isAllOk(const romea::Duration & duration)const
//{
//  return isProprioceptiveOk(duration) && isFollowerTagsOk(duration);
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isFollowerTagRateOk(const size_t & tagIndex) const
//{
//  return romea::RateMonitoring::isInsideRangeRate(getFollowerTagRate(tagIndex),
//                                                      minimal_follower_tag_rate_,
//                                                      maximal_follower_tag_rate_);
//}


////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isAngularSpeedOk(const romea::Duration & duration) const
//{
//  if(angular_speed_rate_monitoring_.timeout(duration))
//    return false;

//  return isAngularSpeedRateOk();
//}


////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isProprioceptiveOk(const romea::Duration & duration) const
//{
//  return isAngularSpeedOk(duration) && isLinearSpeedOk(duration);
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isFollowerTagOk(const romea::Duration & duration,const size_t & tagIndex) const
//{
//  if(follower_tag_rate_monitorings_[tagIndex].timeout(duration))
//    return false;

//  return isFollowerTagRateOk(tagIndex);
//}

////-----------------------------------------------------------------------------
//bool R2HLocalisationDiagnostic::isFollowerTagsOk(const romea::Duration & duration)const
//{
//  for(size_t n=0; n < follower_tag_rate_monitorings_.size();n++)
//  {
//    if(!isFollowerTagOk(duration,n))
//    {
//      return false;
//    }
//  }

//  return true;
//}

////-----------------------------------------------------------------------------
//void R2HLocalisationDiagnostic::publish()
//{

//  linear_speed_rate_diagnostic_.update(getLinearSpeedRate());
//  angular_speed_rate_diagnostic_.update(getAngularSpeedRate());

//  for(size_t n=0; n< follower_tag_rate_monitorings_.size() ;n++)
//  {
//    follower_tag_rate_diagnotics_[n].update(getFollowerTagRate(n));
//  }

//  diagnostics_updater_.update();
//}

