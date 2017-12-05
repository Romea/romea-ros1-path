//#ifndef __DiagnosticR2HLocalisation_HPP__
//#define __DiagnosticR2HLocalisation_HPP__

////ros
//#include <diagnostic_updater/diagnostic_updater.h>

////std
//#include <atomic>

////romea
//#include <monitoring/RateMonitoring.hpp>
//#include <ros_diagnostic_util.hpp>

//typedef enum
//{
//  WAIT_FOR_PROPRIOCEPTIVE_DATA=0,
//  WAIT_FOR_RANGE_DATA,
//  RUNNING
//}LocalisationStatus;

//class DiagnosticLocalisationStatus : public diagnostic_updater::DiagnosticTask
//{
//public:

//  DiagnosticLocalisationStatus(const std::string &name);

//  void setLocalisationStatus(const LocalisationStatus &status);

//  LocalisationStatus getLocalisationStatus() const;

//  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

//private :
//  LocalisationStatus status_;
//};


//class R2HLocalisationDiagnostic
//{

//public:

//  R2HLocalisationDiagnostic();

//  void addFollowerTag(const unsigned long long & tagId);

//  void setMinimalLinearSpeedRate(const double & minimal_linear_speed_rate);
//  void setMinimalAngularSpeedRate(const double & minimal_angular_speed_rate);
//  void setMinimalFollowerTagRate(const double & minimal_tag_rate);
//  void setMaximalFollowerTagRate(const double & maximal_tag_rate);

//  const double & getMinimalOdoRate()const;
//  const double & setMinimalAngularSpeedRate()const;
//  const double & getMinimalFollowerTagRate()const;
//  const double & getMaximalFollowerTagRate()const;

//  void updateLinearSpeedRate(const romea::Duration & duration);
//  void updateAngularSpeedRate(const romea::Duration & duration);
//  void updateFollowerTagRate(const romea::Duration & duration, const size_t & tagIndex);
//  void udapteLocalisationStatus(const LocalisationStatus & status);

//  double getLinearSpeedRate() const;
//  double getAngularSpeedRate() const;
//  double getFollowerTagRate(const size_t & tagIndex) const;
//  LocalisationStatus getLocalisationStatus() const;

//  bool isLinearSpeedOk(const romea::Duration & duration) const;
//  bool isAngularSpeedOk(const romea::Duration & duration) const;
//  bool isProprioceptiveOk(const romea::Duration & duration) const;
//  bool isFollowerTagOk(const romea::Duration & duration,const size_t & tagIndex) const;
//  bool isFollowerTagsOk(const romea::Duration & duration)const;
//  bool isAllOk(const romea::Duration & duration)const;

//  bool isLinearSpeedRateOk() const;
//  bool isAngularSpeedRateOk() const;
//  bool isFollowerTagRateOk(const size_t & tagIndex) const;

//  void publish();

//private :

//  double minimal_linear_speed_rate_;
//  double minimal_angular_speed_rate_;
//  double minimal_follower_tag_rate_;
//  double maximal_follower_tag_rate_;

//  romea::RateMonitoring linear_speed_rate_monitoring_;
//  romea::RateMonitoring angular_speed_rate_monitoring_;
//  std::deque<romea::RateMonitoring> follower_tag_rate_monitorings_;

//  DiagnosticLocalisationStatus localisation_statuc_diagnostic_;
//  romea::DiagnosticGreaterThanRate linear_speed_rate_diagnostic_;
//  romea::DiagnosticGreaterThanRate angular_speed_rate_diagnostic_;
//  std::deque<romea::DiagnosticInsideRangeRate> follower_tag_rate_diagnotics_;

//  diagnostic_updater::CompositeDiagnosticTask composite_diagnostic_;
//  diagnostic_updater::Updater diagnostics_updater_;

//};

//#endif
