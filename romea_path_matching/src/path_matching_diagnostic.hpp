#ifndef __DiagnosticPathMatching_HPP__
#define __DiagnosticPathMatching_HPP__

//#include "path_matching_diagnostic_base.hpp"

//namespace romea{

//class DiagnosticPathStatus : public diagnostic_updater::DiagnosticTask
//{
//public:

//  DiagnosticPathStatus(const std::string &name);

//  void setPathFilename(const std::string & filename);

//  std::string getPathFilename();

//  void setStatus(const bool &status);

//  bool getStatus() const;

//  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

//private :

//  std::string filename_;
//  bool status_;
//};

//class DiagnosticLookupTransformStatus : public diagnostic_updater::DiagnosticTask
//{
//public:

//  DiagnosticLookupTransformStatus(const std::string &name);

//  void setStatus(const bool &status);

//  bool getStatus() const;

//  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

//private :
//  bool status_;
//};




//class PathMatchingDiagnostic : public PathMatchingDiagnosticBase
//{

//public:

//  PathMatchingDiagnostic();
//  virtual ~PathMatchingDiagnostic()=default;

//  void updateLookupTransformStatus(const bool & status);
//  void updatePathStatus(const std::string & filename, const bool & isOpened);

//private :

//  DiagnosticPathStatus path_status_diagnostic_;
//  DiagnosticLookupTransformStatus lookup_transform_status_diagnostic_;

//};

//}


//romea
#include "path_matching_diagnostic_base.hpp"

namespace romea{

class PathMatchingDiagnostic : public PathMatchingDiagnosticBase
{
public:

  PathMatchingDiagnostic();

  virtual ~PathMatchingDiagnostic()=default;

  void setPathFilename(const std::string & filename);
  void updateLookupTransformStatus(const bool & status);
  void updatePathStatus(const bool & isOpened);

  virtual DiagnosticReport getReport()const override;

};

}

#endif

