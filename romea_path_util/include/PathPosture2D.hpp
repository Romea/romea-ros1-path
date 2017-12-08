#ifndef romea_Posture2D_hpp
#define romea_Posture2D_hpp

//eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//std
#include <ostream>

namespace romea {

class PathPosture2D {
public :

  PathPosture2D();

  PathPosture2D(const Eigen::Vector2d & position,
                const double & course,
                const double & curvature,
                const double & dotCurvature);

  PathPosture2D(const Eigen::Vector2d & position,
                const double & course,
                const double & curvature);

  PathPosture2D(const double & x,
                const double & y,
                const double & course,
                const double & curvature,
                const double & dotCurvature);

  PathPosture2D(const double & x,
                const double & y,
                const double & course,
                const double & curvature);


  Eigen::Vector2d getPosition() const;
  double  getX() const;
  double  getY() const;
  double  getCourse() const;
  double  getCurvature() const;
  double  getDotCurvature() const;

protected:

  Eigen::Vector2d position_;
  double course_;
  double curvature_;
  double dotCurvature_;

};

std::ostream& operator<<(std::ostream & os, const PathPosture2D & posture);

}

#endif
