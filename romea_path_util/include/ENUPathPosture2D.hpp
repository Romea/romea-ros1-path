#ifndef romea_Posture2D_hpp
#define romea_Posture2D_hpp

//eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//std
#include <ostream>

namespace romea {

class ENUPathPosture2D {
public :

  ENUPathPosture2D();

  ENUPathPosture2D(const Eigen::Vector2d & position,
                   const double & orientationAroundZAxis,
                   const double & curvatureAlongPath,
                   const double & dotCurvatureAlongPath);

  ENUPathPosture2D(const Eigen::Vector2d & position,
                   const double & orientationAroundZAxis,
                   const double & curvatureAlongPath);

  ENUPathPosture2D(const double & positionAlongXAxis,
                   const double & positionAlongYAxis,
                   const double & orientationAroundZAxis,
                   const double & curvatureAlongPath,
                   const double & dotCurvatureAlongPath);

  ENUPathPosture2D(const double & positionAlongXAxis,
                   const double & positionAlongYAxis,
                   const double & orientationAroundZAxis,
                   const double & curvatureAlongPath);


  Eigen::Vector2d getPosition() const;
  double  getPositionAlongXEastAxis() const;
  double  getPositionAlongYNorthAxis() const;
  double  getOrientationAroundZUpAxis() const;
  double  getCurvatureAlongPath() const;
  double  getDotCurvatureAlongPath() const;

protected:

  Eigen::Vector2d position_;
  double orientationAroundZAxis_;
  double curvatureAlongPath_;
  double dotCurvatureAlongPath_;

};

std::ostream& operator<<(std::ostream & os, const ENUPathPosture2D & posture);

}

#endif
