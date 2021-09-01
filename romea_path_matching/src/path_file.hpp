#ifndef romea_PathFile_HPP
#define romea_PathFile_HPP

//eigen
#include <Eigen/Geometry>

//std
#include <string>
#include <fstream>

//boost
#include <boost/optional.hpp>

//romea
#include <romea_path/PathWayPoint2D.hpp>

namespace romea
{

class PathFile
{

public :

  PathFile(const std::string & filename);

  const std::vector<std::vector<PathWayPoint2D>> & getWayPoints() const;

  const std::string & getCoordinateSystemDescription() const;

  const Eigen::Affine3d & getWorldToPathTransformation() const;

private :

  void loadHeader_();

  void loadWayPoints_();

private :

  std::string coordinate_system_;
  Eigen::Affine3d world_to_path_;
  std::vector<std::vector<PathWayPoint2D>> way_points_;
  std::ifstream file_;

};

}

#endif
