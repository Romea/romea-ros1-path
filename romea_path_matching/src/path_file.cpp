//romea
#include "path_file.hpp"
#include <romea_common/geodesy/ENUConverter.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
PathFile::PathFile(const std::string & filename):
  coordinate_system_(),
  world_to_path_(),
  way_points_(),
  file_(filename)
{
  if(file_.is_open())
  {
   loadHeader_();
   loadWayPoints_();

  }
  else
  {
      throw(std::runtime_error("Failed to open path file "+ filename));
  }

//  if(revert)
//  {
//    std::reverse(std::begin(points_), std::end(points_));
//  }

}

//-----------------------------------------------------------------------------
void PathFile::loadHeader_()
{

  std::string header;
  file_ >> header;

  if(header.compare("WGS84")==0)
  {
    double reference_latitude;
    double reference_longitude;
    double reference_altitude;

    file_ >> reference_latitude >> reference_longitude >> reference_altitude;

    romea::GeodeticCoordinates anchor(reference_latitude/180.*M_PI,
                                      reference_longitude/180.*M_PI,
                                      reference_altitude);

    world_to_path_= ENUConverter(anchor).getEnuToEcefTransform();
  }
  else if(header.compare("ENU")==0 || header.compare("PIXEL")==0)
  {
    world_to_path_= Eigen::Affine3d::Identity();
  }
  else
  {
    throw(std::runtime_error("Failed to extract path header"));
  }

}

//-----------------------------------------------------------------------------
void PathFile::loadWayPoints_()
{

  size_t number_of_sections;
  file_>> number_of_sections;
  std::cout << " number of sections " << number_of_sections<< std::endl;
  way_points_.resize(number_of_sections);
  for(size_t i=0;i<number_of_sections;++i)
  {
    size_t number_of_way_points;
    file_>> number_of_way_points;

    size_t number_of_columns;
    file_>> number_of_columns;

    std::cout << " number of way points " << number_of_way_points<< std::endl;
    std::cout << " number of columns " << number_of_columns<< std::endl;

    way_points_[i].resize(number_of_way_points);
    for(size_t j= 0; j<number_of_way_points; ++j)
    {
       file_>> way_points_[i][j].position.x() ;
       file_>> way_points_[i][j].position.y() ;

       if(number_of_columns==3)
       {
         file_>> way_points_[i][j].desired_speed;
       }

       std::cout << j <<" "
                 <<  way_points_[i][j].position.x() << " "
                  <<way_points_[i][j].position.y()<<" "
                 << way_points_[i][j].desired_speed<<std::endl;
    }
  }
}


//-----------------------------------------------------------------------------
const std::vector<std::vector<PathWayPoint2D> > &PathFile::getWayPoints() const
{
  return way_points_;
}

//-----------------------------------------------------------------------------
const std::string & PathFile::getCoordinateSystemDescription() const
{
  return coordinate_system_;
}

//-----------------------------------------------------------------------------
const Eigen::Affine3d & PathFile::getWorldToPathTransformation() const
{
  return world_to_path_;
}

}

