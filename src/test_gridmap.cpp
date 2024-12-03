#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

class variousTerrainPlanner
{
private:
    ros::NodeHandle nh;
public:
    variousTerrainPlanner(ros::NodeHandle nh_);
    void execute();
    ~variousTerrainPlanner();
};


variousTerrainPlanner::variousTerrainPlanner(ros::NodeHandle nh_):nh(nh_)
{
}
void variousTerrainPlanner::execute()
{
    using namespace grid_map;
GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) =0;
    }
}

variousTerrainPlanner::~variousTerrainPlanner()
{
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  // ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  variousTerrainPlanner planner(nh);
    planner.execute();
  


  return 0;
}
