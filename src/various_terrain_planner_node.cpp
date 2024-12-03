#include <ros/ros.h>
#include <glog/logging.h>
#include <various_terrain_planner.h>
using namespace std;


int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "various_terrain");
    ros::NodeHandle nh("~");
    variousTerrainPlanner planner(nh);
    planner.execute();
    return 0;

}

