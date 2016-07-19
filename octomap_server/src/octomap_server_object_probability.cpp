#include <ros/ros.h>
#include <octomap_server/OctomapServerObjectProbability.h>

#define USAGE "\nUSAGE: octomap_server_object_probability <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace octomap_server;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server_object_probability");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");
  std::string mapFilename(""), mapFilenameParam("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  OctomapServerObjectProbability server;
  ros::spinOnce();

  if (argc == 2){
    mapFilename = std::string(argv[1]);
  }

  if (private_nh.getParam("map_file", mapFilenameParam)) {
    if (mapFilename != "") {
      ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
    } else {
      mapFilename = mapFilenameParam;
    }
  }

  if (mapFilename != "") {
    if (!server.openFile(mapFilename)){
      ROS_ERROR("Could not open file %s", mapFilename.c_str());
      exit(1);
    }
  }

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
