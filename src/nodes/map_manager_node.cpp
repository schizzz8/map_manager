#include <ros/ros.h>
#include "map_manager/map_manager.h"

using namespace std;
using namespace map_manager;

int main (int argc, char* argv[]){
    ros::init(argc,argv,"map_manager");
    ros::NodeHandle nh;

    MapManager manager(nh);

    return 0;
}
