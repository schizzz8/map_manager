#include <ros/ros.h>
#include "map_manager/map_manager.h"
#include <string>

using namespace std;
using namespace map_manager;

void startManager(MapManager* manager, ros::NodeHandle& private_nh){
    cerr << "Map manager parameters: " << endl;

    string filename;
    private_nh.param("filename",filename,string(""));
    cerr << "[string] _filename: " << filename << endl;

    string pose_topic;
    private_nh.param("pose_topic",pose_topic,string("amcl_pose"));
    cerr << "[string] _pose_topic: " << pose_topic << endl;

    int initial_guess;
    private_nh.param("initial_guess",initial_guess,0);
    cerr << "[int] _initial_guess: " << initial_guess << endl;

    manager->loadLocalMapsFromFile(filename);
    manager->setInitialGuess(initial_guess);
    manager->subscribeCallbacks(pose_topic);
}

int main (int argc, char* argv[]){
    ros::init(argc,argv,"map_manager");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MapManager* manager = new MapManager(nh);

    if(ros::ok())
        startManager(manager,private_nh);

    ros::spin();

    return 0;
}
