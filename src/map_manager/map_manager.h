#pragma once

#include <string>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "srrg_boss/deserializer.h"

#include "srrg_core_map_2/local_map.h"
#include "srrg_core_map_2/map_node_relation.h"

#include "srrg_ros_wrappers/ros_utils.h"



namespace map_manager{

class MapManager{
public:
    MapManager(ros::NodeHandle& nh):_nh(nh){}
    void loadLocalMapsFromFile(const std::string& filename="");
    void setInitialGuess(int id=0);
    void subscribeCallbacks(const std::string& pose_topic="");
private:
    ros::NodeHandle _nh;
    ros::Subscriber _pose_sub;
    ros::Publisher _map_pub;
    ros::Publisher _metadata_pub;
    ros::ServiceServer _service;

    nav_msgs::MapMetaData _metadata_message;
    nav_msgs::GetMap::Response _map_resp;

    srrg_boss::Deserializer _deserializer;

    srrg_core_map_2::LocalMap3D* _current_map;
    srrg_core_map_2::MapNodePtrSet _nodes;
    srrg_core_map_2::BinaryMapNodeRelationPtrSet _relations;

    Eigen::Isometry3f _current_map_transform;
    Eigen::Isometry3f _robot_pose;

    typedef std::map<srrg_core_map_2::BaseMapNode*,srrg_core_map_2::MapNodePtrSet> MapNodePtrMapNodePtrSetMap;
    MapNodePtrMapNodePtrSetMap _neighbors_map;

    void getTraversabilityFromLocalMap();

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    bool isTraversable(srrg_core_map_2::LocalMap3D* current_map,Eigen::Vector3f p);

    bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
};

}
