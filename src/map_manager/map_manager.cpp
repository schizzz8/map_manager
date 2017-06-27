#include "map_manager.h"

namespace map_manager {

using namespace std;
using namespace srrg_boss;
using namespace srrg_core_map_2;
using namespace srrg_core_ros;

void MapManager::loadLocalMaps(const string &filename){

    Serializable* o;

    if(!filename.length())
        return;

    _deserializer.setFilePath(filename);

    while (o=_deserializer.readObject()){
        LocalMap3D* lmap = dynamic_cast<LocalMap3D*> (o);
        if(lmap)
            _nodes.insert(lmap);

        Pose3DPose3DMapNodeRelation* rel = dynamic_cast<Pose3DPose3DMapNodeRelation*> (o);
        if(rel){
            LocalMap3D* from = dynamic_cast<LocalMap3D*> (rel->from());
            LocalMap3D* to = dynamic_cast<LocalMap3D*> (rel->to());
            if(from && to)
                _relations.insert(rel);
        }
    }

    cerr << "Read " << _nodes.size() << " local maps." << endl;
    cerr << "Read " << _relations.size() << " relations." << endl;

    //building neighbors map
    for(MapNodePtrSet::iterator it=_nodes.begin();it!=_nodes.end();++it)
        for(MapNodePtrSet::iterator jt=_nodes.begin();jt!=_nodes.end();++jt){

            int id1=(*it)->getId(),id2=(*jt)->getId();

            if(id1==id2)
                continue;

            for(BinaryMapNodeRelationPtrSet::iterator kt=_relations.begin();kt!=_relations.end();++kt)
                if((*kt)->from()->getId() == id1 && (*kt)->to()->getId() == id2 ||
                        (*kt)->from()->getId() == id2 && (*kt)->to()->getId() == id1){
                    MapNodePtrMapNodePtrSetMap::iterator lt = _neighbors_map.find(*it);
                    if(lt!=_neighbors_map.end())
                        lt->second.insert(*jt);
                    else{
                        MapNodePtrSet set;
                        set.insert(*jt);
                        _neighbors_map.insert(make_pair(*it,set));
                    }
                }
        }
}

void MapManager::subscribeCallbacks(const string &pose_topic){
    _pose_sub = _nh.subscribe(pose_topic,10,&MapManager::poseCallback,this);
}

void MapManager::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    _robot_pose=pose2eigen(msg.get()->pose.pose);

    if(! isTraversable(_current_map,_robot_pose.translation())){
        MapNodePtrSet reference_neighbors=_neighbors_map[_current_map];
        for(MapNodePtrSet::iterator it = reference_neighbors.begin(); it != reference_neighbors.end(); ++it){
            LocalMap3D* neighbor = dynamic_cast<LocalMap3D*>(*it);
            if(isTraversable(neighbor,(neighbor->estimate().inverse()*_current_map_transform*_robot_pose).translation())){
                _current_map = neighbor;
                _current_map_transform = _current_map->estimate();
                break;
            }
        }
    }

}

bool MapManager::isTraversable(LocalMap3D *current_map, Eigen::Vector3f p){
    srrg_core::TraversabilityMap* traversability = current_map->traversabilityMap();

    if(!traversability)
        return false;

    float resolution = traversability->resolution();
    const Eigen::Vector3f& origin = traversability->origin();
    srrg_core::ImageData* image = traversability->image();

    Eigen::Vector3f projected = (p - origin)/resolution;
    int r=projected.y();
    int c=projected.x();
    if (r>=image->image().rows || r<0)
        return false;
    if (c>=image->image().cols || r<0)
        return false;
    if (image->image().at<unsigned char>(r,c)!=0)
        return false;
    return true;
}
}
