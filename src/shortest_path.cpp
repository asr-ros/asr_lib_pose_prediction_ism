/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/shortest_path.h"

namespace pose_prediction_ism
{
ShortestPath::ShortestPath(string database_filename):
    PosePredictor::PosePredictor(database_filename, "shortest_path",
                                 Shortest)
{
}
ShortestPath::~ShortestPath()
{
}

AttributedPointCloud ShortestPath::predictUnfoundPoses(ISM::PosePtr &reference_pose, string pattern_name, double percentage_of_records_for_prediction)
{
    ROS_DEBUG_STREAM_NAMED("ShortestPath", "ShortestPath::predictUnfoundPoses");

    ROS_INFO_STREAM("Pose prediction is run for scene " << pattern_name << " with " << percentage_of_records_for_prediction << " times the demonstration trajectory lengths.");
    ROS_INFO_STREAM("It is run for a reference with the pose " << reference_pose << ".");

    shortest_path_map_.clear();
    specifiers_size_map_.clear();
    clearPointCloud();
    IsmObjects x;
    x.push_back(IsmObject (pattern_name, "MOCK"));
    createShortestPathMap(pattern_name,  x);
    ROS_DEBUG_STREAM_NAMED("ShortestPath", "These are shortest paths for unfound objects in scene" << pattern_name << ":");
    printShortestPaths();
    ROS_ASSERT(!shortest_path_map_.empty());
    ROS_ASSERT(!specifiers_size_map_.empty());
    ROS_ASSERT(isPointCloudEmpty());
    createAttributedPointCloud(reference_pose, percentage_of_records_for_prediction);
    ROS_DEBUG_STREAM_NAMED("ShortestPath", "created attributed point cloud with " << attributed_point_cloud_.elements.size() << " elements");

    return attributed_point_cloud_;
}
void ShortestPath::createShortestPathMap(string type, IsmObjects predecessors)
{

    IsmObjectSet objects_in_pattern = getObjectTypesAndIdsBelongingToPattern(type);
    // For all nonReference objects on this tree level: Save shortest path in map
    for (IsmObject object : objects_in_pattern)
    {
        if (!isFoundObject(object) && !isReferenceObject(object))
        {
            ROS_DEBUG_STREAM_NAMED("ShortestPath", "Save shortest path in map for non-reference Object: " << "(" << object.first << "," << object.second << ")");
            predecessors.push_back(object);
            Tupel<ShortestPathMap::iterator,bool>ret;
            ret = shortest_path_map_.insert ( Tupel<IsmObject, IsmObjects>(object, predecessors));
            if (ret.second){
                specifiers_size_map_[object] = votes_
                        .at(type)
                        .at(object.first)
                        .at(object.second).size();
                ROS_DEBUG_STREAM_NAMED("ShortestPath", ceil(average_votes_ * prediction_generation_factor_) << " votes available for object " << "(" << object.first << "," << object.second << ") in scene " << type);
            }
            else{
                ROS_DEBUG_STREAM_NAMED("ShortestPath", "Object " << "(" << object.first << "," << object.second << ")" << " already has a path");
            }

            predecessors.pop_back();
        }
        else
        {
            ROS_DEBUG_STREAM_COND_NAMED(isFoundObject(object), "ShortestPath", "Object " << "(" << object.first << "," << object.second << ")" << "was already found");
            ROS_DEBUG_STREAM_COND_NAMED(isReferenceObject(object), "ShortestPath", "Object " << "(" << object.first << "," << object.second << ")" << "is a reference object");
        }
    }
    // For all reference objects on this tree level: Call recursively the createShortestPathMap function
    for (IsmObject object : objects_in_pattern)
    {
        if (!isFoundObject(object) && isReferenceObject(object))
        {
            ROS_DEBUG_STREAM_NAMED("ShortesPath", "Call createShortestPathMap for reference: " << "(" << object.first << "," << object.second << ")");
            predecessors.push_back(object);
            createShortestPathMap(object.first, predecessors);
            predecessors.pop_back();
        }
    }
}
void ShortestPath::createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction)
{
    ROS_DEBUG_NAMED("ShortesPath", "ShortesPath::createAttributedPointCloud");

    for (ShortestPathMap::iterator shortest_path_it = shortest_path_map_.begin();
         shortest_path_it != shortest_path_map_.end(); ++shortest_path_it)
    {
        //const local_uint SIZE = specifiers_size_map_[shortest_path_it->first];

        local_uint threshold = ceil(percentage_of_records_for_prediction * average_votes_ * prediction_generation_factor_);
        ROS_DEBUG_NAMED("ShortesPath", "ShortesPath: generated poses, threshold: %f", ceil(percentage_of_records_for_prediction * average_votes_ * prediction_generation_factor_));
        IsmObject object  = shortest_path_it->first;
        IsmObjects predecessors = shortest_path_it->second;
        ROS_DEBUG_STREAM_NAMED("ShortesPath", "ISMOBject: " << "(" << object.first << "," << object.second << ")");
        for(IsmObject temp_obj : predecessors){
            ROS_DEBUG_STREAM_NAMED("ShortesPath", "predecessors: " << "(" << temp_obj.first << "," << temp_obj.second << ")");
        }


        for (local_uint i = 0; i < threshold ; ++i)
        {
            ISM::PosePtr object_pose_ptr = predictPose(reference_pose_ptr, predecessors);
            addPointToPointCloud(object_pose_ptr, object.first, object.second);
        }
    }
}
void ShortestPath::printShortestPaths()
{
    for (ShortestPathMap::iterator shortest_path_it = shortest_path_map_.begin();
         shortest_path_it != shortest_path_map_.end();
         ++shortest_path_it)
    {

        IsmObject object  = shortest_path_it->first;
        IsmObjects predecessors = shortest_path_it->second;
        ROS_DEBUG_STREAM_NAMED("ShortestPath", "Unfound object " << "(" << object.first << "," << object.second << ")" << " has " << predecessors.size() << " predecessors in path to reference:");
        for (IsmObject pre : predecessors)
            ROS_DEBUG_STREAM_NAMED("ShortestPath", "> " << pre.first << ": " << pre.second.substr(0,5));
    }
}
}

