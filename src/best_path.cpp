/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/best_path.h"

namespace pose_prediction_ism
{

BestPath::BestPath(std::string database_filename):
    PredictorWithScore::PredictorWithScore(database_filename, "best_path", Best)
{
}

BestPath::~BestPath()
{
}

  AttributedPointCloud BestPath::predictUnfoundPoses(ISM::PosePtr &reference_pose, std::string pattern_name, double percentage_of_records_for_prediction)
{

    ROS_INFO_STREAM("Pose prediction is run for scene " << pattern_name << " with " << percentage_of_records_for_prediction << " times the demonstration trajectory lengths.");
    best_path_map_.clear();
    specifiers_size_map_.clear();
    clearPointCloud();
    IsmObjects x;
    x.push_back(IsmObject (pattern_name, "MOCK"));
    createBestPathMap(pattern_name,  x);
    ROS_DEBUG_STREAM("These are best paths for unfound objects in scene" << pattern_name << ":");
    printBestPaths();
    ROS_ASSERT(!best_path_map_.empty());
    ROS_ASSERT(!specifiers_size_map_.empty());
    ROS_ASSERT(isPointCloudEmpty());
    createAttributedPointCloud(reference_pose, percentage_of_records_for_prediction);
    return attributed_point_cloud_;
}

void BestPath::createBestPathMap(std::string type, IsmObjects predecessors)
{
    IsmObjectSet objects_in_pattern = getObjectTypesAndIdsBelongingToPattern(type);
    for (IsmObject object : objects_in_pattern)
    {
        if (!isFoundObject(object))
        {
            predecessors.push_back(object);
            if (!isReferenceObject(object))
                createBestPathMap(object.first, predecessors);
            else
            {
                std::pair<BestPathMap::iterator,bool>ret;
                double new_score = calculatePathScore(predecessors);
                PathWithScore new_path_with_score(predecessors, new_score);
                ret = best_path_map_.insert(std::pair<IsmObject, PathWithScore>(object,
                                                                         new_path_with_score));
                if (ret.second){
		  specifiers_size_map_[object] = votes_.at(type)
		    .at(object.first)
		    .at(object.second).size();
		  ROS_DEBUG_STREAM(specifiers_size_map_[object] << " votes available for object " << "(" << object.first << "," << object.second << ") in scene " << type);
		  
		}
                else
                {
		  ROS_DEBUG_STREAM("Object " << "(" << object.first << "," << object.second << ")" << " already has a path");  
		  PathWithScore old_path_with_score = best_path_map_[object];
		  double old_score = old_path_with_score.second;
		  if (old_score >= new_score)
		    ROS_DEBUG_STREAM("The the new score is NOT better than old. No changes");
		  else
                    {
		      ROS_DEBUG_STREAM("The the new score is better than old. The new path replaces the old");
		      best_path_map_[object] = new_path_with_score;
		      specifiers_size_map_[object] = votes_.at(type)
			.at(object.first).at(object.second).size();
		      ROS_ASSERT_MSG(best_path_map_[object].second == new_score,
				     "The new path was not inserted");
                    }
                }
            }
            predecessors.pop_back();
        }
        else
            ROS_DEBUG_STREAM("Object " << "(" << object.first << "," << object.second << ")" << "was already found");
    }
}

void BestPath::createAttributedPointCloud(ISM::PosePtr pose, double percentage_of_records_for_prediction)
{
    for (BestPathMap::iterator best_path_it = best_path_map_.begin();
         best_path_it != best_path_map_.end(); ++best_path_it )
    {
        unsigned int threshold = round(percentage_of_records_for_prediction * specifiers_size_map_[best_path_it ->first]);
        IsmObject object  = best_path_it ->first;
        PathWithScore path_with_score = best_path_it ->second;
        IsmObjects predecessors = path_with_score.first;
        for (unsigned int i = 0; i < threshold ; ++i)
        {
            ISM::PosePtr predicted_object_pose_ptr = predictPose(pose, predecessors);
            addPointToPointCloud(predicted_object_pose_ptr, object.first, object.second);
        }
    }
}
double BestPath::calculatePathScore(IsmObjects path)
{

    double score = 1 / ( (double) path.size());
    ROS_ASSERT_MSG(score < 1,
                   "There must be more than one predecessor, otherwise all objects are in the reference");
    return score;
}

void BestPath::printBestPaths()
{
    for (BestPathMap::iterator best_path_it = best_path_map_.begin();
         best_path_it != best_path_map_.end();
         ++best_path_it)
    {
        //uint threshold = round(percentage * specifiersSizeMap[shortestPathIt->first]);
        IsmObject object  = best_path_it->first;
        PathWithScore path_with_score = best_path_it->second;
        IsmObjects predecessors = path_with_score.first;
        ROS_DEBUG_STREAM("(" << object.first << "," << object.second << ")" << " has " << predecessors.size() << " predecessors");
        for (IsmObject pre : predecessors)
            ROS_DEBUG_STREAM("> " << pre.first << ": " << pre.second.substr(0,5));
    }
}

}
