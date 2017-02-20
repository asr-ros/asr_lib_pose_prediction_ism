/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/random_path.h"

namespace pose_prediction_ism {
RandomPath::RandomPath(string database_filename):
    PredictorWithScore::PredictorWithScore(database_filename, "random_path", Random)
{
}

RandomPath::~RandomPath()
{
}

  AttributedPointCloud RandomPath::predictUnfoundPoses(ISM::PosePtr &reference_pose_ptr, string pattern_name, double percentage_of_records_for_prediction)
{
    ROS_INFO_STREAM("Pose prediction is run for scene " << pattern_name << " with " << percentage_of_records_for_prediction << " times the demonstration trajectory lengths.");
    paths_with_scores_map_.clear();
    slots_map_.clear();
    specifiers_size_map_.clear();
    clearPointCloud();
    IsmObjects x;
    x.push_back(IsmObject (pattern_name, "MOCK"));
    createPathsWithScoresMap(pattern_name,  x);
    createSlotsMap();
    createAttributedPointCloud(reference_pose_ptr, percentage_of_records_for_prediction);
    return attributed_point_cloud_;
}

void RandomPath::createPathsWithScoresMap(string type,
                                          IsmObjects predecessors)
{
    IsmObjectSet objects_in_pattern = getObjectTypesAndIdsBelongingToPattern(type);
    for (IsmObject object : objects_in_pattern)
    {
        if (!isFoundObject(object))
        {
            predecessors.push_back(object);
            if (isReferenceObject(object))
                createPathsWithScoresMap(object.first, predecessors);
            else
            {
                Tupel<PathsWithScoresMap::iterator,bool>ret;
                double newScore = calculatePathScore(predecessors);
                PathWithScore newPathWithScore(predecessors,newScore);
                PathsWithScores p;
                p.push_back(newPathWithScore);
                ret = paths_with_scores_map_.insert (Tupel<IsmObject, PathsWithScores>(object, p));
                if (ret.second)
                {
                    ROS_ASSERT_MSG(paths_with_scores_map_[object].size() == 1,
                            "New path was not inserted");
                    specifiers_size_map_[object] = votes_.at(type)
                            .at(object.first)
                            .at(object.second).size();
                }
                else
                {
                    local_uint oldLength = paths_with_scores_map_[object].size();
                    paths_with_scores_map_[object].push_back(newPathWithScore);
                    ROS_ASSERT_MSG(paths_with_scores_map_[object].size() == 1 + oldLength,
                            "New path was not inserted");
                }
            }
            predecessors.pop_back();
        }
        else
            ROS_DEBUG_STREAM("Object " << "(" << object.first << "," << object.second << ")" << "was already found");
    }
}

  void RandomPath::createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction)
{
    ROS_ASSERT(isPointCloudEmpty());
    for (PathsWithScoresMap::iterator paths_it = paths_with_scores_map_.begin();
         paths_it != paths_with_scores_map_.end(); ++paths_it)
    {
        const local_uint SIZE = specifiers_size_map_[paths_it->first];
        local_uint threshold = round(percentage_of_records_for_prediction * SIZE);
        IsmObject object  = paths_it->first;
        PathsWithScores paths_with_scores = paths_it->second;

        bool has_choice_for_path = paths_with_scores.size() > 1;
        Slots percentage_slots;
        if (has_choice_for_path)
            percentage_slots = slots_map_[object];
        for (local_uint i = 0; i < threshold ; ++i)
        {
            IsmObjects predecessors;
            if (has_choice_for_path)
            {
                double slot = rand() / double(RAND_MAX);
                Slots::iterator percentage_slots_it = percentage_slots.begin();
                PathsWithScores::iterator current_path_it = paths_with_scores.begin();
                ROS_ASSERT(percentage_slots.size() == paths_with_scores.size());
                while (slot > (*percentage_slots_it))
                {
                    percentage_slots_it++;
                    current_path_it++;
                    ROS_ASSERT(percentage_slots_it != percentage_slots.end());
                }
                predecessors = current_path_it->first;
            }
            else
                predecessors = paths_with_scores.begin()->first;
            ISM::PosePtr object_pose_ptr = predictPose(reference_pose_ptr,predecessors);
            addPointToPointCloud(object_pose_ptr, object.first, object.second);
        }
    }
}

void RandomPath::createSlotsMap()
{
    ROS_ASSERT(!paths_with_scores_map_.empty());
    ROS_ASSERT(slots_map_.empty());
    for (PathsWithScoresMap::iterator paths_it = paths_with_scores_map_.begin();
         paths_it != paths_with_scores_map_.end(); ++paths_it)
    {
        IsmObject object  = paths_it->first;
        PathsWithScores paths_with_scores = paths_it->second;
        ROS_ASSERT(paths_with_scores.size() > 0);
        if (paths_with_scores.size() > 1)
        {
            double sum = 0.0;
            for (PathsWithScores::iterator current_path_it = paths_with_scores.begin();
                 current_path_it != paths_with_scores.end();
                 ++current_path_it)
                sum += current_path_it->second;
            double current_sum = 0.0;
            Slots percentage_slots;
            for (PathsWithScores::iterator current_path_it = paths_with_scores.begin();
                 current_path_it != paths_with_scores.end();
                 ++current_path_it)
            {
                double score = current_path_it->second;
                double percentage_score = score / sum;
                current_sum += percentage_score;
                percentage_slots.push_back(current_sum);
            }
            ROS_ASSERT(percentage_slots.size() == paths_with_scores.size());
            if (percentage_slots[paths_with_scores.size() - 1] != 1.0)
                percentage_slots[paths_with_scores.size() - 1] = 1.0;
            slots_map_[object] = percentage_slots;
        }
        else
            ROS_DEBUG("No random election necessary: There is only one path");
    }
}
double RandomPath::calculatePathScore(IsmObjects path)
{
    double score = 1 / ( (double) path.size() );
    ROS_ASSERT_MSG(score < 1,
                   "There must be more than one predecessor, otherwise all objects are in the reference");
    return score;
}
}
