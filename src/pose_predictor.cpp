/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/pose_predictor.h"
#include "ISM/utility/GeometryHelper.hpp"
#include "Eigen/Geometry"
namespace pose_prediction_ism
{
PosePredictor::PosePredictor(string database_filename,
                             string name_space,
                             PredictorType predictor_type):
    NAME_SPACE_(name_space),
    PREDICTOR_TYPE_(predictor_type)
{
  
    ROS_INFO_STREAM("Pose prediction is run with database " << database_filename);
    table_helper_ = ISM::TableHelperPtr(new ISM::TableHelper(database_filename));

    int counter = 0;

    for(string pattern_name : table_helper_->getModelPatternNames())
    {
        IsmObjectSet objects_in_pattern = table_helper_->getObjectTypesAndIdsBelongingToPattern(pattern_name);
        ISM::ObjectToVoteMap object_votes = table_helper_->getVoteSpecifiersForPatternAndObjects(pattern_name, objects_in_pattern);
        votes_[pattern_name] = object_votes;

        for (IsmObject object : objects_in_pattern)
        {
            specifiers_size_map_[object] = votes_
                    .at(pattern_name)
                    .at(object.first)
                    .at(object.second).size();

            average_votes_ += specifiers_size_map_[object];

            break;
        }
        counter ++;
    }

    average_votes_ = (average_votes_/counter);

    ROS_INFO("PosePredictor: averageVotes: %f", average_votes_);
}

PosePredictor::~PosePredictor()
{
}

void PosePredictor::traverseISM(string pattern_name, local_uint level)
{
    string s = "";
    for (local_uint i = 0; i < level; ++i)
        s += "-";
    s += ">";
    ROS_DEBUG_STREAM(s << " patternName: " << pattern_name);
    IsmObjectSet objects_in_pattern =
            table_helper_->getObjectTypesAndIdsBelongingToPattern(pattern_name);
    for (IsmObject object : objects_in_pattern)
        traverseISM(object.first, level + 1);
}

ISM::PosePtr PosePredictor::predictPose(ISM::PosePtr reference_pose_ptr,
                                        IsmObjects path_to_object)
{
    ROS_ASSERT_MSG(path_to_object.size() > 1 ,
                   "There must be more than one predecessor, otherwise all objects are in the reference");
    ISM::PosePtr current_pose(reference_pose_ptr);
    for (IsmObjects::iterator predecessors_it = path_to_object.begin();
         predecessors_it + 1 != path_to_object.end();
         ++predecessors_it)
    {
        using namespace ISM;
        List<VoteSpecifierPtr> specifiers = votes_
                .at(predecessors_it->first)
                .at((predecessors_it + 1)->first)
                .at((predecessors_it + 1)->second);
        local_uint index = rand() % specifiers.size();
        VoteSpecifierPtr specifier = specifiers.at(index);
        PointPtr absolute_position = GeometryHelper::getSourcePoint(current_pose,
                                                                specifier->refToObjectQuat,
                                                                specifier->radius);

        Eigen::Quaterniond q;
        if(USE_RANDOMS){
            double r_x, r_y, r_z;
            do{
                r_x = udg_dist_->operator ()();
                r_y = udg_dist_->operator ()();
                r_z = udg_dist_->operator ()();
            }while(std::sqrt(r_x * r_x + r_y * r_y + r_z * r_z) > SPHERE_RADIUS);
            absolute_position->eigen.x() = absolute_position->eigen.x() + r_x;
            absolute_position->eigen.y() = absolute_position->eigen.y() + r_y;
            absolute_position->eigen.z() = absolute_position->eigen.z() + r_z;

            r_x = udg_rot_->operator ()();
            r_y = udg_rot_->operator ()();
            r_z = udg_rot_->operator ()();

            q = Eigen::AngleAxisd(GeometryHelper::deg2rad(r_x), Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(GeometryHelper::deg2rad(r_y),  Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(GeometryHelper::deg2rad(r_z), Eigen::Vector3d::UnitZ());

    }
    PosePtr absolute_pose = GeometryHelper::getSourcePose(current_pose, absolute_position, specifier->refToObjectPoseQuat);
    current_pose = absolute_pose;

    if(USE_RANDOMS){
        current_pose->quat = ISM::GeometryHelper::eigenQuatToQuat(ISM::GeometryHelper::quatToEigenQuat(current_pose->quat) * q);
    }
}
    return current_pose;
}

/* ----------------- Setters ------------------  */
void PosePredictor::setFoundObjects(const FoundObjects &value)
{
    found_objects_ = value;
}
/* ----------------- Getters ------------------  */

string PosePredictor::getMarkerNameSpace() const
{
    return NAME_SPACE_;
}

AttributedPointCloud PosePredictor::getAttributedPointCloud() const
{
    return attributed_point_cloud_;
}
PredictorType PosePredictor::getPredictorType() const
{
    return PREDICTOR_TYPE_;
}

std::ostream &operator <<(std::ostream &strm, const PosePredictorPtr &pPtr)
{
    return strm << (*pPtr);
}

std::ostream&operator <<(std::ostream &strm, const PosePredictor &p)
{
    string type_string;
    switch (p.getPredictorType())
    {
    case Shortest:   type_string = "ShortestPath"; break;
    case Best:       type_string = "BestPath";break;
    case Random:     type_string = "RandomPath";break;
    case PPNonNor:   type_string = "PaperPredictionNonNormalized";break;
    case PPNor:      type_string = "PaperPredictionNormalized";break;
    default:         type_string = "Unknown Predictor";
    }
    return strm << type_string;
}
}











