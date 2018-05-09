/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/old_prediction_non_normalized.h"
#include <ISM/utility/GeometryHelper.hpp>
namespace pose_prediction_ism
{
PaperPredictionNonNormalized::PaperPredictionNonNormalized(std::string dbfileName):
    PosePredictor(dbfileName,"non_normalized", PPNonNor)
{
}
PaperPredictionNonNormalized::~PaperPredictionNonNormalized()
{

}

  AttributedPointCloud PaperPredictionNonNormalized::predictUnfoundPoses(ISM::PosePtr &referencePose, std::string patternName, double numberOfSpecifiers)
{

  ROS_INFO_STREAM("Pose prediction is run for scene " << patternName << ".");
  ROS_INFO_STREAM("It is run for a reference with the pose " << referencePose << ".");

  clearPointCloud();
  calculateRecursiveUnfoundPoses(referencePose,patternName, 1);
  return attributed_point_cloud_;

}

void PaperPredictionNonNormalized::calculateRecursiveUnfoundPoses(ISM::PosePtr &referencePose,
                                                                  std::string patternName,
                                                                  unsigned int weight)
{
    ROS_DEBUG_STREAM("" << referencePose);
    IsmObjectSet objectsInPattern = getObjectTypesAndIdsBelongingToPattern(patternName);
    for (IsmObject object : objectsInPattern)
    {
        ROS_DEBUG("Taking votes for object %s", object.first.c_str());
        if (!isFoundObject(object))

        {
            //ROS_DEBUG("Object %s was NOT found yet", object.first.c_str());
            std::vector<ISM::VoteSpecifierPtr> specifiers = votes_.at(patternName).at(object.first).at(object.second);
            ROS_DEBUG_STREAM("object: " << object.first << std::endl << "specifiers size" << specifiers.size());
            if(isReferenceObject(object))
            {
                for (auto& specifier : specifiers)
                {
                    bool skipOtherSpecifiers = referenceObjectProcessing(object.first,
                                                                         specifier,
                                                                         referencePose,
                                                                         weight,
                                                                         specifiers.size());

                    if (skipOtherSpecifiers) break;
                }
            }
            else
            {
                for (auto& specifier : specifiers)
                {
                    bool skipOtherSpecifiers = nonReferenceObjectProcessing(object,
                                                                            specifier,
                                                                            referencePose,
                                                                            weight,
                                                                            specifiers.size());
                   if (skipOtherSpecifiers) break;
                }
            }
        }
        else
        {
            ROS_DEBUG("Object %s was already found, no further proceeding", object.first.c_str());
        }
    }
}
bool PaperPredictionNonNormalized::nonReferenceObjectProcessing(IsmObject object, ISM::VoteSpecifierPtr specifier, ISM::PosePtr &referencePose, unsigned int weight, unsigned int specifiersSize)
{
    using namespace ISM;
    PointPtr absPosition = GeometryHelper::getSourcePoint(referencePose,
                                                      specifier->refToObjectQuat,
                                                      specifier->radius);
    PosePtr absPose = GeometryHelper::getSourcePose(referencePose,
                                                   absPosition,
                                                   specifier->refToObjectPoseQuat);
    if(ISM::GeometryHelper::poseEqual(referencePose, absPose))
    {

        unsigned int numberOfHypotheses = weight * specifiersSize;
        addPointToPointCloudWithNumber(absPose, object, numberOfHypotheses);
        return true;
    }
    else
    {
        addPointToPointCloudWithNumber(absPose, object, 1);
        return false;
    }
}
bool PaperPredictionNonNormalized::referenceObjectProcessing(std::string objectType,
                                                             ISM::VoteSpecifierPtr specifier,
                                                             ISM::PosePtr &referencePose,
                                                             unsigned int weight,
                                                             unsigned int specifiersSize)
{
    //ROS_INFO("Object %s is a reference object", objectType.c_str());
    using namespace ISM;
    PointPtr absPosition = GeometryHelper::getSourcePoint(referencePose,
                                                      specifier->refToObjectQuat,
                                                      specifier->radius);
    PosePtr absPose = GeometryHelper::getSourcePose(referencePose,
                                                   absPosition,
                                                   specifier->refToObjectPoseQuat);
    if(GeometryHelper::poseEqual(referencePose, absPose))
    {
        ROS_DEBUG("Object %s  is identical to the reference of the ism in which it is object", objectType.c_str());
        unsigned int numberOfHypotheses = specifiersSize * weight;
        //ROS_DEBUG_STREAM("numberOfHypotheses: " << numberOfHypotheses << " weight: " << weight << " percentage: " << percentage);
        calculateRecursiveUnfoundPoses(absPose, objectType, numberOfHypotheses);
        return true;
    }
    else
    {
        //ROS_DEBUG("Object %s  is  NOT identical to the reference of the ism in which it is object", objectType.c_str());
        calculateRecursiveUnfoundPoses(absPose, objectType, weight);
        return false;
    }
}

void PaperPredictionNonNormalized::createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction)
{

}
void PaperPredictionNonNormalized::addPointToPointCloudWithNumber(ISM::PosePtr poseToAdd, IsmObject object, unsigned int numberOfHypotheses)
{
    addPointToPointCloud(poseToAdd, object.first, object.second);
}



}
