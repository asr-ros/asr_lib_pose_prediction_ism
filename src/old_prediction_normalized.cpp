/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/old_prediction_normalized.h"
#include <ISM/utility/GeometryHelper.hpp>
namespace pose_prediction_ism
{
PaperPredictionNormalized::PaperPredictionNormalized(std::string dbfileName):
    PosePredictor::PosePredictor(dbfileName, "normalized", PPNor)
{
}

PaperPredictionNormalized::~PaperPredictionNormalized()
{

}
void PaperPredictionNormalized::nonReferenceObjectProcessing(IsmObject object, ISM::VoteSpecifierPtr specifier, ISM::PosePtr &referencePose, unsigned int weight)
{
    using namespace ISM;
    PointPtr absPosition = GeometryHelper::getSourcePoint(referencePose,
                                                      specifier->refToObjectQuat,
                                                      specifier->radius);
    PosePtr absPose = GeometryHelper::getSourcePose(referencePose,
                                                   absPosition,
                                                   specifier->refToObjectPoseQuat);
    for(unsigned int i = 0; i < weight; i++)
        objectPoseMap[object].push_back(absPose);
}

AttributedPointCloud PaperPredictionNormalized::predictUnfoundPoses(ISM::PosePtr &referencePose,
								    std::string patternName, double numberOfSpecifiers)
{
    objectPoseMap.clear();
    clearPointCloud();
    calculateRecursiveUnfoundPoses(referencePose,patternName,1,0);
    ROS_ASSERT(isPointCloudEmpty());
    ROS_ASSERT(!objectPoseMap.empty());
    ROS_ASSERT(!objectSizeMap.empty());
    createAttributedPointCloudFromMap(numberOfSpecifiers);
    return attributed_point_cloud_;
}

bool PaperPredictionNormalized::referenceObjectProcessing(std::string objectType,
                                                          ISM::VoteSpecifierPtr specifier,
                                                          ISM::PosePtr &referencePose,
                                                          unsigned int weight,
                                                          unsigned int specifiersSize,
                                                          unsigned int level)
{
    //ROS_INFO("Object %s is a reference object", objectType.c_str());
    using namespace ISM;
    PointPtr absPosition = GeometryHelper::getSourcePoint(referencePose,
                                                      specifier->refToObjectQuat,
                                                      specifier->radius);
    PosePtr absPose = GeometryHelper::getSourcePose(referencePose,
                                                   absPosition,
                                                   specifier->refToObjectPoseQuat);
    unsigned int nextLevel = level + 1;
    if(GeometryHelper::poseEqual(referencePose, absPose))
    {
        ROS_DEBUG("Object %s  is identical to the reference of the ism in which it is object", objectType.c_str());
        unsigned int numberOfHypotheses = specifiersSize * weight;
        //ROS_DEBUG_STREAM("numberOfHypotheses: " << numberOfHypotheses << " weight: " << weight << " percentage: " << percentage);
        calculateRecursiveUnfoundPoses(absPose, objectType, numberOfHypotheses, nextLevel);
        return true;
    }
    else
    {
        //ROS_DEBUG("Object %s  is  NOT identical to the reference of the ism in which it is object", objectType.c_str());
        calculateRecursiveUnfoundPoses(absPose, objectType, weight, nextLevel);
        return false;
    }
}


void PaperPredictionNormalized::createAttributedPointCloudFromMap(double numberOfSpecifiers)
{
    ROS_DEBUG_STREAM("objectMap contains:" << std::endl);
    for (std::map<IsmObject , std::vector<ISM::PosePtr>>::iterator it=objectPoseMap.begin(); it!=objectPoseMap.end(); ++it)
    {
        IsmObject object = it->first;
        unsigned int posesSize = it->second.size();
        unsigned int specifiersSize = objectSizeMap[object];
        unsigned int threshold = round(numberOfSpecifiers * specifiersSize);
        for (unsigned int i = 0; i < threshold; ++i)
        {
            unsigned int index = rand() % posesSize;
            addPointToPointCloud(it->second.at(index), object.first, object.second);
        }
        it->second.clear();
    }
    objectPoseMap.clear();
    objectSizeMap.clear();
}

void PaperPredictionNormalized::createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction)
{
}


void PaperPredictionNormalized::calculateRecursiveUnfoundPoses(ISM::PosePtr &referencePose,
                                                               std::string patternName,
                                                               unsigned int weight,
                                                               unsigned int level)
{
    IsmObjectSet objectsInPattern = getObjectTypesAndIdsBelongingToPattern(patternName);
    for (IsmObject object : objectsInPattern)
    {
        std::vector<ISM::VoteSpecifierPtr> specifiers = votes_.at(patternName).at(object.first).at(object.second);
        if(isReferenceObject(object))
        {
            for (unsigned int i = 0; i < specifiers.size(); i++)
            {
                bool skipOtherSpecifiers = referenceObjectProcessing(object.first,
                                                                     specifiers.at(i),
                                                                     referencePose,
                                                                     weight,
                                                                     specifiers.size(),
                                                                     level);
                if (skipOtherSpecifiers){
                    ROS_DEBUG_STREAM("Skipping further votes for object " << "(" << object.first << "," << object.second << ")" << ".");
                    break;
                }

            }
        }
        else
        {
            objectSizeMap[object] = specifiers.size();
            for (unsigned int i = 0; i < specifiers.size(); i++)
                nonReferenceObjectProcessing(object,
                                             specifiers.at(i),
                                             referencePose, weight);
        }
    }
}
}
