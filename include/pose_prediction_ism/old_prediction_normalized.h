/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once
#include "pose_prediction_ism/pose_predictor.h"
namespace pose_prediction_ism
{
    class PaperPredictionNormalized : public PosePredictor
    {
    public:
        PaperPredictionNormalized(std::string dbfileName);
        ~PaperPredictionNormalized();
        /**
         * @brief RecognizerPredictionISM::calcUnfoundPoses
         * Processes the following steps for each object in the objectpattern set.
         *  <ul>
         *      <li>Look whether we have an unfound object (reference object can never be <br>
         *          found as they do not exist). Else ignore vote, as we already know itsattributedPoint.pose.
         *      <li>If the object was not found
         *      <ul>
         *          <li> Get all votes that fit to combination of this reference <br>
         *              (to reference and non-reference objects) and unfound object.
         *          <li> If we have a reference object
         *          <ul>
         *              <li>search under it for unknown object.
         *          </ul>
         *          <li> If we have a non-reference object
         *          <ul>
         *              <li>we have not found and save its position.
         *          </ul>
         *          <li> Save that we found this object as missing in our currently considered scene hypotheses.
         *      </ul>
         *  </ul>
         *
         * @param referencePose
         * @param patternName
         * @param weight
         * @param depth
         */
        AttributedPointCloud predictUnfoundPoses(ISM::PosePtr& referencePose, string patternName, double percentage_of_records_for_prediction);

    private:
        /**
         * @brief referenceObjectProcessing Processes a reference object.
         *  Processes the following steps.
         *  <ul>
         *      <li> Calculate absolute pose of unknown object.
         *      <li> Calculate full pose for next call of this function.
         *      <li> Check whether this reference object is identical to <br>
         *          the reference of the ism in which it is object.
         *      If this reference object is equal to the reference of the ism in which it resides.
         *      <ul>
         *          <li> Every single vote for a hypotheses in specifier has to be repreated <br>
         *              for all redundancies in lower parts of the tree.
         *          <li> One level higher in the scene model hierarchy, object pose hypotheses <br>
         *              should be replicated for every vote in specifiers as well as every time <br>
         *              this function would have been called. Reference object in this ism gets <br>
         *              reference of next higher ism.
         *          <li> If position of this object in scene is equal to pose of reference, then <br>
         *              it has been chosen as reference and its pose will be equal to the reference <br>
         *               through all votes. So we do not need to process any other votes here.
         *      </ul>
         *      If this reference object is not equal to the reference of the ism in which it resides.
         *      <ul>
         *          <li> One level higher in the scene model hierarchy, object pose hypotheses <br>
         *              should be replicated for every time this function would have been called.
         *      </ul>
         *  </ul>
         * @param objectType The object type
         * @param specifier The vote specifier
         * @param referencePose The reference pose of this function call
         * @param weight The weight of this call
         * @param specifiersSize The size of the specifier container.
         * @return Returns true, if we do not have to process further
         */
        bool referenceObjectProcessing(std::string objectType,
                                       ISM::VoteSpecifierPtr specifier,
                                       ISM::PosePtr &referencePose,
                                       local_uint weight,
                                       local_uint specifiersSize,
                                       local_uint level);
        void nonReferenceObjectProcessing(IsmObject object,
                                          ISM::VoteSpecifierPtr specifier,
                                          ISM::PosePtr &referencePose,
                                          local_uint weight);
        void calculateRecursiveUnfoundPoses(ISM::PosePtr& referencePose,
                                            std::string patternName,
                                            local_uint weight,
                                            local_uint level);
        void createAttributedPointCloudFromMap(double numberOfSpecifiers);
        void createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction);
        std::map<IsmObject , std::vector<ISM::PosePtr>> objectPoseMap;
        SizeMap objectSizeMap;
    };
    typedef boost::shared_ptr<PaperPredictionNormalized> PaperPredictionNormalizedPtr;
}
