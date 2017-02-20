/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pose_prediction_ism/utility.h"
#include <ros/ros.h>
namespace pose_prediction_ism
{
    ISM::Point attributedPoint2ismPoint(asr_msgs::AsrAttributedPoint attributedPoint)
    {
        ISM::Point p;
        p.eigen.x() = attributedPoint.pose.position.x;
        p.eigen.y() = attributedPoint.pose.position.y;
        p.eigen.z() = attributedPoint.pose.position.z;
        return p;
    }
    asr_msgs::AsrAttributedPoint ismPosePtr2attributedPoint(ISM::PosePtr ismPosePtr,
                                                               std::string type,
                                                               std::string identifier   )
    {
        asr_msgs::AsrAttributedPoint attributedPoint;
        attributedPoint.type = type;
        attributedPoint.identifier = identifier;
        attributedPoint.pose.position.x = ismPosePtr->point->eigen.x();
        attributedPoint.pose.position.y = ismPosePtr->point->eigen.y();
        attributedPoint.pose.position.z = ismPosePtr->point->eigen.z();
        attributedPoint.pose.orientation.w = ismPosePtr->quat->eigen.w();
        attributedPoint.pose.orientation.x = ismPosePtr->quat->eigen.x();
        attributedPoint.pose.orientation.y = ismPosePtr->quat->eigen.y();
        attributedPoint.pose.orientation.z = ismPosePtr->quat->eigen.z();
        return attributedPoint;
    }
}
