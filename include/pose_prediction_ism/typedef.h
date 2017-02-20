/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef POSE_PREDICTION_TYPEDEFS_H
#define POSE_PREDICTION_TYPEDEFS_H

#include <asr_msgs/AsrAttributedPointCloud.h>
#include <asr_msgs/AsrObject.h>
#include "ISM/common_type/RecognitionResult.hpp"

namespace pose_prediction_ism
{
/* ----------------- Primitive types ------------------  */
typedef unsigned int local_uint;
typedef std::string string;

/* ----------------- Containers ------------------  */
/* For more information about generic typedef look here:
 * http://stackoverflow.com/questions/3591024/typedef-alias-of-an-generic-class
 * NOTE: Needs C++ 11
 */
template<typename Key, typename Value>
using Map = std::map<Key, Value>;

template<typename T_1, typename T_2>
using Tupel = std::pair<T_1, T_2>;

template<typename Item>
using List = std::vector<Item>;
template<typename Item>
using Set = std::set<Item>;

/* ----------------- Boost shared ptr ------------------  */
template<typename Class>
using SharedPtr = boost::shared_ptr<Class>;

/* ----------------- Found object ------------------  */
typedef asr_msgs::AsrObject FoundObject;
typedef List<FoundObject> FoundObjects;

/* ----------------- AttributedPointCloud ------------------  */
typedef asr_msgs::AsrAttributedPointCloud AttributedPointCloud;
typedef asr_msgs::AsrAttributedPoint AttributedPoint;

/* ----------------- RecognitionResult ------------------  */
typedef ISM::RecognitionResult RecognitionResult;
typedef ISM::RecognitionResultPtr RecognitionResultPtr;
typedef List<RecognitionResult> RecognitionResults;
typedef List<RecognitionResultPtr> RecognitionResultPtrs;

}


#endif // POSE_PREDICTION_TYPEDEFS_H
