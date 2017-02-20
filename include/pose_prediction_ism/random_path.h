/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RANDOMPATHNORMALIZATION_H
#define RANDOMPATHNORMALIZATION_H

#include "pose_prediction_ism/predictor_with_score.h"

namespace pose_prediction_ism
{
class RandomPath: public PredictorWithScore
{
public:
    RandomPath(string database_filename);
    ~RandomPath();
    AttributedPointCloud predictUnfoundPoses(ISM::PosePtr &reference_pose, string pattern_name, double percentage_of_records_for_prediction);

private:
    typedef List<double> Slots;
    typedef Map<IsmObject ,  Slots> SlotsMap;
    typedef List<PathWithScore> PathsWithScores;
    typedef Map<IsmObject , PathsWithScores> PathsWithScoresMap;

    /* ----------------- Private members ------------------  */
    PathsWithScoresMap paths_with_scores_map_;
    SlotsMap slots_map_;

    /* ----------------- Functions ------------------  */
    void createPathsWithScoresMap(string type, IsmObjects predecessors);
    void createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction);
    void createSlotsMap();
    double calculatePathScore(IsmObjects path);
};
typedef SharedPtr<RandomPath> RandomPathPtr;

}



#endif // RANDOMPATHNORMALIZATION_H
