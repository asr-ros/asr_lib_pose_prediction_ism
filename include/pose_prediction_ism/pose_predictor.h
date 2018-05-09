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

#include <map>
#include <algorithm>
#include <time.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
/* ----------------- ROS ------------------  */
#include "ros/ros.h"

/* ----------------- ISM ------------------  */
#include <ISM/recognizer/Recognizer.hpp>

/* ----------------- Foreign datatypes  ------------------  */
#include "pose_prediction_ism/typedef.h"
#include "pose_prediction_ism/utility.h"

namespace pose_prediction_ism
{
  enum PredictorType
  {
    Shortest,
    Best,
    Random,
    PPNonNor,
    PPNor
  };
  class PosePredictor
  {
  protected:
    typedef std::pair<std::string, std::string> IsmObject;
    typedef std::vector<IsmObject> IsmObjects;
    typedef std::set<IsmObject> IsmObjectSet;

    PosePredictor(std::string database_filename,
                  std::string name_space,
                  PredictorType predictor_type);
    /**
       Destructor.
    */
    ~PosePredictor();
  public:

    /**
     * @brief predictUnfoundPoses
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
     * @param reference_pose
     * @param pattern_name
     */
    AttributedPointCloud virtual predictUnfoundPoses(ISM::PosePtr& reference_pose, std::string pattern_name, double percentage_of_records_for_prediction) = 0;
    inline void clearPointCloud()
    {
      attributed_point_cloud_.elements.clear();
    }


    /* ----------------- Getter ------------------  */
    std::string getMarkerNameSpace() const;
    AttributedPointCloud getAttributedPointCloud() const;

    /* ----------------- Setter ------------------  */
    void setFoundObjects(const FoundObjects &value);

    inline void setPredictionGenerationFactor(double prediction_generation_factor_){
        this->prediction_generation_factor_ = prediction_generation_factor_;
    }

    /* ----------------- Debug ------------------  */
    void traverseISM(std::string pattern_name, unsigned int level) ;

    inline void enableRandom(double sphere_radius, double max_projection_angle){
      USE_RANDOMS = true;
      SPHERE_RADIUS = sphere_radius;
      MAX_PROJECTION_ANGLE = max_projection_angle;

      udg_dist_ = new UniformDistributionGenerator(boost::mt19937(time(0)), boost::uniform_real<>(-SPHERE_RADIUS, SPHERE_RADIUS));
      udg_rot_ = new UniformDistributionGenerator(boost::mt19937(time(0)), boost::uniform_real<>(-MAX_PROJECTION_ANGLE / 2, MAX_PROJECTION_ANGLE / 2));

    }
    inline void disableRandom(){
      USE_RANDOMS = false;
      SPHERE_RADIUS = 0.0;
      MAX_PROJECTION_ANGLE = 0.0;
    }

    PredictorType getPredictorType() const;

  protected:
    typedef std::map<IsmObject , unsigned int> SizeMap;
    typedef boost::variate_generator<boost::mt19937, boost::uniform_real<> > UniformDistributionGenerator;
    UniformDistributionGenerator* udg_dist_;
    UniformDistributionGenerator* udg_rot_;

    SizeMap specifiers_size_map_;


    /* ----------------- ISM Attributes------------------  */
    ISM::PatternToObjectToVoteMap votes_;

    double prediction_generation_factor_ = 1;

        double average_votes_ = 0;

    AttributedPointCloud attributed_point_cloud_;
    /**
     * @brief RecognizerPredictionISM::addPointToPointCloud Adds an ISM Pose
     *  with object type and identifier as attributed point to the attributed point cloud if the
     *  number of hypotheses is higher than the threshold.
     * @param poseToAdd
     * @param type
     * @param identifier
     */
    inline void addPointToPointCloud(ISM::PosePtr pose_to_add, std::string type, std::string identifier)
    {
      attributed_point_cloud_.elements.push_back(ismPosePtr2attributedPoint(pose_to_add, type, identifier));
    }
    inline bool isFoundObject(IsmObject object)
    {
      return std::find_if(found_objects_.begin(),
			  found_objects_.end(),
			  [object](FoundObject foundObject)
			  {
                return (foundObject.type == object.first &&
                    foundObject.identifier == object.second);
			  }) != found_objects_.end();
    }
    inline bool isPointCloudEmpty()
    {
      return attributed_point_cloud_.elements.empty();
    }

    inline bool isReferenceObject(IsmObject object)
    {
      return object.first.find("_sub") != std::string::npos;
    }

    inline IsmObjectSet getObjectTypesAndIdsBelongingToPattern(std::string type)
    {
      return table_helper_->getObjectTypesAndIdsBelongingToPattern(type);
    }
    ISM::PosePtr predictPose(ISM::PosePtr reference_pose_ptr, IsmObjects path_to_object);

    virtual void createAttributedPointCloud(ISM::PosePtr reference_pose_ptr, double percentage_of_records_for_prediction) = 0;

  private:
    const std::string NAME_SPACE_;
    const PredictorType PREDICTOR_TYPE_;
    bool  USE_RANDOMS = false;
    double SPHERE_RADIUS = 0.0;
    double MAX_PROJECTION_ANGLE = 0.0;
    ISM:: TableHelperPtr table_helper_;
    FoundObjects found_objects_;

  };
  typedef SharedPtr<PosePredictor> PosePredictorPtr;
  std::ostream& operator<<(std::ostream &strm, const PosePredictor &p);
  std::ostream& operator<<(std::ostream &strm, const PosePredictorPtr &pPtr);
}
