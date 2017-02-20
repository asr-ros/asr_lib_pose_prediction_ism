/**

Copyright (c) 2016, Heizmann Heinrich, Heller Florian, Meißner Pascal, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>
#include <pose_prediction_ism/pose_predictor.h>
#include <pose_prediction_ism/shortest_path.h>
#include <pose_prediction_ism/old_prediction_non_normalized.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

class PosePredictionTester{
private:
    std::ofstream output_avg_file_, output_total_file_;
    pose_prediction_ism::PaperPredictionNonNormalizedPtr nnpredictor_;
    pose_prediction_ism::ShortestPathPtr sppredictor_;

    ISM::PosePtr id_pose;
    int iteration_;

    void runTest(std::string path_to_db){
        ROS_INFO("Run Test for database \"%s\"", path_to_db.c_str());


        ISM::TableHelper* helper = new ISM::TableHelper(path_to_db);
        std::vector<std::string> pattern_names = helper->getRecordedPatternNames();

        int set_count = (int) helper->getSetIds().size();
        int object_count = (int) helper->getRecordedObjectSet(1)->objects.size();
        //calculate Treesize from modelpatternname
        int treesize = 0;
        std::vector<std::string> model_pattern_names = helper->getModelPatternNames();
        for(size_t j = 0; j < model_pattern_names.size(); j++){
            std::vector<std::string> tokens;
            boost::split_regex(tokens, model_pattern_names[j], boost::regex("_sub"));
            treesize = std::max(treesize, (int)tokens.size());
        }

        for(size_t j = 0; j < pattern_names.size(); j++){
            std::string pattern_name = pattern_names[j];
            double nn_time = 0;
            double shortest_path_time = 0;
            for(int i = 0; i < iteration_; i++){
                double nn_time_new;
                double shortest_path_time_new;
                ROS_INFO("starting iteration %d/%d for %d-%d", i, iteration_, object_count, set_count);

                ROS_INFO("nnpredictor start");
                nnpredictor_ = pose_prediction_ism::PaperPredictionNonNormalizedPtr(new pose_prediction_ism::PaperPredictionNonNormalized(path_to_db));
               auto t0 = std::chrono::high_resolution_clock::now();
                nnpredictor_->predictUnfoundPoses(id_pose, pattern_name, 1.0);
               auto t1 = std::chrono::high_resolution_clock::now();
                nn_time_new = 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();
                ROS_INFO("nnpredictor end time: %f", nn_time_new);

                ROS_INFO("sppredictor start");
                sppredictor_ = pose_prediction_ism::ShortestPathPtr(new pose_prediction_ism::ShortestPath(path_to_db));
               auto t2 = std::chrono::high_resolution_clock::now();
                sppredictor_->predictUnfoundPoses(id_pose, pattern_name, 1.0);
              auto  t3 = std::chrono::high_resolution_clock::now();
                shortest_path_time_new = 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count();

                ROS_INFO("sppredictor end time: %f", shortest_path_time_new);

                output_total_file_  << i << "," << object_count << "," << set_count << "," << treesize << "," << nn_time_new << "," << shortest_path_time_new << std::endl;
                nn_time += nn_time_new;
                shortest_path_time += shortest_path_time_new;
            }
            output_avg_file_ << object_count << "," << set_count << "," << treesize << "," << (nn_time / iteration_) << "," << (shortest_path_time / iteration_) << std::endl;

        }

    }

public:
    PosePredictionTester(){
        ros::NodeHandle nh("/pose_prediction_test");
        id_pose = ISM::PosePtr(new ISM::Pose(new ISM::Point(0, 0, 0), new ISM::Quaternion(0, 0, 0, 1)));

        //Read Params
        ROS_INFO("Pose Prediction Tester started");
        std::string input_path;
        if(!nh.getParam("input_path", input_path)){
            ROS_ERROR("Faild to read input path!");
            return;
        }
        ROS_INFO("Input Path: %s", input_path.c_str());

        std::string output_path;
        if(!nh.getParam("output_path", output_path)){
            ROS_ERROR("Faild to read output_path!");
            return;
        }
        ROS_INFO("Output path: %s", output_path.c_str());

        if(!nh.getParam("iteration", iteration_)){
            ROS_WARN("Faild to load param iteration, defauting to 10!");
            iteration_ = 10;
        }
        ROS_INFO("Iteratrion: %d", iteration_);

        output_avg_file_.open (output_path+ "pp_avg.csv");
        output_avg_file_ << "objects" << "," << "sets" << "," << "treesize" << "," << "nonnormalized" << "," << "shortestpath" << std::endl;
        output_total_file_.open (output_path + "pp_total.csv");
        output_total_file_<< "test_run" << "," << "objects" << "," << "sets" << "," << "treesize" << "," << "nonnormalized" << "," << "shortestpath" << std::endl;
        boost::filesystem::path path_to_db_dir(input_path);
        std::vector<boost::filesystem::path> paths;
        std::copy(boost::filesystem::directory_iterator(path_to_db_dir), boost::filesystem::directory_iterator(), std::back_inserter(paths));
        std::sort(paths.begin(), paths.end());
        for (std::vector<boost::filesystem::path>::const_iterator i(paths.begin()), i_end(paths.end()); i != i_end; ++i)
        {
            if (!boost::filesystem::is_directory(*i))
            {
                runTest(i->string());

            }
        }
        output_avg_file_.close();
        output_total_file_.close();
    }

};



int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_prediction_test");
    new PosePredictionTester();
}
