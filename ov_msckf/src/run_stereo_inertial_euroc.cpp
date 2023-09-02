/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>

#include <memory>

#include <functional>
#include <numeric>
#include <string>
#include <vector>

#ifdef ENABLE_ROS
    #include <ros/ros.h>
    #include <rosbag/bag.h>
    #include <rosbag/view.h>
    #include <sensor_msgs/Image.h>
    #include <sensor_msgs/Imu.h>

    #include "ros/ROS1Visualizer.h"
#endif

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"


using namespace std;
using namespace ov_msckf;


int run_ros(int argc, char **argv);

int run_rosfree(int argc, char **argv);

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);


std::shared_ptr<ov_msckf::VioManager> sys;
#ifdef ENABLE_ROS
std::shared_ptr<ROS1Visualizer> viz;
#endif


int main(int argc, char **argv) {
#ifdef ENABLE_ROS
    return run_ros(argc, argv);
#else
    return run_rosfree(argc, argv);
#endif
}

int run_ros(int argc, char **argv) {
    return 0;
}

int run_rosfree(int argc, char **argv) {
    if(argc < 4) {
        cerr << endl << "Usage: ./run_stereo_inertial_euroc path_to_config path_to_sequence_folder path_to_times_file" << endl;
        return 1;
    }

    const int num_seq = (argc - 2)/2;
    cout << "num_seq = " << num_seq << endl;

    string config_path = argv[1];
    cout << "config_path:     " << config_path << endl;

    string seq_folder = argv[2];
    cout << "seq_folder:      " << seq_folder << endl;

    string times_file_name = argv[3];
    cout << "times_file_name: " << times_file_name << endl;

    //==========================================================================

    // Load the config
    auto parser = std::make_shared<ov_core::YamlParser>(config_path);

    // Verbosity
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    // Create our VIO system
    ov_msckf::VioManagerOptions params;
    params.print_and_load(parser);
    // params.num_opencv_threads = 0; // uncomment if you want repeatability
    // params.use_multi_threading_pubs = 0; // uncomment if you want repeatability
    params.use_multi_threading_subs = false;
    sys = std::make_shared<ov_msckf::VioManager>(params);

    // Ensure we read in all parameters required
    if (!parser->successful()) {
        PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
        std::exit(EXIT_FAILURE);
    }

    //==========================================================================
    
    // Load all sequences:
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq, 0);

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    int seq;    
    for (seq = 0; seq < num_seq; seq++) {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2 * seq) + 2]);
        string pathTimeStamps(argv[(2 * seq) + 3]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << " LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq] <= 0)||(nImu[seq] <= 0)) {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

    //==========================================================================

    // Run
    cv::Mat imLeft, imRight;

    std::ofstream outfile;
    outfile.open("/tmp/traj_estimate_exe.txt");
    outfile << "# timestamp(s) tx ty tz qx qy qz qw" << std::endl;

    // Seq loop
    for (seq = 0; seq < num_seq; seq++) {
        // Vector for tracking time statistics
        vector<double> vTimesTrack;
        vTimesTrack.resize(nImages[seq], 0.0);

        // Image loop
        for(int ni = 0; ni < nImages[seq]; ni++) {
            std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();


            // IMU processing
            // Load imu measurements from previous frame
            if(ni > 0) {
                while(vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]) {
                    // convert into correct format
                    ov_core::ImuData message;
                    message.timestamp = vTimestampsImu[seq][first_imu[seq]];
                    message.wm << vGyro[seq][first_imu[seq]].x, vGyro[seq][first_imu[seq]].y, vGyro[seq][first_imu[seq]].z;
                    message.am << vAcc[seq][first_imu[seq]].x, vAcc[seq][first_imu[seq]].y, vAcc[seq][first_imu[seq]].z;

                    // send it to our VIO system
                    sys->feed_measurement_imu(message);
                    
                    first_imu[seq]++;
                }
            }


            // Camera processing
            {
                // Read left and right images from file
                imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
                imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);

                if(imLeft.empty()) {
                    cerr << endl << "Failed to load image at: "
                        << string(vstrImageLeft[seq][ni]) << endl;
                    return 1;
                }

                if(imRight.empty()) {
                    cerr << endl << "Failed to load image at: "
                        << string(vstrImageRight[seq][ni]) << endl;
                    return 1;
                }

                double tframe = vTimestampsCam[seq][ni];


                // Create the measurement
                ov_core::CameraData message;
                message.timestamp = tframe;
                message.sensor_ids.push_back(0);
                message.sensor_ids.push_back(1);
                message.images.push_back(imLeft.clone());
                message.images.push_back(imRight.clone());

                // Load the mask if we are using it, else it is empty
                // TODO: in the future we should get this from external pixel segmentation
                if (sys->get_params().use_mask) {
                    message.masks.push_back(sys->get_params().masks.at(0));
                    message.masks.push_back(sys->get_params().masks.at(1));
                } else {
                    // message.masks.push_back(cv::Mat(imLeft.rows, imLeft.cols, CV_8UC1, cv::Scalar(255)));
                    message.masks.push_back(cv::Mat::zeros(imLeft.rows, imLeft.cols, CV_8UC1));
                    message.masks.push_back(cv::Mat::zeros(imRight.rows, imRight.cols, CV_8UC1));
                }

                sys->feed_measurement_camera(message);
            }

            std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
            vTimesTrack[ni] = std::chrono::duration<double>(t2 - t1).count() * 1000;
            cout << "processing image: " << ni << " using " << vTimesTrack[ni] << " ms" << endl;


            // State processing
            if (sys->initialized()) {
                std::shared_ptr<ov_msckf::State> state = sys->get_state();

                // Save camera trajectory
                // timestamp
                outfile.precision(9);
                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile << state->_timestamp << " ";

                // pose
                outfile.precision(6);
                outfile << state->_imu->pos()(0) << " " << state->_imu->pos()(1) << " " << state->_imu->pos()(2) << " " \
                        << state->_imu->quat()(0) << " " << state->_imu->quat()(1) << " " << state->_imu->quat()(2) << " " << state->_imu->quat()(3);

                outfile << std::endl;
            }


            // Wait to load the next frame
            // usleep(33 * 1e6); // 1e6
        } // per img

        double t_avg = std::accumulate(vTimesTrack.begin(), vTimesTrack.end(), 0.0) / vTimesTrack.size();
        cout << endl << "seq " << seq << ": " << endl;
        cout << "\tt_avg: " << t_avg << " ms" << endl;
        cout << "\tt_max: " << *std::max_element(vTimesTrack.begin(), vTimesTrack.end()) << " ms" << endl;
        cout << "\tt_min: " << *std::min_element(vTimesTrack.begin(), vTimesTrack.end()) << " ms" << endl;


        // if(seq < num_seq - 1) {
        //     cout << "Changing the dataset" << endl;
        //     SLAM.ChangeDataset();
        // }
    } // per sequence


    cout << "Done!" << endl;
    return 0;
}


void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}


void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
