//
// Created by dmitrii on 02.05.2020.
//

#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif



class OPVSL{

public:
	
	cv_bridge::CvImageConstPtr img_color, depth;
	std::vector<double> track_times;
	std::chrono::time_point<std::chrono::steady_clock> tp_0;
	openvslam::system* SLAM;
	cv::Mat mask;
	ros::NodeHandle* nh;
	image_transport::ImageTransport* it;
	image_transport::Subscriber csub, dsub;
	message_filters::Subscriber<sensor_msgs::Image> image_sub;
	ros::Timer SLAM_timer;

    void color_image_callback(const sensor_msgs::ImageConstPtr& msg){
        ROS_INFO("color");
        this->img_color = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }

    void depth_image_callback(const sensor_msgs::ImageConstPtr& msg){
        ROS_INFO("depth");
        this->depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }

    void SLAM_push(const ros::TimerEvent&)
    {	
		ROS_INFO("timer");
        if (this->img_color and this->depth){
            const auto tp_1 = std::chrono::steady_clock::now();
            const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - this->tp_0).count();

            SLAM->feed_RGBD_frame(this->img_color->image, this->depth->image, timestamp, this->mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            this->track_times.push_back(track_time);
        }
    }

    OPVSL(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
        google::InitGoogleLogging(argv[0]);
        google::InstallFailureSignalHandler();
#endif
        // create options
        popl::OptionParser op("Allowed options");
        auto help = op.add<popl::Switch>("h", "help", "produce help message");
        auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
        auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
        auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
        auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
        auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
        auto map_db_path = op.add<popl::Value<std::string>>("", "map-db",
                                                            "store a map database at this path after SLAM", "");
        try {
            op.parse(argc, argv);
        }
        catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            std::cerr << std::endl;
            std::cerr << op << std::endl;
        }

        // check validness of options
        if (help->is_set()) {
            std::cerr << op << std::endl;
        }
        if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
            std::cerr << "invalid arguments" << std::endl;
            std::cerr << std::endl;
            std::cerr << op << std::endl;
        }

        // setup logger
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
        if (debug_mode->is_set()) {
            spdlog::set_level(spdlog::level::debug);
        } else {
            spdlog::set_level(spdlog::level::info);
        }

        // load configuration
        std::shared_ptr<openvslam::config> cfg;
        try {
            cfg = std::make_shared<openvslam::config>(setting_file_path->value());
        }
        catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
        }

#ifdef USE_GOOGLE_PERFTOOLS
        ProfilerStart("slam.prof");
#endif

        // run tracking
        if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
            // load the mask image
            this->mask = mask_img_path->value().empty() ? cv::Mat{} : cv::imread(mask_img_path->value(),
                                                                                 cv::IMREAD_GRAYSCALE);

            // build a SLAM system
            // startup the SLAM process
            this->SLAM = new openvslam::system(cfg, vocab_file_path->value());
            this->SLAM->startup();

            // create a viewer object
            // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
            pangolin_viewer::viewer viewer(cfg, SLAM, SLAM->get_frame_publisher(),
                                           SLAM->get_map_publisher());
#elif USE_SOCKET_PUBLISHER
            socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

            this->tp_0 = std::chrono::steady_clock::now();

            this->nh = new ros::NodeHandle;
            this->it = new image_transport::ImageTransport(*nh);

            this->csub = this->it->subscribe("color/image_raw", 1, &OPVSL::color_image_callback, this);
            this->dsub = this->it->subscribe("depth/image_raw", 1, &OPVSL::depth_image_callback, this);
            this->SLAM_timer = this->nh->createTimer(ros::Duration(0.016666667), &OPVSL::SLAM_push, this);

            // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
            std::thread thread([&]() {
                viewer.run();
                if (this->SLAM->terminate_is_requested()) {
                    // wait until the loop BA is finished
                    while (this->SLAM->loop_BA_is_running()) {
                        std::this_thread::sleep_for(std::chrono::microseconds(5000));
                    }
                    ros::shutdown();
                }
            });
#elif USE_SOCKET_PUBLISHER
            std::thread thread([&]() {
            publisher.run();
            if (this->SLAM.terminate_is_requested()) {
                // wait until the loop BA is finished
                while (this->SLAM.loop_BA_is_running()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(5000));
                }
                ros::shutdown();
            }
        });
#endif

            ros::spin();

            this->SLAM_timer.stop();

            // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
            viewer.request_terminate();
            thread.join();
#elif USE_SOCKET_PUBLISHER
            publisher.request_terminate();
            thread.join();
#endif

            // shutdown the SLAM process
            this->SLAM->shutdown();


            if (eval_log->is_set()) {
                // output the trajectories for evaluation
                this->SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
                this->SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
                // output the tracking times for evaluation
                std::ofstream ofs("track_times.txt", std::ios::out);
                if (ofs.is_open()) {
                    for (const auto track_time : this->track_times) {
                        ofs << track_time << std::endl;
                    }
                    ofs.close();
                }
            }

            if (!map_db_path->value().empty()) {
                // output the map database
                this->SLAM->save_map_database(map_db_path->value());
            }

            if (this->track_times.size()) {
                std::sort(this->track_times.begin(), this->track_times.end());
                const auto total_track_time = std::accumulate(this->track_times.begin(), this->track_times.end(), 0.0);
                std::cout << "median tracking time: " << this->track_times.at(track_times.size() / 2) << "[s]"
                          << std::endl;
                std::cout << "mean tracking time: " << total_track_time / this->track_times.size() << "[s]"
                          << std::endl;
            }
        } else {
            throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
        }

#ifdef USE_GOOGLE_PERFTOOLS
        ProfilerStop();
#endif

    }


};


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "rgbd_slam");
    OPVSL(argc, argv);
}

