//
// Created by dmitrii on 06.05.2020.
//

#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/data/landmark.h>
#include "openvslam/publish/map_publisher.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>

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


std::chrono::time_point<std::chrono::steady_clock> tp_0;
std::vector<double> track_times;
cv::Mat mask;
openvslam::system* SLAM;
std::shared_ptr<openvslam::publish::map_publisher> OVS_map;
ros::Publisher odometry_pub_;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
nav_msgs::OdometryPtr odom_msg_ (new nav_msgs::Odometry);
geometry_msgs::TransformStampedPtr map_tf (new geometry_msgs::TransformStamped);
geometry_msgs::TransformStampedPtr odom_tf (new geometry_msgs::TransformStamped);


void calculate_odometry(Eigen::Matrix<double, 4, 4> &cam_pose_, tf2_ros::Buffer* tf_buf,
        tf2_ros::TransformBroadcaster* tf_br){

    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rotation_matrix = cam_pose_.block(0, 0, 3, 3);
    Eigen::Vector3d translation_vector = cam_pose_.block(0, 3, 3, 1);

    tf2::Matrix3x3 tf_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

    tf2::Vector3 tf_translation_vector(translation_vector(0), translation_vector(1), translation_vector(2));

    tf2::Transform transform_tf(tf_rotation_matrix, tf_translation_vector);

    if (tf_buf->canTransform("chasiss", odom_tf->child_frame_id, ros::Time(0))){
        ROS_INFO_ONCE("Chasiss frame found, standart behaviour");
        auto bl_to_cam = tf_buf->lookupTransform("chasiss", odom_tf->child_frame_id, ros::Time(0));
        auto tmpr = tf2::Matrix3x3(tf2::Quaternion(bl_to_cam.transform.rotation.x, bl_to_cam.transform.rotation.y,
                bl_to_cam.transform.rotation.z, bl_to_cam.transform.rotation.w));

        tf2::Vector3 tmpt(bl_to_cam.transform.translation.x, bl_to_cam.transform.translation.y,
                bl_to_cam.transform.translation.z);
        transform_tf = tf2::Transform(tmpr, tmpt) * transform_tf;
        odom_tf->child_frame_id = "chasiss";
    }

    odom_msg_->pose.pose.orientation.x = transform_tf.getRotation().getX();
    odom_msg_->pose.pose.orientation.y = transform_tf.getRotation().getY();
    odom_msg_->pose.pose.orientation.z = transform_tf.getRotation().getZ();
    odom_msg_->pose.pose.orientation.w = transform_tf.getRotation().getW();


    odom_tf->transform.rotation.x = transform_tf.getRotation().getX();
    odom_tf->transform.rotation.y = transform_tf.getRotation().getY();
    odom_tf->transform.rotation.z = transform_tf.getRotation().getZ();
    odom_tf->transform.rotation.w = transform_tf.getRotation().getW();

    odom_msg_->pose.pose.position.x = transform_tf.getOrigin().getX();
    odom_msg_->pose.pose.position.y = transform_tf.getOrigin().getY();
    odom_msg_->pose.pose.position.z = transform_tf.getOrigin().getZ();

    odom_tf -> transform.translation.x = transform_tf.getOrigin().getX();
    odom_tf -> transform.translation.y = transform_tf.getOrigin().getY();
    odom_tf -> transform.translation.z = transform_tf.getOrigin().getZ();

    odom_msg_->header.stamp = ros::Time::now();
    odom_tf->header.stamp = ros::Time::now();
    odometry_pub_.publish(odom_msg_);

    tf_br->sendTransform(*odom_tf);
}


void convert_map(ros::Publisher* map_publisher, tf2_ros::Buffer* tf_buf, tf2_ros::TransformBroadcaster* tf_br){
    std::vector<openvslam::data::landmark*> landmarks;
    std::set<openvslam::data::landmark*> local_landmarks;
    OVS_map->get_landmarks(landmarks, local_landmarks);

    for (const auto lm : landmarks) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        if (local_landmarks.count(lm)) {
            continue;
        }
        const openvslam::Vec3_t pos_w = lm->get_pos_in_world();
        pcl::PointXYZ newPoint;
        newPoint.x = pos_w.x();
        newPoint.y = pos_w.y();
        newPoint.z = pos_w.z();
        map_cloud_msg->points.push_back(newPoint);
    }
    map_cloud_msg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    map_tf->header.stamp = ros::Time::now();

    if (tf_buf->canTransform("chasiss", odom_tf->child_frame_id, ros::Time(0)))
        tf_br->sendTransform(*map_tf);
    map_publisher->publish(map_cloud_msg);

}

void convert_local_map(ros::Publisher* local_map_publisher){
    std::vector<openvslam::data::landmark*> landmarks;
    std::set<openvslam::data::landmark*> local_landmarks;
    OVS_map->get_landmarks(landmarks, local_landmarks);
    local_map_cloud_msg->points.clear();
    for (const auto lm : local_landmarks) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const openvslam::Vec3_t pos_w = lm->get_pos_in_world();
        pcl::PointXYZ newPoint;
        newPoint.x = pos_w.x();
        newPoint.y = pos_w.y();
        newPoint.z = pos_w.z();
        local_map_cloud_msg->points.push_back(newPoint);
    }
    local_map_cloud_msg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    local_map_publisher->publish(local_map_cloud_msg);
}


void callback(sensor_msgs::ImageConstPtr color, sensor_msgs::ImageConstPtr depth,
        tf2_ros::TransformBroadcaster* tf_br, tf2_ros::Buffer* tf_buf,
        ros::Publisher* map_publisher, ros::Publisher* local_map_publisher){


	const auto tp_1 = std::chrono::steady_clock::now();
	const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();

	// input the current frame and estimate the camera pose
	auto cam_pose_ = SLAM->feed_RGBD_frame(cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8)->image,
	                                 cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_16UC1)->image,
	                                 timestamp, mask);
	const auto tp_2 = std::chrono::steady_clock::now();


	const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
	track_times.push_back(track_time);

	// TODO: I need multiprocessing here
	calculate_odometry(cam_pose_, tf_buf, tf_br);
    convert_map(map_publisher, tf_buf, tf_br);
    convert_local_map(local_map_publisher);

}


void rgbd_tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                   const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path) {
    // load the mask image
    mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    SLAM = new openvslam::system(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM->startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#endif

    OVS_map = SLAM->get_map_publisher();

    map_cloud_msg->header.frame_id = "map";
    local_map_cloud_msg->header.frame_id = "rs_camera_color_optical_frame";

    map_tf->header.frame_id = "map";
    map_tf->child_frame_id = "odom";

    map_tf->transform.rotation.x = 0;
    map_tf->transform.rotation.y = 0;
    map_tf->transform.rotation.z = 0;
    map_tf->transform.rotation.w = 1;
    map_tf->transform.translation.x = 0;
    map_tf->transform.translation.y = 0;
    map_tf->transform.translation.z = 0;

    odom_msg_->header.frame_id = "odom";
    odom_msg_->child_frame_id = "rs_camera_color_optical_frame";
    odom_tf->header.frame_id = "odom";
    odom_tf->child_frame_id = "rs_camera_color_optical_frame";

    tp_0 = std::chrono::steady_clock::now();

    // initialize this node
    ros::NodeHandle nh;
    //image_transport::ImageTransport it(nh);
    tf2_ros::TransformBroadcaster tf_br;
    tf2_ros::Buffer tf_Buffer;
    tf2_ros::TransformListener tf_list(tf_Buffer);

	message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, "color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth/image_raw", 1);
    ros::Publisher map_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("map_cloud", 4, true);
    ros::Publisher local_map_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("local_map_cloud", 4, false);
    odometry_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> subs_sync(color_sub, depth_sub, 6);
    subs_sync.registerCallback(boost::bind(&callback, _1, _2, &tf_br, &tf_Buffer, &map_publisher, &local_map_publisher));


    if (tf_Buffer.canTransform("chasiss", odom_tf->child_frame_id, ros::Time(0)))
        ROS_WARN("Couldn't find transform between chasiss and camera, only odom map->camera is published");


    // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
    std::thread thread([&]() {
        viewer.run();
        if (SLAM->terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM->loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#elif USE_SOCKET_PUBLISHER
    std::thread thread([&]() {
        publisher.run();
        if (SLAM->terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM->loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#endif

    ros::spin();

    // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
    viewer.request_terminate();
    thread.join();
#elif USE_SOCKET_PUBLISHER
    publisher.request_terminate();
    thread.join();
#endif

    // shutdown the SLAM process
    SLAM->shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM->save_map_database(map_db_path);
    }

    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
        std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
    }
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    ros::init(argc, argv, "openvslam");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
        rgbd_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}

