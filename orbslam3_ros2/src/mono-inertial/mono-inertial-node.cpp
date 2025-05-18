#include "mono-inertial-node.hpp"
#include "utility.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

// Max buffer size to prevent memory overflow
const int MAX_BUFFER_SIZE = 5000;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2"), 
    SLAM_(pSLAM), 
    syncThread_(new std::thread(&MonoInertialNode::SyncWithImu, this))
{
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera", 300, std::bind(&MonoInertialNode::GrabImage, this, _1));

    rclcpp::QoS imu_qos = rclcpp::QoS(rclcpp::KeepLast(3000)).best_effort();
    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", imu_qos, std::bind(&MonoInertialNode::GrabImu, this, _1));
}

MonoInertialNode::~MonoInertialNode()
{
    if (syncThread_ && syncThread_->joinable()) {
        syncThread_->join();
        delete syncThread_;
    }
    SLAM_->SaveKeyFrameTrajectoryEuRoC("OrbSlam3EuRoC.txt");
    SLAM_->SaveKeyFrameTrajectoryTUM("OrbSlam3TUM.txt");
}

void MonoInertialNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    double cam_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // std::cout << "📷 Camera Timestamp (sec.nanosec): " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl;

    {
        std::lock_guard<std::mutex> lock(bufMutex_);

        if (!imuBuf_.empty()) {
            double closest_imu_time = 0;
            double min_diff = std::numeric_limits<double>::max();
            sensor_msgs::msg::Imu::SharedPtr best_imu_match = nullptr;

            // Copy queue into a temporary vector for iteration
            std::queue<sensor_msgs::msg::Imu::SharedPtr> temp_queue = imuBuf_;
            while (!temp_queue.empty()) {
                auto imu_msg = temp_queue.front();
                temp_queue.pop();

                double imu_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
                double diff = fabs(cam_time - imu_time);

                if (diff < min_diff) {
                    min_diff = diff;
                    closest_imu_time = imu_time;
                    best_imu_match = imu_msg;
                }
            }

            if (best_imu_match) {
                // std::cout << "📡 Closest IMU Timestamp: " << closest_imu_time
                        //   << " (Difference: " << min_diff << " sec)" << std::endl;

                if (min_diff > 0.05) {
                    // std::cerr << "⚠️  WARNING: IMU-Camera time difference is large: "
                    //           << min_diff << " sec" << std::endl;
                }
            }
        }

        imgBuf_.push(msg);

        // Prevent buffer overflow
        if (imgBuf_.size() > MAX_BUFFER_SIZE) {
            imgBuf_.pop();
        }
    }
}

void MonoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double imu_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // std::cout << "📡 IMU Timestamp (sec.nanosec): " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl;
    // Print IMU timestamps
    //std::cout << std::fixed << std::setprecision(9);
    //std::cout << "📡 IMU Timestamp: " << imu_time << std::endl;

    // Log to file
    std::ofstream logFile("/home/jeppe/imu_log.txt", std::ios::app);
    logFile << std::fixed << std::setprecision(9) << imu_time << std::endl;
    logFile.close();

    {
        std::lock_guard<std::mutex> lock(bufMutex_);
        imuBuf_.push(msg);

        // Debug: Print buffer size and first/last timestamps
        if (!imuBuf_.empty()) {
            double first_imu_time = imuBuf_.front()->header.stamp.sec + imuBuf_.front()->header.stamp.nanosec * 1e-9;
            double last_imu_time = imuBuf_.back()->header.stamp.sec + imuBuf_.back()->header.stamp.nanosec * 1e-9;
            // std::cout << "🛑 IMU Buffer: Size=" << imuBuf_.size()
            //           << " | First: " << first_imu_time
            //           << " | Last: " << last_imu_time
            //           << " | Current: " << imu_time << std::endl;
        }

        // Prevent buffer overflow
        if (imuBuf_.size() > MAX_BUFFER_SIZE) {
            imuBuf_.pop();
        }
    }
}


cv::Mat MonoInertialNode::GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image.clone();
}

void MonoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.1;  // Maximum time difference between images and IMU
    const double imuTimeWindow = 1.0; // Keep IMU data within ±1 second of the image
    const double imuMaxAge = 20.0;    // Maximum IMU message age before deletion
    const double maxImageAheadTime = 2.0; // Maximum Image age

    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 0;

        {
            std::lock_guard<std::mutex> lock(bufMutex_);

            // std::cout << "\n🔄 Checking buffers... "
            //           << " ImgBufSize: " << imgBuf_.size()
            //           << " | ImuBufSize: " << imuBuf_.size() << std::endl;

            if (!imgBuf_.empty() && !imuBuf_.empty())
            {
                tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);
                // std::cout << "📸 New Image Timestamp: " << tIm << std::endl;

                // Step 1: Remove IMU messages that are TOO OLD (20+ seconds before image)
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) < (tIm - imuMaxAge))
                {
                    //std::cout << "🧹 Removing very old IMU message: " 
                              //<< Utility::StampToSec(imuBuf_.front()->header.stamp)
                              //<< " < " << (tIm - imuMaxAge) << " (20+ sec old)" << std::endl;
                    imuBuf_.pop();
                }


                // Step 2: Ensure we still have valid IMU data
                if (imuBuf_.empty())
                {
                    //std::cout << "⚠️ IMU Buffer is empty after cleanup, skipping frame!" << std::endl;
                    continue;
                }

                // Step 3: Retrieve and remove the next image from the buffer
                im = GetImage(imgBuf_.front());
                imgBuf_.pop();
                //std::cout << "📤 Image processed and removed from buffer.\n" << std::endl;

                std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

                // Step 4: Process IMU data: Keep only messages within ±1 second of the image
                while (!imuBuf_.empty())
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);

                    // 🛑 If IMU messages are too far in the future, REMOVE them
                    if (t > (tIm + imuTimeWindow))
                    {
                        //std::cout << "🧹 Removing IMU message too far in future: " << t
                        //          << " > " << (tIm + imuTimeWindow) << std::endl;
                        imuBuf_.pop();  // ✅ Remove and check again
                        continue;       // 🔄 Keep looping to clean up
                    }

                    // ✅ Keep only IMU messages that are within the time window
                    cv::Point3f acc(
                        imuBuf_.front()->linear_acceleration.x,
                        imuBuf_.front()->linear_acceleration.y,
                        imuBuf_.front()->linear_acceleration.z);

                    cv::Point3f gyr(
                        imuBuf_.front()->angular_velocity.x,
                        imuBuf_.front()->angular_velocity.y,
                        imuBuf_.front()->angular_velocity.z);

                    // 🔍 Debugging: Check for NaN or Inf values
                    if (std::isnan(acc.x) || std::isnan(acc.y) || std::isnan(acc.z) ||
                        std::isnan(gyr.x) || std::isnan(gyr.y) || std::isnan(gyr.z))
                    {
                        std::cerr << "🚨 ERROR: NaN detected in IMU data! Timestamp: " << t << std::endl;
                    }
                    if (std::isinf(acc.x) || std::isinf(acc.y) || std::isinf(acc.z) ||
                        std::isinf(gyr.x) || std::isinf(gyr.y) || std::isinf(gyr.z))
                    {
                        std::cerr << "🚨 ERROR: Inf detected in IMU data! Timestamp: " << t << std::endl;
                    }

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop(); // ✅ Remove used IMU message from buffer
                }

                // Step 5: Print Debugging Info
                // << "📡 IMU Measurements for this frame: " << vImuMeas.size() << std::endl;

                if (vImuMeas.empty())
                {
                    //std::cout << "⚠️ No valid IMU measurements found for this frame!" << std::endl;
                }

                int trackingState = SLAM_->GetTrackingState();
                // std::cout << "🎯 ORB-SLAM Tracking State: " << trackingState << std::endl;

                auto start = std::chrono::high_resolution_clock::now();
                SLAM_->TrackMonocular(im, tIm, vImuMeas);
                auto end = std::chrono::high_resolution_clock::now();

                //std::cout << "✅ Tracking completed for this frame." << std::endl;

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
}
