#include "mono-inertial-node2.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera", 10, std::bind(&MonocularInertialNode::GrabImage, this, _1));
    subImu_ = this->create_subscription<ImuMsg>(
        "imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);

    std::cout << "System Initialization Complete" << std::endl;
}

MonocularInertialNode::~MonocularInertialNode()
{
//    if (syncThread_->joinable()) {
    syncThread_->join();
//    }
    delete syncThread_;

    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("OrbSlam3TUM.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    if (!std::isnan(msg->linear_acceleration.x) && !std::isnan(msg->linear_acceleration.y) &&
        !std::isnan(msg->linear_acceleration.z) && !std::isnan(msg->angular_velocity.x) &&
        !std::isnan(msg->angular_velocity.y) && !std::isnan(msg->angular_velocity.z))
    {
        bufMutex_.lock();
        imuBuf_.push(msg);
        bufMutex_.unlock();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid IMU data - Rxd NaN");
    }
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialNode::SyncWithImu()
{
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> img_lock(bufMutexImg_, std::defer_lock);
        std::unique_lock<std::mutex> imu_lock(bufMutex_, std::defer_lock);

        std::lock(img_lock, imu_lock);

        if (!imgBuf_.empty() && !imuBuf_.empty()) {
            auto imgPtr = imgBuf_.front();
            double tImage = Utility::StampToSec(imgPtr->header.stamp);
            // double tImageshort = fmod(tImage, 100);

            cv::Mat imageFrame = GetImage(imgPtr); // Process image before popping
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            std::stringstream imu_data_stream;

            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImage) {
                auto imuPtr = imuBuf_.front();
                double tIMU = Utility::StampToSec(imuPtr->header.stamp);
                // double tIMUshort = fmod(tIMU, 100);

                imuBuf_.pop();
                cv::Point3f acc(imuPtr->linear_acceleration.x, imuPtr->linear_acceleration.y, imuPtr->linear_acceleration.z);
                cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y, imuPtr->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));

                // Debug info
                // imu_data_stream << "IMU at " << std::fixed << std::setprecision(6) << tIMUshort << " - Acc: [" << acc << "], Gyr: [" << gyr << "]\n";
            }

            imgBuf_.pop(); // Safely pop the image from the buffer here

            if (vImuMeas.empty()) {
                RCLCPP_WARN(this->get_logger(), "No valid IMU data available for the current frame at time %.6f.", tImage);
                continue; // Skip processing this frame
            }

            try {
                m_SLAM->TrackMonocular(imageFrame, tImage, vImuMeas);
                // RCLCPP_INFO(this->get_logger(), "Image at %.6f processed with IMU data: \n%s", tImageshort, imu_data_stream.str().c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "SLAM processing exception: %s", e.what());
            }
        }

        img_lock.unlock();
        imu_lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

