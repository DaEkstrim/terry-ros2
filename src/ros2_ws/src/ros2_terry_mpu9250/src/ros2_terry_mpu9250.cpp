#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "TerryI2cCommunicator.h"
#include "mpu9250.h"

class TerryMpu9250Node : public rclcpp::Node
{
public:
    TerryMpu9250Node() : Node("terry_mpu9250")
    {
        std::unique_ptr<I2cCommunicator> i2cBus = std::make_unique<TerryI2cCommunicator>();
        mpu9250_ = std::make_unique<MPU9250>(std::move(i2cBus));

        int8_t status = mpu9250_->begin();
        if (status < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "IMU initialization unsuccessful. Status code: %d", status);
        }

        // setting the accelerometer full scale range to +/-2G
        mpu9250_->setAccelRange(MPU9250::ACCEL_RANGE_2G);
        // setting the gyroscope full scale range to +/-250 deg/s
        mpu9250_->setGyroRange(MPU9250::GYRO_RANGE_250DPS);
        // setting DLPF bandwidth to 20 Hz
        mpu9250_->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        mpu9250_->setSrd(19);
       
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        magPublisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
        
        std::chrono::duration<int64_t, std::milli> frequency = std::chrono::milliseconds(10);
        timer_ = this->create_wall_timer(frequency, std::bind(&TerryMpu9250Node::timerTick, this));

        RCLCPP_INFO(this->get_logger(), "Terry Mpu9250 started");
    }

private:
    std::unique_ptr<MPU9250> mpu9250_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int8_t readSensorResult_;
    void timerTick()
    {

        readSensorResult_ = mpu9250_->readSensor();

        if (readSensorResult_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read sensor data. Result: %d", readSensorResult_);
        }

        // RCLCPP_INFO(this->get_logger(), "Acceleration X: %f, Y: %f, Z: %f",
        //       mpu9250_->getAccelX_mss(), mpu9250_->getAccelY_mss(), mpu9250_->getAccelZ_mss());

        // RCLCPP_INFO(this->get_logger(), "Gyroscope X: %f, Y: %f, Z: %f",
        //       mpu9250_->getGyroX_rads(), mpu9250_->getGyroY_rads(), mpu9250_->getGyroZ_rads());
        
        // RCLCPP_INFO(this->get_logger(), "Magnetometer X: %f, Y: %f, Z: %f",
        //       mpu9250_->getMagX_uT(), mpu9250_->getMagY_uT(), mpu9250_->getMagZ_uT());
        
        auto message = sensor_msgs::msg::Imu();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "base_link";
        // Direct measurements
        message.linear_acceleration_covariance[0] = -1.0;
        message.linear_acceleration.x = mpu9250_->getAccelX_mss();
        message.linear_acceleration.y = mpu9250_->getAccelY_mss();
        message.linear_acceleration.z = mpu9250_->getAccelZ_mss();
        message.angular_velocity_covariance[0] = -1.0;
        message.angular_velocity.x = mpu9250_->getGyroX_rads();
        message.angular_velocity.y = mpu9250_->getGyroY_rads();
        message.angular_velocity.z = mpu9250_->getGyroZ_rads();
        
        auto magMessage = sensor_msgs::msg::MagneticField();
        magMessage.header.stamp = message.header.stamp;
        magMessage.header.frame_id = message.header.frame_id;
        magMessage.magnetic_field_covariance[0] = -1.0;
        magMessage.magnetic_field.x = mpu9250_->getMagX_uT();
        magMessage.magnetic_field.y = mpu9250_->getMagY_uT();
        magMessage.magnetic_field.z = mpu9250_->getMagZ_uT();

        publisher_->publish(message);
        magPublisher_->publish(magMessage);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TerryMpu9250Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}