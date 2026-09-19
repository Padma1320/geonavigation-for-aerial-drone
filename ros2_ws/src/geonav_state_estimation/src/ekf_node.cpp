#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class GeoNavEKF : public rclcpp::Node
{
public:
    GeoNavEKF()
    : Node("geonav_ekf"),
      initialized_(false),
      have_heading_(false),
      have_reference_(false),
      last_imu_timestamp_(0),
      reference_north_(0.0),
      reference_east_(0.0)
    {
        // ============================================================
        // PARAMETERS
        // ============================================================

        motion_speed_threshold_ =
            this->declare_parameter<double>(
                "motion_speed_threshold",
                0.30
            );

        motion_hold_sec_ =
            this->declare_parameter<double>(
                "motion_hold_sec",
                2.0
            );

        dropout_duration_sec_ =
            this->declare_parameter<double>(
                "dropout_duration_sec",
                5.0
            );

        log_csv_ =
            this->declare_parameter<bool>(
                "log_csv",
                true
            );

        csv_path_ =
            this->declare_parameter<std::string>(
                "csv_path",
                "/tmp/geonav_ekf_motion_flight.csv"
            );

        // ============================================================
        // EKF STATE
        //
        // x =
        // [ North
        //   East
        //   Vnorth
        //   Veast ]
        // ============================================================

        x_.setZero();

        P_.setIdentity();
        P_ *= 1.0;

        // Process noise
        Q_.setZero();

        Q_(0, 0) = 0.05;
        Q_(1, 1) = 0.05;
        Q_(2, 2) = 0.50;
        Q_(3, 3) = 0.50;

        // Measurement noise
        R_.setZero();

        R_(0, 0) = 0.50;
        R_(1, 1) = 0.50;
        R_(2, 2) = 0.20;
        R_(3, 3) = 0.20;

        // ============================================================
        // QoS
        // ============================================================

        auto px4_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(10)
            )
            .best_effort()
            .transient_local();

        auto sensor_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(10)
            )
            .best_effort()
            .durability_volatile();

        // ============================================================
        // SUBSCRIPTIONS
        // ============================================================

        imu_sub_ =
            this->create_subscription<
                px4_msgs::msg::SensorCombined
            >(
                "/fmu/out/sensor_combined",
                sensor_qos,
                std::bind(
                    &GeoNavEKF::imuCallback,
                    this,
                    _1
                )
            );

        local_position_sub_ =
            this->create_subscription<
                px4_msgs::msg::VehicleLocalPosition
            >(
                "/fmu/out/vehicle_local_position_v1",
                px4_qos,
                std::bind(
                    &GeoNavEKF::localPositionCallback,
                    this,
                    _1
                )
            );

        reference_odom_sub_ =
            this->create_subscription<
                px4_msgs::msg::VehicleOdometry
            >(
                "/fmu/out/vehicle_odometry",
                px4_qos,
                std::bind(
                    &GeoNavEKF::referenceOdomCallback,
                    this,
                    _1
                )
            );

        // ============================================================
        // OUTPUT
        // ============================================================

        estimate_pub_ =
            this->create_publisher<
                nav_msgs::msg::Odometry
            >(
                "/geonav/state_estimate",
                10
            );

        // ============================================================
        // TIME INITIALIZATION
        // ============================================================

        start_time_ =
            this->get_clock()->now();

        motion_start_time_ =
            this->get_clock()->now();

        dropout_end_time_ =
            this->get_clock()->now();

        // ============================================================
        // CSV LOGGING
        // ============================================================

        if (log_csv_)
        {
            csv_file_.open(csv_path_);

            if (csv_file_.is_open())
            {
                csv_file_
                    << "time_sec,"
                    << "ekf_north,"
                    << "ekf_east,"
                    << "ekf_vnorth,"
                    << "ekf_veast,"
                    << "ref_north,"
                    << "ref_east,"
                    << "position_error_m,"
                    << "dropout_active,"
                    << "horizontal_speed_mps\n";

                RCLCPP_INFO(
                    this->get_logger(),
                    "CSV logging enabled: %s",
                    csv_path_.c_str()
                );
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Could not open CSV file: %s",
                    csv_path_.c_str()
                );
            }
        }

        // ============================================================
        // STARTUP INFORMATION
        // ============================================================

        RCLCPP_INFO(
            this->get_logger(),
            "=============================================="
        );

        RCLCPP_INFO(
            this->get_logger(),
            "GeoNav EKF started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "State: [North, East, Vnorth, Veast]"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Prediction: PX4 SensorCombined IMU"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Correction: PX4 VehicleLocalPosition"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Reference only: PX4 VehicleOdometry"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Output: /geonav/state_estimate"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "AUTOMATIC MOTION-TRIGGERED DROPOUT ENABLED"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Motion speed threshold: %.2f m/s",
            motion_speed_threshold_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Required continuous motion: %.1f s",
            motion_hold_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Dropout duration: %.1f s",
            dropout_duration_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for UAV motion..."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "=============================================="
        );
    }

    ~GeoNavEKF()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }

private:

    // ================================================================
    // TIME
    // ================================================================

    double elapsedSeconds()
    {
        return
            (
                this->get_clock()->now()
                -
                start_time_
            ).seconds();
    }

    // ================================================================
    // DROPOUT STATUS
    // ================================================================

    bool isDropoutActive()
    {
        if (!dropout_triggered_)
        {
            return false;
        }

        if (dropout_finished_)
        {
            return false;
        }

        return
            this->get_clock()->now()
            <
            dropout_end_time_;
    }

    // ================================================================
    // AUTOMATIC MOTION DETECTION
    // ================================================================

    void updateMotionTrigger(
        double vn,
        double ve
    )
    {
        // Only one automatic dropout per node run.
        if (dropout_triggered_)
        {
            return;
        }

        const double horizontal_speed =
            std::hypot(
                vn,
                ve
            );

        latest_horizontal_speed_ =
            horizontal_speed;

        // ------------------------------------------------------------
        // NOT MOVING FAST ENOUGH
        // ------------------------------------------------------------

        if (
            horizontal_speed
            <
            motion_speed_threshold_
        )
        {
            if (motion_candidate_active_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Motion candidate reset | speed=%.2f m/s",
                    horizontal_speed
                );
            }

            motion_candidate_active_ =
                false;

            return;
        }

        // ------------------------------------------------------------
        // FIRST DETECTION OF MOTION
        // ------------------------------------------------------------

        if (!motion_candidate_active_)
        {
            motion_candidate_active_ =
                true;

            motion_start_time_ =
                this->get_clock()->now();

            RCLCPP_INFO(
                this->get_logger(),
                "MOTION DETECTED | speed=%.2f m/s",
                horizontal_speed
            );

            RCLCPP_INFO(
                this->get_logger(),
                "Waiting %.1f s continuous motion before dropout...",
                motion_hold_sec_
            );

            return;
        }

        // ------------------------------------------------------------
        // CHECK HOW LONG UAV HAS CONTINUOUSLY MOVED
        // ------------------------------------------------------------

        const double moving_time =
            (
                this->get_clock()->now()
                -
                motion_start_time_
            ).seconds();

        if (
            moving_time
            <
            motion_hold_sec_
        )
        {
            return;
        }

        // ------------------------------------------------------------
        // AUTOMATICALLY TRIGGER DROPOUT
        // ------------------------------------------------------------

        dropout_triggered_ =
            true;

        dropout_finished_ =
            false;

        dropout_end_time_ =
            this->get_clock()->now()
            +
            rclcpp::Duration::from_seconds(
                dropout_duration_sec_
            );

        RCLCPP_WARN(
            this->get_logger(),
            "=============================================="
        );

        RCLCPP_WARN(
            this->get_logger(),
            "AUTOMATIC MEASUREMENT DROPOUT STARTED"
        );

        RCLCPP_WARN(
            this->get_logger(),
            "UAV confirmed moving for %.1f s",
            moving_time
        );

        RCLCPP_WARN(
            this->get_logger(),
            "Current horizontal speed: %.2f m/s",
            horizontal_speed
        );

        RCLCPP_WARN(
            this->get_logger(),
            "VehicleLocalPosition correction disabled"
        );

        RCLCPP_WARN(
            this->get_logger(),
            "EKF NOW RUNNING ON IMU PREDICTION ONLY"
        );

        RCLCPP_WARN(
            this->get_logger(),
            "Dropout duration: %.1f s",
            dropout_duration_sec_
        );

        RCLCPP_WARN(
            this->get_logger(),
            "=============================================="
        );
    }

    // ================================================================
    // CHECK DROPOUT RECOVERY
    // ================================================================

    void checkDropoutRecovery()
    {
        if (!dropout_triggered_)
        {
            return;
        }

        if (dropout_finished_)
        {
            return;
        }

        if (
            this->get_clock()->now()
            <
            dropout_end_time_
        )
        {
            return;
        }

        dropout_finished_ =
            true;

        RCLCPP_WARN(
            this->get_logger(),
            "=============================================="
        );

        RCLCPP_WARN(
            this->get_logger(),
            "MEASUREMENT DROPOUT ENDED"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "VehicleLocalPosition correction restored"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "EKF correcting accumulated inertial drift"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Automatic dropout experiment completed"
        );

        RCLCPP_WARN(
            this->get_logger(),
            "=============================================="
        );
    }

    // ================================================================
    // IMU PREDICTION
    // ================================================================

    void imuCallback(
        const px4_msgs::msg::SensorCombined::SharedPtr msg
    )
    {
        if (!initialized_)
        {
            return;
        }

        if (!have_heading_)
        {
            return;
        }

        const uint64_t timestamp =
            msg->timestamp;

        if (last_imu_timestamp_ == 0)
        {
            last_imu_timestamp_ =
                timestamp;

            return;
        }

        const double dt =
            static_cast<double>(
                timestamp
                -
                last_imu_timestamp_
            )
            *
            1e-6;

        last_imu_timestamp_ =
            timestamp;

        if (
            dt <= 0.0
            ||
            dt > 0.1
        )
        {
            return;
        }

        // ------------------------------------------------------------
        // IMU BODY-FRAME ACCELERATION
        //
        // PX4 FRD:
        // X = forward
        // Y = right
        // Z = down
        // ------------------------------------------------------------

        const double a_forward =
            static_cast<double>(
                msg->accelerometer_m_s2[0]
            );

        const double a_right =
            static_cast<double>(
                msg->accelerometer_m_s2[1]
            );

        const double cos_yaw =
            std::cos(
                heading_
            );

        const double sin_yaw =
            std::sin(
                heading_
            );

        // ------------------------------------------------------------
        // BODY -> NED HORIZONTAL ROTATION
        // ------------------------------------------------------------

        const double a_north =
            cos_yaw * a_forward
            -
            sin_yaw * a_right;

        const double a_east =
            sin_yaw * a_forward
            +
            cos_yaw * a_right;

        // ------------------------------------------------------------
        // STATE TRANSITION
        // ------------------------------------------------------------

        Eigen::Matrix4d F =
            Eigen::Matrix4d::Identity();

        F(0, 2) =
            dt;

        F(1, 3) =
            dt;

        // ------------------------------------------------------------
        // CONTROL MATRIX
        // ------------------------------------------------------------

        Eigen::Matrix<double, 4, 2> B;

        B <<
            0.5 * dt * dt, 0.0,
            0.0, 0.5 * dt * dt,
            dt, 0.0,
            0.0, dt;

        Eigen::Vector2d u;

        u <<
            a_north,
            a_east;

        // ------------------------------------------------------------
        // EKF PREDICTION
        // ------------------------------------------------------------

        x_ =
            F
            *
            x_
            +
            B
            *
            u;

        P_ =
            F
            *
            P_
            *
            F.transpose()
            +
            Q_
            *
            dt;

        publishEstimate();
    }

    // ================================================================
    // LOCAL POSITION CALLBACK
    // ================================================================

    void localPositionCallback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg
    )
    {
        // ------------------------------------------------------------
        // HEADING
        // ------------------------------------------------------------

        if (
            std::isfinite(
                msg->heading
            )
        )
        {
            heading_ =
                static_cast<double>(
                    msg->heading
                );

            have_heading_ =
                true;
        }

        // ------------------------------------------------------------
        // VALIDITY
        // ------------------------------------------------------------

        if (!msg->xy_valid)
        {
            return;
        }

        if (!msg->v_xy_valid)
        {
            return;
        }

        const double north =
            static_cast<double>(
                msg->x
            );

        const double east =
            static_cast<double>(
                msg->y
            );

        const double vn =
            static_cast<double>(
                msg->vx
            );

        const double ve =
            static_cast<double>(
                msg->vy
            );

        if (
            !std::isfinite(north)
            ||
            !std::isfinite(east)
            ||
            !std::isfinite(vn)
            ||
            !std::isfinite(ve)
        )
        {
            return;
        }

        latest_horizontal_speed_ =
            std::hypot(
                vn,
                ve
            );

        // ------------------------------------------------------------
        // INITIALIZATION
        // ------------------------------------------------------------

        if (!initialized_)
        {
            x_ <<
                north,
                east,
                vn,
                ve;

            P_.setIdentity();

            P_ *=
                0.5;

            initialized_ =
                true;

            RCLCPP_INFO(
                this->get_logger(),
                "EKF INITIALIZED | "
                "N=%.2f E=%.2f "
                "VN=%.2f VE=%.2f",
                north,
                east,
                vn,
                ve
            );

            publishEstimate();

            return;
        }

        // ------------------------------------------------------------
        // AUTOMATIC MOTION DETECTOR
        // ------------------------------------------------------------

        updateMotionTrigger(
            vn,
            ve
        );

        // ------------------------------------------------------------
        // ACTIVE DROPOUT
        // ------------------------------------------------------------

        if (isDropoutActive())
        {
            // Deliberately ignore position and velocity measurement.
            //
            // IMU prediction continues in imuCallback().
            return;
        }

        // ------------------------------------------------------------
        // AUTOMATIC RECOVERY
        // ------------------------------------------------------------

        checkDropoutRecovery();

        // ============================================================
        // EKF MEASUREMENT CORRECTION
        // ============================================================

        Eigen::Vector4d z;

        z <<
            north,
            east,
            vn,
            ve;

        Eigen::Matrix4d H =
            Eigen::Matrix4d::Identity();

        // Innovation
        Eigen::Vector4d innovation =
            z
            -
            H
            *
            x_;

        // Innovation covariance
        Eigen::Matrix4d S =
            H
            *
            P_
            *
            H.transpose()
            +
            R_;

        // Kalman gain
        Eigen::Matrix4d K =
            P_
            *
            H.transpose()
            *
            S.inverse();

        // Correct state
        x_ =
            x_
            +
            K
            *
            innovation;

        // ------------------------------------------------------------
        // JOSEPH-FORM COVARIANCE UPDATE
        // ------------------------------------------------------------

        Eigen::Matrix4d I =
            Eigen::Matrix4d::Identity();

        P_ =
            (
                I
                -
                K
                *
                H
            )
            *
            P_
            *
            (
                I
                -
                K
                *
                H
            ).transpose()
            +
            K
            *
            R_
            *
            K.transpose();

        publishEstimate();
    }

    // ================================================================
    // REFERENCE ODOMETRY CALLBACK
    // ================================================================

    void referenceOdomCallback(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg
    )
    {
        // VehicleOdometry is evaluation/reference only.
        //
        // It is NOT used by the Kalman correction.

        if (
            !std::isfinite(
                msg->position[0]
            )
            ||
            !std::isfinite(
                msg->position[1]
            )
        )
        {
            return;
        }

        reference_north_ =
            static_cast<double>(
                msg->position[0]
            );

        reference_east_ =
            static_cast<double>(
                msg->position[1]
            );

        have_reference_ =
            true;
    }

    // ================================================================
    // PUBLISH ESTIMATE
    // ================================================================

    void publishEstimate()
    {
        if (!initialized_)
        {
            return;
        }

        nav_msgs::msg::Odometry output;

        output.header.stamp =
            this->get_clock()->now();

        output.header.frame_id =
            "map";

        output.child_frame_id =
            "base_link";

        // ------------------------------------------------------------
        // POSITION
        // ------------------------------------------------------------

        output.pose.pose.position.x =
            x_(0);

        output.pose.pose.position.y =
            x_(1);

        output.pose.pose.position.z =
            0.0;

        // ------------------------------------------------------------
        // ORIENTATION
        // ------------------------------------------------------------

        if (have_heading_)
        {
            const double half_yaw =
                heading_
                *
                0.5;

            output.pose.pose.orientation.w =
                std::cos(
                    half_yaw
                );

            output.pose.pose.orientation.x =
                0.0;

            output.pose.pose.orientation.y =
                0.0;

            output.pose.pose.orientation.z =
                std::sin(
                    half_yaw
                );
        }
        else
        {
            output.pose.pose.orientation.w =
                1.0;
        }

        // ------------------------------------------------------------
        // VELOCITY
        // ------------------------------------------------------------

        output.twist.twist.linear.x =
            x_(2);

        output.twist.twist.linear.y =
            x_(3);

        output.twist.twist.linear.z =
            0.0;

        // ------------------------------------------------------------
        // COVARIANCE
        // ------------------------------------------------------------

        output.pose.covariance[0] =
            P_(0, 0);

        output.pose.covariance[7] =
            P_(1, 1);

        output.twist.covariance[0] =
            P_(2, 2);

        output.twist.covariance[7] =
            P_(3, 3);

        estimate_pub_->publish(
            output
        );

        // ------------------------------------------------------------
        // POSITION ERROR
        // ------------------------------------------------------------

        const double t =
            elapsedSeconds();

        const bool dropout =
            isDropoutActive();

        double error =
            NAN;

        if (have_reference_)
        {
            error =
                std::hypot(
                    x_(0)
                    -
                    reference_north_,
                    x_(1)
                    -
                    reference_east_
                );
        }

        // ------------------------------------------------------------
        // CSV LOGGING
        // ------------------------------------------------------------

        if (
            log_csv_
            &&
            csv_file_.is_open()
        )
        {
            csv_file_
                << std::fixed
                << std::setprecision(6)
                << t
                << ","
                << x_(0)
                << ","
                << x_(1)
                << ","
                << x_(2)
                << ","
                << x_(3)
                << ",";

            if (have_reference_)
            {
                csv_file_
                    << reference_north_
                    << ","
                    << reference_east_
                    << ","
                    << error
                    << ",";
            }
            else
            {
                csv_file_
                    << "nan,nan,nan,";
            }

            csv_file_
                << (
                    dropout
                    ?
                    1
                    :
                    0
                )
                << ","
                << latest_horizontal_speed_
                << "\n";
        }

        // ------------------------------------------------------------
        // PERIODIC TERMINAL STATUS
        // ------------------------------------------------------------

        static int counter =
            0;

        counter++;

        if (
            counter
            >=
            100
        )
        {
            counter =
                0;

            if (have_reference_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "EKF | "
                    "N=%.2f E=%.2f "
                    "VN=%.2f VE=%.2f | "
                    "speed=%.2f m/s | "
                    "error=%.3f m | "
                    "dropout=%s",
                    x_(0),
                    x_(1),
                    x_(2),
                    x_(3),
                    latest_horizontal_speed_,
                    error,
                    dropout
                        ?
                        "YES"
                        :
                        "NO"
                );
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "EKF | "
                    "N=%.2f E=%.2f "
                    "VN=%.2f VE=%.2f | "
                    "speed=%.2f m/s | "
                    "dropout=%s",
                    x_(0),
                    x_(1),
                    x_(2),
                    x_(3),
                    latest_horizontal_speed_,
                    dropout
                        ?
                        "YES"
                        :
                        "NO"
                );
            }
        }
    }

    // ================================================================
    // ROS OBJECTS
    // ================================================================

    rclcpp::Subscription<
        px4_msgs::msg::SensorCombined
    >::SharedPtr imu_sub_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleLocalPosition
    >::SharedPtr local_position_sub_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry
    >::SharedPtr reference_odom_sub_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry
    >::SharedPtr estimate_pub_;

    // ================================================================
    // EKF
    // ================================================================

    Eigen::Vector4d x_;

    Eigen::Matrix4d P_;

    Eigen::Matrix4d Q_;

    Eigen::Matrix4d R_;

    bool initialized_;

    bool have_heading_;

    bool have_reference_;

    double heading_{
        0.0
    };

    uint64_t last_imu_timestamp_;

    // ================================================================
    // REFERENCE
    // ================================================================

    double reference_north_;

    double reference_east_;

    // ================================================================
    // AUTOMATIC MOTION TRIGGER
    // ================================================================

    double motion_speed_threshold_;

    double motion_hold_sec_;

    bool motion_candidate_active_{
        false
    };

    rclcpp::Time motion_start_time_;

    double latest_horizontal_speed_{
        0.0
    };

    // ================================================================
    // AUTOMATIC DROPOUT
    // ================================================================

    double dropout_duration_sec_;

    bool dropout_triggered_{
        false
    };

    bool dropout_finished_{
        false
    };

    rclcpp::Time dropout_end_time_;

    // ================================================================
    // LOGGING
    // ================================================================

    bool log_csv_;

    std::string csv_path_;

    std::ofstream csv_file_;

    // ================================================================
    // TIME
    // ================================================================

    rclcpp::Time start_time_;
};

int main(
    int argc,
    char * argv[]
)
{
    rclcpp::init(
        argc,
        argv
    );

    auto node =
        std::make_shared<
            GeoNavEKF
        >();

    rclcpp::spin(
        node
    );

    rclcpp::shutdown();

    return 0;
}
