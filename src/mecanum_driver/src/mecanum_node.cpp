#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <serial/serial.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

class MecanumRobot
{
public:
    static constexpr int NMOTORS = 4;

    enum Motor
    {
        LEFT_UP = 0,
        LEFT_DOWN = 1,
        RIGHT_UP = 2,
        RIGHT_DOWN = 3
    };

    MecanumRobot()
        : nh_(), pnh_("~")
    {
        ROS_INFO("Initializing Mecanum Robot...");

        // Parameters
        pnh_.param("wheel_diameter", wheel_diameter_, 9.5);
        pnh_.param("encoder_total", encoder_total_count_, 1798);
        pnh_.param("total_length", total_length_, 41.04);

        pnh_.param("port", port_, std::string("/dev/esp32"));
        pnh_.param("baud", baud_, 57600);

        pnh_.param("use_imu", use_imu_, false);
        pnh_.param("test_mode", test_mode_, 0);

        pnh_.param("start_run", start_velocity_test_, 1.0);

        pnh_.param("x_offset", x_offset_, 1.0);
        pnh_.param("y_offset", y_offset_, 1.0);
        pnh_.param("theta_offset", theta_offset_, 1.0);

        pnh_.param("distance_move", distance_move_, 1.0);
        pnh_.param("axis_move", axis_move_, 1.0);
        pnh_.param("velocity_move", velocity_move_, 1.0);

        pnh_.param("coeff_speed", coeff_speed_, 1.0);

        pnh_.param("max_speed", max_speed_, 30.0);
        pnh_.param("loop_rate", loop_rate_, 40.0);

        pnh_.param("odom_frame", odom_frame_, std::string("odom"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));

        pnh_.param("publish_tf", publish_tf_, false);

        // Robot parameters
        a_coeff_ = 0.52188555;
        b_coeff_ = 0.23905722;

        cm_per_count_ =
            (M_PI * wheel_diameter_) / encoder_total_count_;

        delta_cvt_ = 60.0 / (M_PI * wheel_diameter_);

        // Initialize variables
        robot_pose_.fill(0.0);
        encoder_total_.fill(0.0);
        last_encoder_.fill(0.0);
        speed_desired_.fill(0.0);

        speed_filter_x_ = 0.0;
        speed_filter_y_ = 0.0;
        speed_filter_z_ = 0.0;

        last_speed_x_ = 0.0;
        last_speed_y_ = 0.0;
        last_speed_z_ = 0.0;

        vx_now_ = 0.0;
        vy_now_ = 0.0;
        theta_now_ = 0.0;

        vx_run_ = 0.0;
        vy_run_ = 0.0;
        w_run_ = 0.0;

        first_encoder_received_ = false;
        motor_error_ = false;

        // ROS Publisher
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(
            "/odom", 5);

        motor_error_pub_ = nh_.advertise<std_msgs::Bool>(
            "/MOTOR_ERROR", 1);

        desired_pub_ = nh_.advertise<std_msgs::Float32>(
            "/speed_desired", 1);

        feedback_pub_ = nh_.advertise<std_msgs::Float32>(
            "/speed_feedback", 1);

        // ROS Subscriber
        cmd_vel_sub_ = nh_.subscribe(
            "/cmd_vel_timeout",
            10,
            &MecanumRobot::cmdVelCallback,
            this);

        // Serial initialization
        try
        {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial_.setTimeout(
                serial::Timeout::simpleTimeout(10));

            serial_.open();

            if (serial_.isOpen())
            {
                ROS_INFO("Serial connected: %s @ %d",
                         port_.c_str(), baud_);
            }
        }
        catch (const serial::IOException& e)
        {
            ROS_FATAL("Cannot open serial port: %s", e.what());
            throw;
        }

        last_time_encod_ = ros::Time::now();

        ROS_INFO("Mecanum Robot initialized");
        ROS_INFO("Wheel diameter: %.2f cm", wheel_diameter_);
        ROS_INFO("Encoder total: %d", encoder_total_count_);
        ROS_INFO("CM per count: %.6f", cm_per_count_);
        ROS_INFO("Total length: %.2f cm", total_length_);
    }

    ~MecanumRobot()
    {
        stopRobot();

        if (serial_.isOpen())
        {
            serial_.close();
        }

        ROS_INFO("Mecanum Robot stopped");
    }

    // =====================================================
    // CALLBACK CMD_VEL
    // =====================================================

    void cmdVelCallback(
        const geometry_msgs::Twist::ConstPtr& msg)
    {
        vx_run_ = msg->linear.x * 100.0;
        vy_run_ = msg->linear.y * 100.0;
        w_run_ = msg->angular.z;

        calculateSpeed(vx_run_, vy_run_, w_run_);

        runRobot();
    }

    // =====================================================
    // ENCODER OVERFLOW
    // =====================================================

    double normalizeOverflow(
        double encoder_now,
        double last_encoder)
    {
        double delta = encoder_now - last_encoder;

        if (delta > 10000)
        {
            delta = (encoder_now - 32768.0)
                    - (32768.0 + last_encoder);
        }
        else if (delta < -10000)
        {
            delta = (32768.0 + encoder_now)
                    + (32768.0 - last_encoder);
        }

        return delta;
    }

    double calculateEncoderDelta(int motor)
    {
        double delta = normalizeOverflow(
            encoder_total_[motor],
            last_encoder_[motor]);

        last_encoder_[motor] = encoder_total_[motor];

        return delta;
    }

    // =====================================================
    // MECANUM ODOMETRY
    // =====================================================

    void updatePosition()
    {
        std::array<double, 4> delta;

        delta[LEFT_UP] =
            calculateEncoderDelta(LEFT_UP) * cm_per_count_;

        delta[LEFT_DOWN] =
            calculateEncoderDelta(LEFT_DOWN) * cm_per_count_;

        delta[RIGHT_UP] =
            calculateEncoderDelta(RIGHT_UP) * cm_per_count_;

        delta[RIGHT_DOWN] =
            calculateEncoderDelta(RIGHT_DOWN) * cm_per_count_;

        // Forward kinematics (cm)
        double dx =
            (delta[LEFT_UP] +
             delta[RIGHT_UP] +
             delta[LEFT_DOWN] +
             delta[RIGHT_DOWN]) / 4.0;

        double dy =
            (-delta[RIGHT_UP] -
             delta[LEFT_DOWN] +
             delta[LEFT_UP] +
             delta[RIGHT_DOWN]) / 4.0;

        double dtheta =
            (delta[RIGHT_UP] -
             delta[LEFT_DOWN] -
             delta[LEFT_UP] +
             delta[RIGHT_DOWN]) /
            (4.0 * total_length_);

        // Convert cm to meter
        dx /= 100.0;
        dy /= 100.0;

        ros::Time current_time = ros::Time::now();

        double dt = (current_time - last_time_encod_).toSec();

        if (dt <= 0.0)
        {
            last_time_encod_ = current_time;
            return;
        }

        // Velocity
        vx_now_ = dx / dt;
        vy_now_ = dy / dt;
        theta_now_ = dtheta / dt;

        // Update robot pose
        double theta = robot_pose_[2];

        robot_pose_[0] +=
            dx * cos(theta) - dy * sin(theta);

        robot_pose_[1] +=
            dy * cos(theta) + dx * sin(theta);

        robot_pose_[2] += dtheta;

        // Normalize angle
        while (robot_pose_[2] >= M_PI)
            robot_pose_[2] -= 2.0 * M_PI;

        while (robot_pose_[2] <= -M_PI)
            robot_pose_[2] += 2.0 * M_PI;

        // Low pass filter
        speed_filter_x_ =
            a_coeff_ * speed_filter_x_
            + b_coeff_ * vx_now_
            + b_coeff_ * last_speed_x_;

        speed_filter_y_ =
            a_coeff_ * speed_filter_y_
            + b_coeff_ * vy_now_
            + b_coeff_ * last_speed_y_;

        speed_filter_z_ =
            a_coeff_ * speed_filter_z_
            + b_coeff_ * theta_now_
            + b_coeff_ * last_speed_z_;

        last_speed_x_ = vx_now_;
        last_speed_y_ = vy_now_;
        last_speed_z_ = theta_now_;

        publishOdometry(current_time, dtheta);

        last_time_encod_ = current_time;
    }

    // =====================================================
    // PUBLISH ODOMETRY
    // =====================================================

    void publishOdometry(
        const ros::Time& stamp,
        double dtheta)
    {
        nav_msgs::Odometry odom;

        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = robot_pose_[0];
        odom.pose.pose.position.y = robot_pose_[1];
        odom.pose.pose.position.z = 0.0;

        geometry_msgs::Quaternion q =
            tf::createQuaternionMsgFromYaw(robot_pose_[2]);

        odom.pose.pose.orientation = q;

        double yaw_covariance = (dtheta == 0.0) ? 1e-4 : 0.35;

        // Pose covariance
        odom.pose.covariance.fill(0.0);

        odom.pose.covariance[0] = 0.1;
        odom.pose.covariance[7] = 0.1;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = yaw_covariance;

        // Twist covariance
        odom.twist.covariance.fill(0.0);

        odom.twist.covariance[0] = 0.05;
        odom.twist.covariance[7] = 0.05;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = yaw_covariance;

        odom.twist.twist.linear.x = speed_filter_x_;
        odom.twist.twist.linear.y = speed_filter_y_;
        odom.twist.twist.angular.z = -speed_filter_z_;

        odom_pub_.publish(odom);

        // Optional TF
        if (publish_tf_)
        {
            tf::Transform transform;

            transform.setOrigin(
                tf::Vector3(
                    robot_pose_[0],
                    robot_pose_[1],
                    0.0));

            transform.setRotation(
                tf::createQuaternionFromYaw(robot_pose_[2]));

            tf_broadcaster_.sendTransform(
                tf::StampedTransform(
                    transform,
                    stamp,
                    odom_frame_,
                    base_frame_));
        }
    }

    // =====================================================
    // INVERSE KINEMATICS
    // =====================================================

    void calculateSpeed(
        double vx,
        double vy,
        double dtheta)
    {
        std::array<double, 4> speed_cm_s;

        speed_cm_s[LEFT_UP] =
            vx + vy - dtheta * total_length_;

        speed_cm_s[RIGHT_UP] =
            vx - vy + dtheta * total_length_;

        speed_cm_s[LEFT_DOWN] =
            vx - vy - dtheta * total_length_;

        speed_cm_s[RIGHT_DOWN] =
            vx + vy + dtheta * total_length_;

        for (int i = 0; i < NMOTORS; i++)
        {
            speed_desired_[i] =
                static_cast<int>(speed_cm_s[i]);
        }
    }

    // =====================================================
    // SEND MOTOR SPEED
    // =====================================================

    void runRobot()
    {
        std::array<int, 4> speed_wheel;

        for (int i = 0; i < NMOTORS; i++)
        {
            speed_wheel[i] =
                static_cast<int>(speed_desired_[i]);

            speed_wheel[i] = std::max(
                -static_cast<int>(max_speed_),
                std::min(
                    static_cast<int>(max_speed_),
                    speed_wheel[i]));
        }

        std::stringstream ss;

        ss << speed_wheel[LEFT_UP] << "/"
           << speed_wheel[LEFT_DOWN] << "&"
           << speed_wheel[RIGHT_UP] << "*"
           << speed_wheel[RIGHT_DOWN] << ";";

        std::string serial_data = ss.str();

        try
        {
            if (serial_.isOpen())
            {
                serial_.write(serial_data);
            }
        }
        catch (const serial::IOException& e)
        {
            ROS_ERROR_THROTTLE(
                1.0, "Serial write error: %s", e.what());

            publishMotorError(true);
        }
    }

    // =====================================================
    // SERIAL ENCODER PARSER
    // =====================================================

    bool parseEncoder(const std::string& line)
    {
        std::string data = line;

        // Remove semicolon
        data.erase(
            std::remove(data.begin(), data.end(), ';'),
            data.end());

        // Format: L_UP/L_DOWN&R_UP*R_DOWN

        size_t amp_pos = data.find('&');

        if (amp_pos == std::string::npos)
            return false;

        std::string left = data.substr(0, amp_pos);
        std::string right = data.substr(amp_pos + 1);

        size_t slash_pos = left.find('/');
        size_t star_pos = right.find('*');

        if (slash_pos == std::string::npos ||
            star_pos == std::string::npos)
        {
            return false;
        }

        try
        {
            encoder_total_[LEFT_UP] =
                std::stod(left.substr(0, slash_pos));

            encoder_total_[LEFT_DOWN] =
                std::stod(left.substr(slash_pos + 1));

            encoder_total_[RIGHT_UP] =
                std::stod(right.substr(0, star_pos));

            encoder_total_[RIGHT_DOWN] =
                std::stod(right.substr(star_pos + 1));
        }
        catch (...)
        {
            return false;
        }

        return true;
    }

    // =====================================================
    // READ SERIAL
    // =====================================================

    void readSerial()
    {
        try
        {
            if (!serial_.isOpen())
                return;

            while (serial_.available() > 0)
            {
                std::string line = serial_.readline(
                    128, "\n");

                // Remove whitespace
                line.erase(
                    std::remove(
                        line.begin(),
                        line.end(),
                        '\r'),
                    line.end());

                if (!parseEncoder(line))
                    continue;

                publishMotorError(false);

                if (!first_encoder_received_)
                {
                    for (int i = 0; i < NMOTORS; i++)
                    {
                        last_encoder_[i] = encoder_total_[i];
                    }

                    first_encoder_received_ = true;

                    last_time_encod_ = ros::Time::now();

                    ROS_INFO("First encoder received");

                    continue;
                }

                updatePosition();
            }
        }
        catch (const serial::IOException& e)
        {
            ROS_ERROR_THROTTLE(
                1.0, "Serial read error: %s", e.what());

            publishMotorError(true);
        }
    }

    // =====================================================
    // MOTOR ERROR
    // =====================================================

    void publishMotorError(bool error)
    {
        motor_error_ = error;

        std_msgs::Bool msg;
        msg.data = motor_error_;

        motor_error_pub_.publish(msg);
    }

    // =====================================================
    // STOP ROBOT
    // =====================================================

    void stopRobot()
    {
        speed_desired_.fill(0.0);

        try
        {
            if (serial_.isOpen())
            {
                serial_.write("0/0&0*0;");
            }
        }
        catch (...)
        {
            ROS_WARN("Cannot send stop command");
        }
    }

    // =====================================================
    // MAIN LOOP
    // =====================================================

    void run()
    {
        ros::Rate rate(loop_rate_);

        ROS_INFO("Mecanum robot is running");

        while (ros::ok())
        {
            ros::spinOnce();

            readSerial();

            rate.sleep();
        }

        stopRobot();
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cmd_vel_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher motor_error_pub_;
    ros::Publisher desired_pub_;
    ros::Publisher feedback_pub_;

    tf::TransformBroadcaster tf_broadcaster_;

    serial::Serial serial_;

    // Robot parameters
    double wheel_diameter_;
    int encoder_total_count_;
    double total_length_;

    double coeff_speed_;
    double max_speed_;
    double loop_rate_;

    double cm_per_count_;
    double delta_cvt_;

    int baud_;
    std::string port_;

    bool use_imu_;
    bool publish_tf_;
    bool motor_error_;

    int test_mode_;

    double start_velocity_test_;

    double x_offset_;
    double y_offset_;
    double theta_offset_;

    double distance_move_;
    double axis_move_;
    double velocity_move_;

    std::string odom_frame_;
    std::string base_frame_;

    // Robot state
    std::array<double, 3> robot_pose_;

    std::array<double, 4> encoder_total_;
    std::array<double, 4> last_encoder_;
    std::array<double, 4> speed_desired_;

    double vx_now_;
    double vy_now_;
    double theta_now_;

    double vx_run_;
    double vy_run_;
    double w_run_;

    // Filter
    double a_coeff_;
    double b_coeff_;

    double speed_filter_x_;
    double speed_filter_y_;
    double speed_filter_z_;

    double last_speed_x_;
    double last_speed_y_;
    double last_speed_z_;

    bool first_encoder_received_;

    ros::Time last_time_encod_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mecanum_robot");

    try
    {
        MecanumRobot robot;
        robot.run();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("Mecanum robot exception: %s", e.what());
        return 1;
    }

    ROS_INFO("Mecanum robot node exited");

    return 0;
}
