#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <string>


#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <fstream>
#include <ignition/math/Quaternion.hh>

using namespace sensor_msgs;

using namespace message_filters;

/// IMU read variables
double roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot;
/// Gyro joint variables
double theta, theta_dot;
std::string name;
double value;
double RadToDeg = 180 / M_PI;

static int tout = 0;

std::string decimal(int r);


static std::ofstream states;


static ignition::math::Quaterniond rpy;

void callback(const ImuConstPtr &imu,
                  const JointStateConstPtr &joint) {

    ignition::math::Quaterniond rpy;

    rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

    roll = rpy.Roll();
//    roll *= RadToDeg;

    roll_dot = imu->angular_velocity.x;


    theta = joint->position[1];
//    theta *= RadToDeg;
    theta_dot = joint->velocity[1];

    if (states.is_open()) {
        states << tout << "\t" << roll << "\t" << theta << "\t" << roll_dot << "\n";
    }


    tout++;
//    ROS_INFO("MOTO pos:[%f] vel:[%f]", roll, roll_dot);
//    ROS_INFO("GYRO pos:[%f] vel:[%f]", theta, theta_dot);
    ROS_INFO("x1 [%f], x2: [%f], x3: [%f]", roll, theta, roll_dot);


}



int main(int argc, char **argv) {

    ros::init(argc, argv, "read_states");
    ros::NodeHandle nh;
    states.open("real_states.txt");
    message_filters::Subscriber<Imu> imu_sub(nh, "/imu_base", 1000);
    message_filters::Subscriber<JointState> joint_sub(nh, "/moto/joint_states", 1000);

//    images_file.open("states.txt");


//    typedef sync_policies::ExactTime<Imu, JointState> MySyncPolicy;
    /// ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    TimeSynchronizer<MySyncPolicy> sync(MySyncPolicy(1000), imu_sub, joint_sub);
    TimeSynchronizer<Imu, JointState> sync(imu_sub, joint_sub,100000);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::spin();

    return 0;
}
