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


static std::ofstream images_file;

static ignition::math::Quaterniond rpy;

void ssave(){


}

void callback(const ImuConstPtr &imu,
              const JointStateConstPtr &joint) {

    ignition::math::Quaterniond rpy;

    rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);

    roll = rpy.Roll();
    roll *= RadToDeg;

    roll_dot = imu->angular_velocity.x;


    theta = joint->position[1];
    theta *= RadToDeg;
    theta_dot = joint->velocity[1];

//    if (images_file.is_open()) {
//        images_file << tout << "\t" << roll << "\t" << theta << "\t" << roll_dot << "\t" << theta_dot << "\n";
//    }


    tout++;
    ROS_INFO("MOTO pos:[%f] vel:[%f]", roll, roll_dot);
    ROS_INFO("GYRO pos:[%f] vel:[%f]", theta, theta_dot);


}



int main(int argc, char **argv) {

    ros::init(argc, argv, "read_states");
    ros::NodeHandle nh;

    message_filters::Subscriber<Imu> imu_sub(nh, "/imu_base", 100);
    message_filters::Subscriber<JointState> joint_sub(nh, "/moto/joint_states", 100);

//    images_file.open("states.txt");


//    typedef sync_policies::ExactTime<Imu, JointState> MySyncPolicy;
    /// ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    TimeSynchronizer<MySyncPolicy> sync(MySyncPolicy(1000), imu_sub, joint_sub);
    TimeSynchronizer<Imu, JointState> sync(imu_sub, joint_sub,10000);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::spin();

    return 0;
}
