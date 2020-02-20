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
double x1 = 0;
double x2 = 0;
double x3 = 0;
double theta, theta_dot;
double value;
double RadToDeg = 180 / M_PI;

static int tout = 0;

std::string decimal(int r);


static std::ofstream states;


static ignition::math::Quaterniond rpy;
class Read_States {
private:
    std::ofstream states;
    ros::NodeHandle nh;
    ros::Subscriber sub_imu1;
    ros::Subscriber sub_theta1;


    ignition::math::Quaterniond rpy;
public:
    Read_States() {
        sub_theta1 = nh.subscribe("/moto/joint_states", 10, &Read_States::read_theta, this);
        sub_imu1 = nh.subscribe("/moto/moto/imu_base", 10, &Read_States::callback, this);
        states.open("observer_states.txt");
    }

    ~Read_States() {}

    void read_theta(const sensor_msgs::JointStateConstPtr &joint) {
        theta = joint->position[1];
        theta_dot = joint->velocity[1];
//        ROS_INFO("x2: [%f]", y_2);
    }

    void callback(const sensor_msgs::Imu::ConstPtr &imu) {

        ignition::math::Quaterniond rpy;

        rpy.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        roll = rpy.Roll();
        roll_dot = imu->angular_velocity.x;
        x1 = roll;
        x2 = theta;
        x3 = roll_dot;

        if (states.is_open()) {
            states << tout << "\t" << x1 << "\t" << x2 << "\t" << x3 << "\n";
        }
        tout++;
        ROS_INFO("x1 [%f], x2: [%f], x3: [%f]", x1, x2, x3);
    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "read_states");
    ros::NodeHandle nh;
    states.open("real_states.txt");
    Read_States ReadStates;

    /*
    message_filters::Subscriber<Imu> imu_sub(nh, "/imu_base", 1000);
    message_filters::Subscriber<JointState> joint_sub(nh, "/moto/joint_states", 1000);

//    images_file.open("states.txt");


//    typedef sync_policies::ExactTime<Imu, JointState> MySyncPolicy;
    /// ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    TimeSynchronizer<MySyncPolicy> sync(MySyncPolicy(1000), imu_sub, joint_sub);
    TimeSynchronizer<Imu, JointState> sync(imu_sub, joint_sub,100000);
    sync.registerCallback(boost::bind(&callback, _1, _2));
*/

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
