//
// Created by martinjr on 20.10.16.
//

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <kinova_msgs/JointVelocity.h>

#include <moveit/move_group_interface/move_group.h>

#define PI 3.14159265359
#define TWOPI 6.28318530718
#define DEG2RAD (PI/180)

class FakeJointStateDriver {
public:
    FakeJointStateDriver(ros::NodeHandle&);

private:
    ros::NodeHandle &node, action_node;
    ros::Publisher pub;
    sensor_msgs::JointState msg;

    bool jointVelocityMode;
    kinova_msgs::JointVelocity currentJointVelocity;

    ros::Time lastUpdate;

    ros::Timer update_timer;
    ros::Subscriber set_angles_sub, set_carth_vel_sub;

    bool simulate_time;
    double speedup_multiplier;

    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server;

    void update(const ros::TimerEvent&);
    void cbSetJointValues(sensor_msgs::JointStateConstPtr);
    void cbFakeAnglesAction(const control_msgs::FollowJointTrajectoryGoalConstPtr &);
    void cbSetVelocity(kinova_msgs::JointVelocityConstPtr);
};

FakeJointStateDriver::FakeJointStateDriver(ros::NodeHandle &node_handle) :
    node(node_handle),
    action_node(node, "controller"),
    pub(node.advertise<sensor_msgs::JointState>("out/joint_state", 10)),
    action_server(action_node, "follow_joint_trajectory",
                   boost::bind(&FakeJointStateDriver::cbFakeAnglesAction, this, _1), false),
    jointVelocityMode(false)
{
    std::vector<double> v;
    for (int i = 0; i < 6; ++i) {
        v.push_back(3.14);
    }
    for (int i = 6; i < 9 ; ++i) {
        v.push_back(0);
    }

    std::vector<std::string> names;
    for (int j = 1; j < 7; ++j) {
        names.push_back("j2n6a300_joint_" + boost::lexical_cast<std::string>(j));
    }
    for (int j = 1; j < 4; ++j) {
        names.push_back("j2n6a300_joint_finger_" + boost::lexical_cast<std::string>(j));
    }

    msg.position=v;
    msg.name=names;

    node_handle.param("simulateTime", simulate_time, true);
    node_handle.param("/speedup_multiplier", speedup_multiplier, 1.0);


    ros::Rate r(100);
    update_timer=node.createTimer(ros::Duration(r), &FakeJointStateDriver::update, this);
    lastUpdate=ros::Time::now();

    set_angles_sub=node.subscribe("in/set_joint_state", 10, &FakeJointStateDriver::cbSetJointValues, this);
    set_carth_vel_sub=node.subscribe("in/joint_velocity", 10, &FakeJointStateDriver::cbSetVelocity, this);

    action_server.start();
}

/**
     * Normalizes any number to an arbitrary range by assuming the range wraps
     * around when going below min or above max.
     *
     * @param value The number to be normalized
     * @param start Lower threshold
     * @param end   Upper threshold
     * @return Returns the normalized number.
     *
     * Taken form the original FollowJointTrajectoryAction server
     * by Matej Balga
     */
double normalize(const double value, const double start, const double end) {
    const double width = end - start; //
    const double offsetValue = value - start; // value relative to 0

    return ( offsetValue - (floor(offsetValue / width) * width)) +start;
    // + start to reset back to start of original range
}

void FakeJointStateDriver::update(const ros::TimerEvent&) {
    msg.header.stamp=ros::Time::now();

    if (jointVelocityMode) {
        jointVelocityMode = false;

        double dt=(ros::Time::now()-lastUpdate).toSec();

        msg.position[0]=normalize(msg.position[0]+(currentJointVelocity.joint1*DEG2RAD)*dt, -TWOPI, TWOPI);
        msg.position[1]=normalize(msg.position[1]+(currentJointVelocity.joint2*DEG2RAD)*dt, 0, TWOPI);
        msg.position[2]=normalize(msg.position[2]+(currentJointVelocity.joint3*DEG2RAD)*dt, 0, TWOPI);
        msg.position[3]=normalize(msg.position[3]+(currentJointVelocity.joint4*DEG2RAD)*dt, -TWOPI, TWOPI);
        msg.position[4]=normalize(msg.position[4]+(currentJointVelocity.joint5*DEG2RAD)*dt, -TWOPI, TWOPI);
        msg.position[5]=normalize(msg.position[5]+(currentJointVelocity.joint6*DEG2RAD)*dt, -TWOPI, TWOPI);
    }

    lastUpdate=ros::Time::now();

    pub.publish(msg);
}

void FakeJointStateDriver::cbSetJointValues(sensor_msgs::JointStateConstPtr in) {
    ROS_INFO("Received joint value update");
    msg.position=in->position;
}

void FakeJointStateDriver::cbSetVelocity(kinova_msgs::JointVelocityConstPtr velocity) {
    jointVelocityMode=true;
    currentJointVelocity=*velocity;
}

void FakeJointStateDriver::cbFakeAnglesAction(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
    trajectory_msgs::JointTrajectoryPoint goal_waypoint =
            goal->trajectory.points[goal->trajectory.points.size() - 1];

    ros::Time start_time = ros::Time::now();

    ROS_INFO("Executing trajectory with speedup %fx", speedup_multiplier);

    if (simulate_time) {
        for (unsigned int i = 0; i < goal->trajectory.points.size(); i++) {

            // Get the waypoint to be reached
            trajectory_msgs::JointTrajectoryPoint waypoint =
                    goal->trajectory.points[i];

            ros::Rate r(100);   // The loop below will run at 100Hz

            sensor_msgs::JointState start = msg;
            ros::Time waypointTime = ros::Time::now();

            ROS_INFO("Velocity wpt [%.3f] [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", waypoint.time_from_start.toSec(),
                     waypoint.velocities[0], waypoint.velocities[1], waypoint.velocities[2], waypoint.velocities[3],
                     waypoint.velocities[4], waypoint.velocities[5]);

            while (waypoint.time_from_start.toSec() / speedup_multiplier >= (ros::Time::now() - start_time).toSec()) {
                ros::spinOnce();

                if (action_server.isPreemptRequested() || !ros::ok()) {
                    action_server.setPreempted();
                    return;
                }

                ros::Duration dt = ros::Time::now() - waypointTime;

                for (int j = 0; j < 6; ++j) {
                    msg.position[j] = start.position[j] + waypoint.velocities[j] * speedup_multiplier * dt.toSec();
                }

                // Ensures executing at 100Hz
                r.sleep();
            }

            for (int j = 0; j < 6; ++j) {
                msg.position[j] = waypoint.positions[j];
            }
        }
    }
    else {
        for (int j = 0; j < 6; ++j) {
            msg.position[j] = goal_waypoint.positions[j];
        }
    }

    action_server.setSucceeded();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "j2n6a300_driver");

    ros::NodeHandle nh;

    FakeJointStateDriver driver(nh);

    ros::spin();
}
