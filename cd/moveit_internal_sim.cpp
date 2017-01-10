//
// Created by martinjr on 24.10.16.
//

#include "moveit_internal_sim.h"


#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#define PI 3.14159265359
#define TWOPI 6.28318530718
#define DEG2RAD (PI/180)

MoveitInternalSim::MoveitInternalSim(ros::NodeHandle &parent_node) :
    node(parent_node),
    scene_pub(node.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1)),
    attached_pub(node.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1)),
    joint_vel_pub(node.advertise<kinova_msgs::JointVelocity>("/j2n6a300_driver/in/joint_velocity", 10)),
    expl_pose_service(node.advertiseService("set_exploration_pose", &MoveitInternalSim::setPoseSrvCallback, this)),
    expl_move_service(node.advertiseService("exploration_move", &MoveitInternalSim::moveSrvCallback, this)),
    monitor("robot_description"),
    scene(monitor.getPlanningScene()),
    manipulator(scene->getRobotModel()->getJointModelGroup("manipulator")),
    move_group("manipulator"),
    contact(false), stick_attached(false)
{
    ORIENT_FWD.x=0.707;
    ORIENT_FWD.y=0;
    ORIENT_FWD.z=0;
    ORIENT_FWD.w=0.707;

    ORIENT_DOWN.w=0.401;
    ORIENT_DOWN.x=0.913;
    ORIENT_DOWN.y=0.064;
    ORIENT_DOWN.z=-0.028;

    cart_vel_sub=node.subscribe("in/test_cart_vel", 10, &MoveitInternalSim::cbCartVelocity, this);

    opto_sub=node.subscribe("/optoforce", 10, &MoveitInternalSim::cbOptoforce, this);

    move_group.setPlannerId("BKPIECEkConfigDefault");
    move_group.setEndEffectorLink("j2n6a300_end_effector");
    move_group.setPoseReferenceFrame("j2n6a300_link_base");
}


// taken from http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void MoveitInternalSim::cbOptoforce(geometry_msgs::Vector3ConstPtr force3) {
    if (!contact && (fabs(force3->x) > 2 || fabs(force3->y) > 2 || fabs(force3->z) > 4)) {
        contact=true;
        ROS_INFO_STREAM("Detected optoforce contact at "<<force3->x<<", "<<force3->y<<", "<<force3->z);
        if (moving_planned) move_group.stop();
    }

    if (contact && (fabs(force3->x) < 2 && fabs(force3->y) < 2 && fabs(force3->z) < 4)) {
        contact=false;
        ROS_INFO_STREAM("RESET optoforce contact at "<<force3->x<<", "<<force3->y<<", "<<force3->z);
    }
}

void MoveitInternalSim::cbCartVelocity(kinova_msgs::PoseVelocityConstPtr vel_msg) {
    Eigen::VectorXd twist(6);
    Eigen::Vector3d cart_vel, angle_vel;

    cart_vel << vel_msg->twist_linear_x, vel_msg->twist_linear_y, vel_msg->twist_linear_z;
    angle_vel << vel_msg->twist_angular_x*DEG2RAD, vel_msg->twist_angular_y*DEG2RAD, vel_msg->twist_angular_z*DEG2RAD;

    publishCarthesianMove(cart_vel, angle_vel);
}

bool MoveitInternalSim::publishCarthesianMove(Eigen::Vector3d cart_vel, Eigen::Vector3d angle_vel) {
    robot_state::RobotStatePtr state=move_group.getCurrentState();

    cart_vel=cartVelToCorrectFrame(cart_vel);

    //ROS_INFO_STREAM("Velocity in link_6"<<cart_vel);

    Eigen::VectorXd twist(6);
    twist << cart_vel, angle_vel;

    Eigen::VectorXd joint_veloc(6);

    bool found=false;
    int attempts=0;
    while (!found and attempts<4) {
        found=true;

        state->computeVariableVelocity(manipulator, joint_veloc, twist, manipulator->getLinkModel("j2n6a300_link_6"));

        for (int i = 0; i < 6; ++i) {
            if (fabs(joint_veloc[i]) > 0.8) {
                ROS_WARN_STREAM("Setting too high velocities for " << i << ": " << joint_veloc[i] << ", slowing down");
                twist=twist/2;
                found=false;
                attempts++;
            }
        }
    }

    //ROS_INFO_STREAM("Calculated joint velocity: "<<joint_veloc[0]<<", "<<joint_veloc[1]<<", "<<joint_veloc[2]
    //                                             <<", "<<joint_veloc[3]<<", "<<joint_veloc[4]<<", "<<joint_veloc[5]);

    if (!found) {
        ROS_ERROR_STREAM("Velocity irretrievably high");
        return false;
    }

    for (int j = 0; j < 6; ++j) {
        if (fabs(state->getVariablePosition(j)+joint_veloc[j]*0.01) > TWOPI) {
            ROS_ERROR_STREAM("Would violate joint limits, don't do that");
            return false;
        }
    }

    //ROS_INFO_STREAM("Joint velocities for movement at " << twist << ": " << joint_veloc);

    kinova_msgs::JointVelocity jv_msg;

    // KINOVA velocities in degrees
    jv_msg.joint1=joint_veloc[0]/DEG2RAD;
    jv_msg.joint2=joint_veloc[1]/DEG2RAD;
    jv_msg.joint3=joint_veloc[2]/DEG2RAD;
    jv_msg.joint4=joint_veloc[3]/DEG2RAD;
    jv_msg.joint5=joint_veloc[4]/DEG2RAD;
    jv_msg.joint6=joint_veloc[5]/DEG2RAD;

    joint_vel_pub.publish(jv_msg);

    return true;
}

Eigen::Vector3d MoveitInternalSim::cartVelToCorrectFrame(Eigen::Vector3d cart_vel) {
    geometry_msgs::Vector3Stamped vm, nv;
    vm.header.frame_id="root";

    //ROS_INFO_STREAM("Velocity in root: "<<cart_vel);

    tf::vectorEigenToMsg(cart_vel, vm.vector);

    tf_listen.transformVector("j2n6a300_link_6", vm, nv);

    tf::vectorMsgToEigen(nv.vector, cart_vel);

    return cart_vel;
}

bool MoveitInternalSim::moveSrvCallback(jaco_moveit::ExplorePoint::Request& request, jaco_moveit::ExplorePoint::Response& response) {
    ros::Rate r(50);

    if (!stick_attached) {
        attachStick();
    }

    double dt=r.expectedCycleTime().toSec();
    double speed=0.1;
    double KP=2;

    Eigen::VectorXd twist(6);
    Eigen::Vector3d cart_vel, angle_vel;


    geometry_msgs::Point t=getCellPoint(cx+request.x,cy+request.y,cz+request.z);

    bool move_x, move_y, move_z;
    move_x=abs(request.x)>0;
    move_y=abs(request.y)>0;
    move_z=abs(request.z)>0;

    if (move_x+move_y+move_z>1) {
        ROS_ERROR_STREAM("Moving along more than one axis! ("<<move_x<<", "<<move_y<<", "<<move_z<<")");
        return false;
    }


    Eigen::Vector3d current_pose;

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    tf::pointMsgToEigen(move_group.getCurrentPose().pose.position, current_pose);
    ROS_INFO_STREAM("Current pose: "<<current_pose[0]<<","<<current_pose[1]<<","<<current_pose[2]);
    ROS_INFO_STREAM("Target pose:"<<t.x<<", "<<t.y<<","<<t.z);

    moving_jacob=true;

    bool ok=false;
    int cause=-1;
    while (true) { // runs at 50Hz
        tf::pointMsgToEigen(move_group.getCurrentPose().pose.position, current_pose);

        if (move_x) {
            cart_vel << sgn(request.x)*speed, KP*(t.y-current_pose[1]), KP*(t.z-current_pose[2]);
            if (fabs(t.x-current_pose[0]) < 1e-2) {
                ok=true;
                break;
            }

            if (fabs(t.y-current_pose[1]) > 0.3 || fabs(t.z-current_pose[2]) > 0.3) {
                cause=1;
                break;
            }
        }
        if (move_y) {
            cart_vel << KP*(t.x-current_pose[0]), -sgn(request.y)*speed, KP*(t.z-current_pose[2]); // our y is inverted
            if (fabs(t.y-current_pose[1]) < 1e-2) {
                ok=true;
                break;
            }

            if (fabs(t.x-current_pose[0]) > 0.3 || fabs(t.z-current_pose[2]) > 0.3) {
                cause=1;
                break;
            }
        }
        if (move_z) {
            cart_vel << KP*(t.x-current_pose[0]), KP*(t.y-current_pose[1]), sgn(request.z)*speed;
            if (fabs(t.z-current_pose[2]) < 1e-2) {
                ok=true;
                break;
            }

            if (fabs(t.y-current_pose[1]) > 0.3 || fabs(t.x-current_pose[0]) > 0.3) {
                cause=1;
                break;
            }
        }


        //ROS_INFO_STREAM("Controlling velocity: "<<cart_vel[0]<<", "<<cart_vel[1]<<", "<<cart_vel[2]);

        Eigen::Vector3d real_cart_vel=cartVelToCorrectFrame(cart_vel);
        angle_vel << 0, 0, 0;
        twist << real_cart_vel, angle_vel;


        planning_scene::PlanningScenePtr scene=monitor.getPlanningScene();
        robot_state::RobotState sc_state=scene->getCurrentStateNonConst();

        sc_state.setFromDiffIK(manipulator, twist, "j2n6a300_link_6", dt);
        sc_state.update();

        collision_result.clear();
        scene->setCurrentState(sc_state);
        scene->checkCollision(collision_request, collision_result);

        if (collision_result.collision) {
            cause=3;

            for (std::map< std::pair<std::string,std::string>, std::vector<collision_detection::Contact> >::iterator i = collision_result.contacts.begin();
                    i != collision_result.contacts.end(); ++i) {

                ROS_INFO_STREAM("Detected potential contact between "
                                        << i -> first.first << " and "
                                        << i -> first.second);
            }

            break;
        }

        // publish twice to keep up with the driver
        bool correct=this->publishCarthesianMove(cart_vel, angle_vel);
        this->publishCarthesianMove(cart_vel, angle_vel);

        if (!correct) {
            cause=2;
            break;
        }

        if (!fixing && contact) { // if we hit something (but ignore it if we are just returning)
            cause=4;
            break;
        }

        r.sleep();
    }

    moving_jacob=false;


    ROS_INFO_STREAM("Reached: "<<current_pose[0]<<","<<current_pose[1]<<","<<current_pose[2]);

    if (ok) {
        double other_eps=0.1;
        if ((move_x && (fabs(current_pose[1]-t.y)>other_eps || fabs(current_pose[2]-t.z)>other_eps)) ||
                (move_y && (fabs(current_pose[0]-t.x)>other_eps || fabs(current_pose[2]-t.z)>other_eps)) ||
                (move_z && (fabs(current_pose[1]-t.y)>other_eps || fabs(current_pose[0]-t.x)>other_eps))) {
            ROS_WARN_STREAM("Followed OK trajectory to a distant point");
            ROS_WARN_STREAM("End pose: "<<current_pose[0]<<","<<current_pose[1]<<","<<current_pose[2]);
            response.obstacle=false;
            response.reached=false;
            return true;
        }

        // set success
        ROS_INFO_STREAM("Movement succeeded");
        cx+=request.x; cy+=request.y; cz+=request.z;
        removeCube(cx,cy,cz);
        response.obstacle=false;
        response.reached=true;
    }
    else {
        switch (cause) {
            case 1: // diverged from path
            case 2: // singularity in speeds
                ROS_WARN_STREAM("Reached nonsense pose, do something about it!");
                ROS_WARN_STREAM("End pose: "<<current_pose[0]<<","<<current_pose[1]<<","<<current_pose[2]);
                response.obstacle=false;
                response.reached=false;
                return true;
                break;

            case 3: // would collide
                ROS_WARN_STREAM("Movement would hit an obstacle");

                if (!fixing) {
                    cx += request.x;
                    cy += request.y;
                    cz += request.z;

                    jaco_moveit::ExplorePointRequest r2;
                    jaco_moveit::ExplorePointResponse re;
                    r2.x = -request.x;
                    r2.y = -request.y;
                    r2.z = -request.z;

                    fixing=true;
                    bool returned=moveSrvCallback(r2, re);
                    fixing=false;

                    response.obstacle=true;
                    response.reached=false;
                    return returned;
                }
                else {
                    ROS_ERROR_STREAM("FAILED: hit an obstacle!");

                    cx += request.x;
                    cy += request.y;
                    cz += request.z;

                    return false;
                }
                break;

            case 4: // detected obstacle
                ROS_WARN_STREAM("Detected an obstacle");

                if (!fixing) {
                    cx += request.x;
                    cy += request.y;
                    cz += request.z;

                    jaco_moveit::ExplorePointRequest r2;
                    jaco_moveit::ExplorePointResponse re;
                    r2.x = -request.x;
                    r2.y = -request.y;
                    r2.z = -request.z;

                    fixing=true;
                    bool returned=moveSrvCallback(r2, re);
                    fixing=false;

                    response.obstacle=true;
                    response.reached=true;
                    return returned;
                }
                else {
                    ROS_ERROR_STREAM("FAILED: hit an obstacle when returning!!");

                    cx += request.x;
                    cy += request.y;
                    cz += request.z;

                    return false;
                }
                break;

            default:
                ROS_ERROR("Failed without cause!");
        }
    }

    return ok;
}


typedef moveit::planning_interface::MoveGroup::Plan Plan;

bool MoveitInternalSim::setPoseSrvCallback(jaco_moveit::ExplorePoint::Request& request, jaco_moveit::ExplorePoint::Response& response) {
    ROS_INFO_STREAM("ExplorePoint service called to explore " << request.x << "," << request.y << "," << request.z);
    //setRobotPose(request.x, request.y, request.z);

    if (!stick_attached) {
        attachStick();
    }

    move_group.setStartStateToCurrentState();
    // move_group.setJointValueTarget(robot_state);

    geometry_msgs::Pose pose;

    pose.position=getCellPoint(request.x, request.y, request.z);

    if (request.z<=2) {
        pose.orientation = ORIENT_DOWN;
    }
    else pose.orientation = ORIENT_FWD;

    //move_group.setJointValueTarget(pose);
    move_group.setPoseTarget(pose);

    removeCube(request.x, request.y, request.z);

    Plan p;
    ROS_INFO_STREAM("Planning to "<< pose);
    int tries = 0;
    bool plan_result;

    do {
        plan_result=move_group.plan(p);
    } while (!plan_result && (tries++)<=3);

    ROS_INFO_STREAM("done");

    if (!plan_result) {
        ROS_ERROR_STREAM("Planning failed, destination unreachable");
        addCube(request.x, request.y, request.z);
        response.obstacle=true;
        response.reached=false;
        return true;
    }
    else {
        ROS_INFO_STREAM("Plan found, execute it now.");
        response.reached=true;
    }


    ROS_INFO("Executing...");
    moving_planned=true;
    // blocks, returns true if moved successfuly, else returns false
    bool success=move_group.execute(p);
    moving_planned=false;
    ROS_INFO("done.");

    if (success) {
        cx=request.x;
        cy=request.y;
        cz=request.z;

        response.obstacle=false;
        ROS_INFO_STREAM("Plan executed fine");
    }
    else {
        response.obstacle=true;
        addCube(request.x, request.y, request.z);

        double tx = OI+request.x*D;
        double ty = -(OJ+request.y*D);
        double tz = OK+request.z*D;

        // follow the trajectory backwards
        ROS_INFO_STREAM("Retracing plan back out of the obstacle");
        std::vector<trajectory_msgs::JointTrajectoryPoint> traj = p.trajectory_.joint_trajectory.points;
        ros::Rate r(100);
        kinova_msgs::JointVelocity vel;

        for (int i = traj.size()-2; i >=0; --i) {
            trajectory_msgs::JointTrajectoryPoint current=traj[i];
            ros::Duration t=traj[i+1].time_from_start-traj[i].time_from_start;

            vel.joint1=-current.velocities[0]/DEG2RAD;
            vel.joint2=-current.velocities[1]/DEG2RAD;
            vel.joint3=-current.velocities[2]/DEG2RAD;
            vel.joint4=-current.velocities[3]/DEG2RAD;
            vel.joint5=-current.velocities[4]/DEG2RAD;
            vel.joint6=-current.velocities[5]/DEG2RAD;

            ros::Time start=ros::Time::now();
            while (ros::Time::now()-start < t) {
                joint_vel_pub.publish(vel);

                r.sleep();
            }

            geometry_msgs::Point tip=getStickTip();
            if (tip.x<tx-D/2 || tip.x>tx+D/2 || tip.y<ty-D/2 || tip.y>ty+D/2 || tip.z<tz-D/2 || tip.z>tz+D/2) {
                ROS_INFO_STREAM("obstacle left successfuly");
                break;
            }
        }

        ROS_WARN_STREAM("Execution stopped, you should do something else");
    }

    return true;
}


void MoveitInternalSim::fillExplorationSpace() {
    moveit_msgs::CollisionObject cube;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for (int i = 0; i < FIELD_W; ++i) {
        for (int j = 0; j < FIELD_D; ++j) {
            for (int k = 0; k < FIELD_H; ++k) {
                geometry_msgs::Point t=getCellPoint(i,j,k);

                double r=sqrt(t.x*t.x + t.y*t.y + t.z*t.z);

                ROS_INFO_STREAM("cube "<<i<<":"<<j<<":"<<k<<" = "<<t.x<<", "<<t.y<<", "<<t.z<<" - r="<<r);

                if (r <= 0.9 && r >= 0.45) {
                    cube=getCube(i,j,k);
                    collision_objects.push_back(cube);
                }
            }
        }
    }

    planning_scene_interface.addCollisionObjects(collision_objects);
}

moveit_msgs::CollisionObject MoveitInternalSim::getCube(int i, int j, int k) {
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "root";
    obj.id = "cube" + boost::lexical_cast<std::string>(i) + ","
             + boost::lexical_cast<std::string>(j) + ","
             + boost::lexical_cast<std::string>(k);

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = D;
    primitive.dimensions[1] = D;
    primitive.dimensions[2] = D;

    pose.position.x = OI+i*D;
    pose.position.y = -(OJ+j*D);
    pose.position.z = OK+k*D;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);

    obj.operation=obj.ADD;

    return obj;
}

void MoveitInternalSim::addCube(int i, int j, int k) {
    moveit_msgs::CollisionObject cube=getCube(i,j,k);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cube);
    planning_scene_interface.addCollisionObjects(collision_objects);

    //scene->processCollisionObjectMsg(obj);
}

void MoveitInternalSim::addTable() {
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "root";
    obj.id = "table";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 2;
    primitive.dimensions[2] = 0.04;

    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.03;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);

    obj.operation=obj.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(obj);
    planning_scene_interface.addCollisionObjects(collision_objects);

    //scene->processCollisionObjectMsg(obj);
}

void MoveitInternalSim::removeCube(int i, int j, int k) {
    std::vector<std::string> cubes;
    cubes.push_back("cube" + boost::lexical_cast<std::string>(i) + ","
                    + boost::lexical_cast<std::string>(j) + ","
                    + boost::lexical_cast<std::string>(k));

    planning_scene_interface.removeCollisionObjects(cubes);
    // scene->processCollisionObjectMsg(obj);
    // redrawRviz();
}

geometry_msgs::Point MoveitInternalSim::getCellPoint(int i, int j, int k) {
    geometry_msgs::Point p;

    p.x = OI+i*D;
    p.y = -(OJ+j*D-STICK_LEN-0.02);
    p.z = OK+k*D;

    if (k<=2) {
        p.y-=STICK_LEN-sqrt(STICK_LEN*STICK_LEN/2);
        p.z+=sqrt(STICK_LEN*STICK_LEN/2)-0.03;
    }


    return p;
}

void MoveitInternalSim::attachStick() {
    moveit_msgs::CollisionObject att_object;
    att_object.id = "attached_stick";
    att_object.operation = moveit_msgs::CollisionObject::ADD;
    att_object.header.frame_id = "root";
    att_object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive object;
    object.type = shape_msgs::SolidPrimitive::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = .3;
    object.dimensions[1] = 0.005;
    att_object.primitives.push_back(object);

    geometry_msgs::PoseStamped p;
    p.pose.position.x=0;
    p.pose.position.y=0;
    p.pose.position.z=object.dimensions[0]/2;
    p.pose.orientation.w=1;
    p.header.frame_id="j2n6a300_end_effector";

    geometry_msgs::PoseStamped ee_pose;
    ee_pose.header.frame_id="root";

    tf_listen.waitForTransform("root", "j2n6a300_end_effector", ros::Time::now(), ros::Duration(5));
    tf_listen.transformPose("root", p, ee_pose);


    att_object.primitive_poses.push_back(ee_pose.pose);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(att_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::Duration(0.2).sleep();

    move_group.attachObject("attached_stick", "j2n6a300_link_6");

    stick_attached=true;
}

void MoveitInternalSim::detachStick() {
    std::vector<std::string> stick;
    stick.push_back("attached_stick");
    move_group.detachObject("attached_stick");
    planning_scene_interface.removeCollisionObjects(stick);
}

geometry_msgs::Point MoveitInternalSim::getStickTip() {
    geometry_msgs::PointStamped tip_eeframe;
    tip_eeframe.header.frame_id="j2n6a300_link_6";
    tip_eeframe.point.x=0;
    tip_eeframe.point.y=0;
    tip_eeframe.point.z=.3;

    geometry_msgs::PointStamped tip_root;
    tip_root.header.frame_id="root";

    tf_listen.transformPoint("root", tip_eeframe, tip_root);

    return tip_root.point;
}

void MoveitInternalSim::redrawRviz() {
    moveit_msgs::PlanningScene scene_msg;
    scene->setCurrentState(*move_group.getCurrentState());
    scene->getPlanningSceneMsg(scene_msg);
    scene_pub.publish(scene_msg);
}

void MoveitInternalSim::test() {
    ros::Rate r(1);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;


    moveit_msgs::CollisionObject obj;
    obj.header.frame_id="root";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);

    moveit_msgs::PlanningScene scene_msg;

    for (int i = 0; i < 5; ++i) {
        geometry_msgs::PoseStamped hand_pose;
        hand_pose.header.frame_id="root";
        hand_pose.pose.position.x=0.15*i-0.35;
        hand_pose.pose.position.y=-0.30;
        hand_pose.pose.position.z=0.5;

        hand_pose.pose.orientation=ORIENT_FWD;


        redrawRviz();

        scene->checkCollision(collision_request, collision_result);
        ROS_INFO_STREAM("Test: Current state is "
                                << (collision_result.collision ? "in" : "not in")
                                << "  collision");

        r.sleep();
    }

    scene->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test -1: Current state is "
                            << (collision_result.collision ? "in" : "not in")
                            << " collision");
}