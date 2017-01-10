//
// Created by martinjr on 21.10.16.
//

#include <ros/ros.h>

#include <exploration_scene_manager.h>
#include <moveit_internal_sim.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "haptic_exploration");
    ros::NodeHandle nh;

    //ExplorationSceneManager exploration_scene(nh);

    MoveitInternalSim sim(nh);

    sim.fillExplorationSpace();
//    sim.addCube(3,0,4);
//    sim.addCube(2,0,4);
    sim.addTable();

    sleep(1);

    ros::AsyncSpinner aspin(4);
    aspin.start();

    ros::waitForShutdown();
}