// cartesian_move_tester: 
// wsn, July, 2018
// a cartesian-move action client for testing painting robot

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <cartesian_planner/cart_moveAction.h>
//#include <cartesian_planner/cart_motion_commander.h>
//#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>

//change the following for setting height of flange for cartesian moves in horizontal plane
// 0.14 seems good for pointer...close to ground

const float flange_z_above_ground = 0.14;
using namespace std;

    double arrival_time = 6.0; //ADJUST THIS FOR SPEED CHANGE

int main(int argc, char** argv) {
    ros::init(argc, argv, "cartesian_move_tester"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     

    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    double flange_height = 0.25; //keep tool flange this high above ground
    Eigen::Affine3d a_tool_start, a_tool_end;

    int rtn_val;
    int njnts;

    geometry_msgs::PoseStamped tool_pose, goal_tool_pose, start_tool_pose;
    
  
    Eigen::VectorXd q_des_vec;
    Eigen::Matrix3d R_des;
    Eigen::Vector3d p_des, xvec_des, yvec_des, zvec_des;
    zvec_des << 0, 0, -1; //point down
    xvec_des << 0, 1, 0;
    yvec_des = zvec_des.cross(xvec_des);
    R_des.col(0) = xvec_des;
    R_des.col(1) = yvec_des;
    R_des.col(2) = zvec_des;

    //pdes w/rt system frame:
    p_des << 0.2, 0, flange_z_above_ground;

    CartMotionCommander cart_motion_commander;

    ROS_INFO("commanding move to home pose");
    //int plan_jspace_traj_current_to_waiting_pose(int nsteps, double arrival_time);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_waiting_pose(30, 2.0);
    //send command to execute planned motion
    rtn_val = cart_motion_commander.execute_planned_traj();

    ROS_WARN("commanding motion to 25cm above ground, pointing down: ");
    a_tool_end.linear() = R_des;
    a_tool_end.translation() = p_des;
    ROS_INFO_STREAM("p_des = "<<p_des.transpose()<<endl);

    //convert affine to geometry_msgs/PoseStamped, and specify reference frame as system_ref_frame:
    int nsteps = 50;

    goal_tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(a_tool_end, "system_ref_frame");
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, goal_tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        //ros::Duration(arrival_time+0.2).sleep(); 
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    double x_des, y_des;
    a_tool_start = a_tool_end;
    while (ros::ok()) {
        cout << "enter desired x: ";
        cin >> x_des;
        if (false) {
            ROS_WARN("NOPE...too close to bumper");
        } else {
            cout << "enter desired y: ";
            cin >> y_des;
            p_des[0] = x_des;
            p_des[1] = y_des;
            a_tool_end.translation() = p_des;
            goal_tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(a_tool_end, "system_ref_frame");
            rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, arrival_time, goal_tool_pose);
            if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
                ROS_INFO("successful plan; command execution of trajectory");
                rtn_val = cart_motion_commander.execute_planned_traj();
                ROS_INFO_STREAM("p_des = "<<p_des.transpose()<<endl);
            } else {
                ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            }

        }

    }



}

