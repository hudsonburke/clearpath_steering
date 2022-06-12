// Author Hudson Burke
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include "pubSysCls.h"

using namespace sFnd;

#define ACC_LIM_RPM_PER_SEC 100000
#define VEL_LIM_RPM 700
#define NUM_MOVES 5
#define TIME_TILL_TIMEOUT 10000 // The timeout used for homing(ms)
#define SC_HUB_PORT 0           // only using 1 SC_HUB, so it will always be Port 0
#define NUM_NODES 4             // 4 motors/nodes

#define FL_NODE_NUM 0
#define FR_NODE_NUM 1
#define BL_NODE_NUM 2
#define BR_NODE_NUM 3

#define PI 3.14159265358979323846
#define TOTAL_ENCODER_COUNTS 800 // manual page 115

INode *FL_Node;
INode *FR_Node;
INode *BL_Node;
INode *BR_Node;

int32_t rad_to_encoder_counts(double radians) // TODO: Find number of encoder counts
{
    return (int32_t)TOTAL_ENCODER_COUNTS * radians / (2 * PI);
}

void callback_fl(const std_msgs::Float64 &fl_command)
{
    int32_t target = rad_to_encoder_counts(fl_command.data);
    FL_Node->Motion.MoveWentDone();              // Clear the rising edge Move done register
    FL_Node->Motion.MovePosnStart(target, true); // Move to absolute position
}

void callback_fr(const std_msgs::Float64 &fr_command)
{
    int32_t target = rad_to_encoder_counts(fr_command.data);
    FR_Node->Motion.MoveWentDone();              // Clear the rising edge Move done register
    FR_Node->Motion.MovePosnStart(target, true); // Move to absolute position
}

void callback_bl(const std_msgs::Float64 &bl_command)
{
    int32_t target = rad_to_encoder_counts(bl_command.data);
    BL_Node->Motion.MoveWentDone();              // Clear the rising edge Move done register
    BL_Node->Motion.MovePosnStart(target, true); // Move to absolute position
}

void callback_br(const std_msgs::Float64 &br_command)
{
    int32_t target = rad_to_encoder_counts(br_command.data);
    BR_Node->Motion.MoveWentDone();              // Clear the rising edge Move done register
    BR_Node->Motion.MovePosnStart(target, true); // Move to absolute position
}

int main(int argc, char *argv[])
{
    // setup
    size_t portCount = 0;
    std::vector<std::string> comHubPorts;

    SysManager *myMgr = SysManager::Instance(); // Create System Manager myMgr
    SysManager::FindComHubPorts(comHubPorts);
    ROS_INFO("Found %d SC Hubs", comHubPorts.size());

    if (comHubPorts.size() > 1)
    {
        ROS_ERROR("Too many ports connected.");
        return -1;
    }
    else if (comHubPorts.size() < 1)
    {
        ROS_ERROR("No ports connected.");
        return -1;
    }
    myMgr->ComHubPort(SC_HUB_PORT, comHubPorts[SC_HUB_PORT].c_str()); // define the first SC Hub port (port 0) to be associated with COM portnum (as seen in device manager)
    myMgr->PortsOpen(SC_HUB_PORT);                                    // Open the port

    IPort &myPort = myMgr->Ports(SC_HUB_PORT);
    ROS_INFO(" Port[%d]: state=%d, nodes=%d", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

    // Once the code gets past this point, it can be assumed that the Port has been opened without issue
    // Now we can get a reference to our port object which we will use to access the node objects
    size_t iNode = 0;

    // Create a shortcut reference for each node

    // enable and home all Nodes/Motors
    for (iNode = 0; iNode < myPort.NodeCount(); iNode++)
    {
        INode &theNode = myPort.Nodes(iNode); // TODO: this may not work since I assign the reference to another object outside
        theNode.EnableReq(false);             // Ensure Node is disabled before loading config file

        myMgr->Delay(200);

        // theNode.Setup.ConfigLoad("Config File path");

        ROS_INFO("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
        ROS_INFO("            userID: %s\n", theNode.Info.UserID.Value());
        ROS_INFO("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
        ROS_INFO("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
        ROS_INFO("             Model: %s\n", theNode.Info.Model.Value());

        // The following statements will attempt to enable the node.  First,
        // any shutdowns or NodeStops are cleared, finally the node is enabled
        theNode.Status.AlertsClear();   // Clear Alerts on node
        theNode.Motion.NodeStopClear(); // Clear Nodestops on Node
        theNode.EnableReq(true);        // Enable node
        // At this point the node is enabled
        ROS_INFO("Node \t%zi enabled\n", iNode);
        double timeout = myMgr->TimeStampMsec() +
                         TIME_TILL_TIMEOUT; // define a timeout in case the node is unable to enable
                                            // This will loop checking on the Real time values of the node's Ready status
        while (!theNode.Motion.IsReady())
        {
            if (myMgr->TimeStampMsec() > timeout)
            {
                ROS_ERROR("Error: Timed out waiting for Node %d to enable\n", iNode);
                return -2;
            }
        }

        // At this point the Node is enabled, and we will now check to see if the Node has been homed
        // Check the Node to see if it has already been homed,
        if (theNode.Motion.Homing.HomingValid())
        {
            if (theNode.Motion.Homing.WasHomed())
            {
                ROS_INFO("Node %d has already been homed, current position is: \t%8.0f \n", iNode,
                         theNode.Motion.PosnMeasured.Value());
                ROS_INFO("Rehoming Node... \n");
            }
            else
            {
                ROS_INFO("Node [%d] has not been homed.  Homing Node now...\n", iNode);
            }
            // Now we will home the Node
            theNode.Motion.Homing.Initiate();

            timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; // define a timeout in case the node is unable to enable
                                                                  //  Basic mode - Poll until disabled
            while (!theNode.Motion.Homing.WasHomed())
            {
                if (myMgr->TimeStampMsec() > timeout)
                {
                    ROS_ERROR(
                        "Node did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t "
                        "-Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n");
                    return -2;
                }
            }
            ROS_INFO("Node completed homing\n");
        }
        else
        {
            printf("Node[%d] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
        }
    }
    if (iNode < 3)
    {
        ROS_ERROR("Missing %d motor connections.", NUM_NODES - iNode + 1);
        return -1;
    }

    FL_Node = &myPort.Nodes(FL_NODE_NUM);
    FR_Node = &myPort.Nodes(FR_NODE_NUM);
    BL_Node = &myPort.Nodes(BL_NODE_NUM);
    BR_Node = &myPort.Nodes(BR_NODE_NUM);

    ros::init(argc, argv, "hank_clearpath_steering");

    ros::NodeHandle nh;
    // angles for each wheel in radians
    ros::Subscriber sub_fl = nh.subscribe("/robot/front_left_position_controller/command", 1, callback_fl);
    ros::Subscriber sub_fr = nh.subscribe("/robot/front_right_position_controller/command", 1, callback_fr);
    ros::Subscriber sub_bl = nh.subscribe("/robot/back_left_position_controller/command", 1, callback_bl);
    ros::Subscriber sub_br = nh.subscribe("/robot/back_right_position_controller/command", 1, callback_br);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}