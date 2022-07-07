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

#define ACC_LIM_RPM_PER_SEC 10000
#define VEL_LIM_RPM 350
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

int32_t rad_to_encoder_counts(double radians) // TODO: Find number of encoder counts
{
    return (int32_t)TOTAL_ENCODER_COUNTS * radians / (2 * PI);
}

void steering_callback(const boost::shared_ptr<std_msgs::Float64 const> command, INode &Node){
    try{
        // ROS_INFO("----- MOVING NODE %zi -----", Node.Info.);
        ROS_INFO("Radians: %f", command->data);
        int32_t target = rad_to_encoder_counts(command->data);
        ROS_INFO("Counts: %d", target);
        Node.Motion.MoveWentDone();              // Clear the rising edge Move done register
        Node.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
        Node.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
        Node.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
        Node.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
        Node.Motion.MovePosnStart(target, true); // Move to absolute position
    }
    catch (mnErr& theErr){
        ROS_ERROR("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
    }
    
}


int main(int argc, char *argv[])
{
    // setup
    size_t portCount = 0;
    std::vector<std::string> comHubPorts;

    SysManager* myMgr = SysManager::Instance(); // Create System Manager myMgr
    
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

    for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
        myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
                                        // with COM portnum (as seen in device manager)
    }
    // myMgr->ComHubPort(SC_HUB_PORT, comHubPorts[SC_HUB_PORT].c_str()); // define the first SC Hub port (port 0) to be associated with COM portnum (as seen in device manager)
    myMgr->PortsOpen(portCount);                                    // Open the port

    IPort &myPort = myMgr->Ports(SC_HUB_PORT);
    ROS_INFO("Port[%d]: state=%d, nodes=%d", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

    // Once the code gets past this point, it can be assumed that the Port has been opened without issue
    // Now we can get a reference to our port object which we will use to access the node objects
    size_t iNode = 0;

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
        ROS_ERROR("Missing %d motor connections.", NUM_NODES - iNode);
        return -1;
    }

    ros::init(argc, argv, "hank_clearpath_steering");
    // size_t FLNodeNum = 2;
    // size_t FRNodeNum = 3;
    // size_t BLNodeNum = 4;
    // size_t BRNodeNum = 5;

    // INode &FL_Node = myPort.Nodes(FLNodeNum);
    // INode &FR_Node = myPort.Nodes(FRNodeNum);
    // INode &BL_Node = myPort.Nodes(BLNodeNum);
    // INode &BR_Node = myPort.Nodes(BRNodeNum);


    INode &FL_Node = myPort.Nodes(FL_NODE_NUM);
    INode &FR_Node = myPort.Nodes(FR_NODE_NUM);
    INode &BL_Node = myPort.Nodes(BL_NODE_NUM);
    INode &BR_Node = myPort.Nodes(BR_NODE_NUM);

    

    ros::NodeHandle nh;
    // angles for each wheel in radians
    ros::Subscriber sub_fl = nh.subscribe<std_msgs::Float64>("/robot/front_left_position_controller/command", 1, boost::bind(&steering_callback, _1, boost::ref(FL_Node)));
    ros::Subscriber sub_fr = nh.subscribe<std_msgs::Float64>("/robot/front_right_position_controller/command", 1, boost::bind(&steering_callback, _1,boost::ref(FR_Node)));
    ros::Subscriber sub_bl = nh.subscribe<std_msgs::Float64>("/robot/back_left_position_controller/command", 1, boost::bind(&steering_callback, _1, boost::ref(BL_Node)));
    ros::Subscriber sub_br = nh.subscribe<std_msgs::Float64>("/robot/back_right_position_controller/command", 1, boost::bind(&steering_callback, _1, boost::ref(BR_Node)));

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}


// void callback_fl(const std_msgs::Float64 &fl_command)
// {
//     int32_t target = rad_to_encoder_counts(fl_command.data);
//     FL_Node.Motion.MoveWentDone();              // Clear the rising edge Move done register
//     FL_Node.Motion.MovePosnStart(target, true); // Move to absolute position
//     printf("Moving Node \t%zi \n", FL_Node);
//     // printf("%f estimated time.\n", FL_Node->Motion.MovePosnDurationMsec(target));
//     // double timeout = myMgr->TimeStampMsec() + FL_Node->Motion.MovePosnDurationMsec(target) + 100; // define a timeout in case the node is unable to enable

//     // while (!theNode.Motion.MoveIsDone())
//     // {
//     //     if (myMgr->TimeStampMsec() > timeout)
//     //     {
//     //         ROS_ERROR("Error: Timed out waiting for move to complete\n");
//     //         break;
//     //     }
//     // }
//     // printf("Node \t%zi Move Done\n", *FL_Node);
// }

// void callback_fr(const std_msgs::Float64 &fr_command)
// {
//     int32_t target = rad_to_encoder_counts(fr_command.data);
//     FR_Node.Motion.MoveWentDone();              // Clear the rising edge Move done register
//     FR_Node.Motion.MovePosnStart(target, true); // Move to absolute position
//     printf("Moving Node \t%zi \n", FR_Node);
// }

// void callback_bl(const std_msgs::Float64 &bl_command)
// {
//     int32_t target = rad_to_encoder_counts(bl_command.data);
//     BL_Node.Motion.MoveWentDone();              // Clear the rising edge Move done register
//     BL_Node.Motion.MovePosnStart(target, true); // Move to absolute position
//     printf("Moving Node \t%zi \n", BL_Node);
// }

// void callback_br(const std_msgs::Float64 &br_command)
// {
//     int32_t target = rad_to_encoder_counts(br_command.data);
//     BR_Node.Motion.MoveWentDone();              // Clear the rising edge Move done register
//     BR_Node.Motion.MovePosnStart(target, true); // Move to absolute position
//     printf("Moving Node \t%zi \n", BR_Node);
// }