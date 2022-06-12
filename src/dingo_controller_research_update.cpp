#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <time.h>
#include <sstream>

/**
 * This node takes in a twist message and outputs 8 motor commands for a 4 wheel steering motor
 */

 class DingoController{
    public:
        ros::Publisher command_front_left_velocity;
        ros::Publisher command_front_right_velocity;
        ros::Publisher command_back_left_velocity;
        ros::Publisher command_back_right_velocity;
        ros::Publisher command_front_left_position;
        ros::Publisher command_front_right_position;
        ros::Publisher command_back_left_position;
        ros::Publisher command_back_right_position;

        std_msgs::String status;

        void DingoControllerCallback(const geometry_msgs::Twist::ConstPtr& cmd);
 };


 /** This function is entered everytime a new message gets recieved */
void DingoController::DingoControllerCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	bool simulation;

	ros::param::get("/simulation", simulation);

    std_msgs::Float64 scale_factor;  //defines the scale. if the scale is 1, the commands are scaled to move the base at a speed of 1. This needs to be increased here to edit the speed in simulation
	scale_factor.data = 1.5; //was 2.5 which is roughly .67 m/s
	if(simulation == true)
	{
		scale_factor.data = 7.95; //7.95 scales so a twist of 1 outputs 1m/s in the simulation   
	}

    std_msgs::Float64 velx;
    std_msgs::Float64 vely;
    std_msgs::Float64 velz;
    std_msgs::Float64 omega;
    std_msgs::Float64 omega_y;
    std_msgs::Float64 omega_z;

    std_msgs::Float64 vel; //used when all wheel speeds and positions are the same
    std_msgs::Float64 pos;

    std_msgs::Float64 vel_fl; //used when wheel speeds and positions are unique
    std_msgs::Float64 vel_fr;
    std_msgs::Float64 vel_bl;
    std_msgs::Float64 vel_br;
    std_msgs::Float64 pos_fl;
    std_msgs::Float64 pos_fr;
    std_msgs::Float64 pos_bl;
    std_msgs::Float64 pos_br;

    std_msgs::Float64 ICR_x;
    std_msgs::Float64 ICR_y;
    std_msgs::Float64 ICR_dist;
    std_msgs::Float64 zero;
    std_msgs::Float64 negative_vel;
    std_msgs::Float64 negative_pos;
    zero.data = 0;

    geometry_msgs::Twist new_cmd = *cmd; //creates a new variable from the value in the pointer

    velx.data = new_cmd.linear.x;  //command for wheel power in Ackerman driving
    vely.data = new_cmd.linear.y;
    velz.data = new_cmd.linear.z;  //command for strafing as well
    omega.data = new_cmd.angular.x;  //command angular velocity turning
    omega_y.data = new_cmd.angular.y; //command for turn in place
    omega_z.data = new_cmd.angular.z; //command for strafing

    if(sqrt(velx.data*velx.data + vely.data*vely.data)<1.5) //limits wheel velocity to a value of 1 (when there was velx and vely the velocity could be sqrt(2))
    {
        vel.data = scale_factor.data*sqrt(velx.data*velx.data + vely.data*vely.data); //command velocity vector
    }
    else
    {
        vel.data = scale_factor.data*1.5;
    }

    if(vel.data>.05 && abs(omega.data)<.05) //if not turning
    {
        pos.data = atan2(vely.data,velx.data);
        negative_vel.data = -vel.data;
        if(velx.data<0)
        {
            command_front_left_velocity.publish(negative_vel);
            command_front_right_velocity.publish(vel);
            command_back_left_velocity.publish(negative_vel);
            command_back_right_velocity.publish(vel);
            // command_front_left_position.publish(pos);
            // command_front_right_position.publish(pos);
            // command_back_left_position.publish(pos);
            // command_back_right_position.publish(pos);
            status.data = "Driving forward not turning";
        }
        if(velx.data>=0)
        {
            if(pos.data>0)
            {
                negative_pos.data = (pos.data-M_PI);
            }
            if(pos.data<0)
            {
                negative_pos.data = (pos.data+M_PI);
            }
            command_front_left_velocity.publish(vel);
            command_front_right_velocity.publish(negative_vel);
            command_back_left_velocity.publish(vel);
            command_back_right_velocity.publish(negative_vel);
            // command_front_left_position.publish(negative_pos);
            // command_front_right_position.publish(negative_pos);
            // command_back_left_position.publish(negative_pos);
            // command_back_right_position.publish(negative_pos);
            status.data = "Driving backwards not turning";
        }
    }

    else if(vel.data<.05 && abs(omega.data)>.05) //if turning, not driving
    {
        ICR_y.data = .631/2 + 1.3 - abs(omega.data); //distance of the ICR from the center of the robot
        //ICR_x.data defined in the if statements



        if(omega.data>0) //if turning left
        {
            status.data = "Turning left";
            ICR_dist.data = sqrt(ICR_x.data*ICR_x.data + ICR_y.data*ICR_y.data); //absolute value of distance to ICR

            pos_fl.data = atan2(.724/2-ICR_x.data,ICR_y.data-.631/2);
            pos_fr.data = atan2(.724/2-ICR_x.data,ICR_y.data+.631/2);
            pos_bl.data = -atan2(.724/2+ICR_x.data,ICR_y.data-.631/2);
            pos_br.data = -atan2(.724/2+ICR_x.data,ICR_y.data+.631/2);
        }

        if(omega.data<0) //if turning right
        {
            status.data = "Turning right";
            ICR_dist.data = sqrt(ICR_x.data*ICR_x.data + ICR_y.data*ICR_y.data); //absolute value of distance to ICR

            pos_fl.data = -atan2(.724/2-ICR_x.data,ICR_y.data+.631/2);
            pos_fr.data = -atan2(.724/2-ICR_x.data,ICR_y.data-.631/2);
            pos_bl.data =  atan2(.724/2+ICR_x.data,ICR_y.data+.631/2);
            pos_br.data =  atan2(.724/2+ICR_x.data,ICR_y.data-.631/2);
        }

        command_front_left_velocity.publish(zero);
        command_front_right_velocity.publish(zero);
        command_back_left_velocity.publish(zero);
        command_back_right_velocity.publish(zero);
        command_front_left_position.publish(pos_fl);
        command_front_right_position.publish(pos_fr);
        command_back_left_position.publish(pos_bl);
        command_back_right_position.publish(pos_br);
    }
    else if(vel.data>.05 && abs(omega.data)>.05) //if driving and turning
    {
        ICR_y.data = .631/2 + 1.3 - abs(omega.data); //distance of the ICR from the center of the robot
        //ICR_x.data defined in the if statements
        if(velx.data >0) //allows turning while driving backwards
        {
            vel.data = -vel.data;
        }
        if(omega.data>0) //if turning left
        {
            if(velx.data>0)
            {
                ICR_x.data =  -vely.data;
                status.data = "Driving forward + turning left";
            }
            if(velx.data<0)
            {
                ICR_x.data =  vely.data;
                status.data = "Driving backwards + turning left";
            }
            ICR_dist.data = sqrt(ICR_x.data*ICR_x.data + ICR_y.data*ICR_y.data); //absolute value of distance to ICR

            pos_fl.data = atan2(.724/2-ICR_x.data,ICR_y.data-.631/2);
            pos_fr.data = atan2(.724/2-ICR_x.data,ICR_y.data+.631/2);
            pos_bl.data = -atan2(.724/2+ICR_x.data,ICR_y.data-.631/2);
            pos_br.data = -atan2(.724/2+ICR_x.data,ICR_y.data+.631/2);
            vel_fl.data = -(vel.data*(sqrt((.724/2-ICR_x.data)*(.724/2-ICR_x.data) + (ICR_y.data-.631/2)*(ICR_y.data-.631/2)))/(ICR_dist.data)); //scales veloctity by relative distance to the ICR. (closer to ICR will have smaller velocty)
            vel_fr.data = vel.data*(sqrt((.724/2-ICR_x.data)*(.724/2-ICR_x.data) + (ICR_y.data+.631/2)*(ICR_y.data+.631/2)))/(ICR_dist.data);
            vel_bl.data = -(vel.data*(sqrt((.724/2+ICR_x.data)*(.724/2+ICR_x.data) + (ICR_y.data-.631/2)*(ICR_y.data-.631/2)))/(ICR_dist.data));
            vel_br.data = vel.data*(sqrt((.724/2+ICR_x.data)*(.724/2+ICR_x.data) + (ICR_y.data+.631/2)*(ICR_y.data+.631/2)))/(ICR_dist.data);
        }

        if(omega.data<0) //if turning right
        {
            if(velx.data>0)
            {
                ICR_x.data =  vely.data;
                status.data = "Driving forward + turning right";
            }
            if(velx.data<0)
            {
                ICR_x.data =  -vely.data;
                status.data = "Driving backwards + turning right";
            }
            ICR_dist.data = sqrt(ICR_x.data*ICR_x.data + ICR_y.data*ICR_y.data); //absolute value of distance to ICR

            pos_fl.data = -atan2(.724/2-ICR_x.data,ICR_y.data+.631/2);
            pos_fr.data = -atan2(.724/2-ICR_x.data,ICR_y.data-.631/2);
            pos_bl.data =  atan2(.724/2+ICR_x.data,ICR_y.data+.631/2);
            pos_br.data =  atan2(.724/2+ICR_x.data,ICR_y.data-.631/2);
            vel_fl.data = -(vel.data*(sqrt((.724/2-ICR_x.data)*(.724/2-ICR_x.data) + (ICR_y.data+.631/2)*(ICR_y.data+.631/2)))/(ICR_dist.data)); //scales veloctity by relative distance to the ICR. (closer to ICR will have smaller velocty)
            vel_fr.data = vel.data*(sqrt((.724/2-ICR_x.data)*(.724/2-ICR_x.data) + (ICR_y.data-.631/2)*(ICR_y.data-.631/2)))/(ICR_dist.data);
            vel_bl.data = -(vel.data*(sqrt((.724/2+ICR_x.data)*(.724/2+ICR_x.data) + (ICR_y.data+.631/2)*(ICR_y.data+.631/2)))/(ICR_dist.data));
            vel_br.data = vel.data*(sqrt((.724/2+ICR_x.data)*(.724/2+ICR_x.data) + (ICR_y.data-.631/2)*(ICR_y.data-.631/2)))/(ICR_dist.data);
        }


        command_front_left_velocity.publish(vel_fl);
        command_front_right_velocity.publish(vel_fr);
        command_back_left_velocity.publish(vel_bl);
        command_back_right_velocity.publish(vel_br);
        command_front_left_position.publish(pos_fl);
        command_front_right_position.publish(pos_fr);
        command_back_left_position.publish(pos_bl);
        command_back_right_position.publish(pos_br);
    }

    else if(abs(omega_y.data)>.05) //turn in place. Pressing left right triggers will turn wheels to position, then forward on left stick will give power to the wheels
    {
        vel_fl.data = -scale_factor.data*omega_y.data;
        vel_fr.data = -scale_factor.data*omega_y.data;
        vel_bl.data = -scale_factor.data*omega_y.data;
        vel_br.data = -scale_factor.data*omega_y.data;
        pos_fl.data = -.854; //from geomtery (angle tangent to center point)
        pos_fr.data = .717;
        pos_bl.data = .717;
        pos_br.data = -.854;


        command_front_left_position.publish(pos_fl);
        command_front_right_position.publish(pos_fr);
        command_back_left_position.publish(pos_bl);
        command_back_right_position.publish(pos_br);
        command_front_left_velocity.publish(vel_fl);
        command_front_right_velocity.publish(vel_fr);
        command_back_left_velocity.publish(vel_bl);
        command_back_right_velocity.publish(vel_br);
        status.data = "Turning in place: with power to wheels";
    
     

    }

    else if(abs(omega_z.data)>.05 || abs(velz.data)>0.05) //strafe left/right. Pressing left/right on directional pad will turn wheels to position, then forward on left stick will give power to the wheels
    {
    
        std_msgs::Float64 turn_in_place_command;
        std_msgs::Float64 neg_turn_in_place_command;
        turn_in_place_command.data = 1;
        neg_turn_in_place_command.data = -1;



        if(velz.data >= 0) //if forward/back on left stick
        {
            command_front_left_velocity.publish(neg_turn_in_place_command);
            command_front_right_velocity.publish(turn_in_place_command);
            command_back_left_velocity.publish(neg_turn_in_place_command);
            command_back_right_velocity.publish(turn_in_place_command);

            //pos.data = omega_z.data*M_PI/2; //turns wheel to +- pi/2 (note omega_y is either +-1)
            pos.data = atan2(omega_z.data, velz.data); //*M_PI/2; //turns wheel to +- pi/2 (note omega_y is either +-1)
            //try ratio of two joysticks too, that might help


            command_front_left_position.publish(pos);  //turn wheels to face left
            command_front_right_position.publish(pos);
            command_back_left_position.publish(pos);
            command_back_right_position.publish(pos);
            status.data = "Strafe forwards";
        }
        else
        {
            command_front_left_velocity.publish(turn_in_place_command);
            command_front_right_velocity.publish(neg_turn_in_place_command);
            command_back_left_velocity.publish(turn_in_place_command);
            command_back_right_velocity.publish(neg_turn_in_place_command);

            //pos.data = -omega_z.data*M_PI/2; //turns wheel to +- pi/2 (note omega_y is either +-1)
            pos.data = atan2(-omega_z.data, -velz.data); //*M_PI/2; //turns wheel to +- pi/2 (note omega_y is either +-1)

            command_front_left_position.publish(pos);  //turn wheels to face left
            command_front_right_position.publish(pos);
            command_back_left_position.publish(pos);
            command_back_right_position.publish(pos);
            status.data = "Strafe backwards";
        }

    }

    else  //dont move
    {
        command_front_left_velocity.publish(zero);
        command_front_right_velocity.publish(zero);
        command_back_left_velocity.publish(zero);
        command_back_right_velocity.publish(zero);
        command_front_left_position.publish(zero);
        command_front_right_position.publish(zero);
        command_back_left_position.publish(zero);
        command_back_right_position.publish(zero);
        status.data = "Not driving";
    }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "commands");

  ros::NodeHandle n;

  DingoController dingo_controller_instance;

  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &DingoController::DingoControllerCallback, &dingo_controller_instance);
  ros::Publisher controller_status = n.advertise<std_msgs::String>("controller_status", 1);

  dingo_controller_instance.command_front_left_velocity = n.advertise<std_msgs::Float64>("/robot/front_left_velocity_controller/command", 1000);
  dingo_controller_instance.command_front_right_velocity = n.advertise<std_msgs::Float64>("/robot/front_right_velocity_controller/command", 1000);
  dingo_controller_instance.command_back_left_velocity = n.advertise<std_msgs::Float64>("/robot/back_left_velocity_controller/command", 1000);
  dingo_controller_instance.command_back_right_velocity = n.advertise<std_msgs::Float64>("/robot/back_right_velocity_controller/command", 1000);
  
  dingo_controller_instance.command_front_left_position = n.advertise<std_msgs::Float64>("/robot/front_left_position_controller/command", 1000);
  dingo_controller_instance.command_front_right_position = n.advertise<std_msgs::Float64>("/robot/front_right_position_controller/command", 1000);
  dingo_controller_instance.command_back_left_position = n.advertise<std_msgs::Float64>("/robot/back_left_position_controller/command", 1000);
  dingo_controller_instance.command_back_right_position = n.advertise<std_msgs::Float64>("/robot/back_right_position_controller/command", 1000);


  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    //publishes commans at the frequency defined in loop_rate()
    controller_status.publish(dingo_controller_instance.status);

    loop_rate.sleep();
  }


  return 0;
}