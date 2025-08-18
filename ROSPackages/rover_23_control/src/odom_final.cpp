// odometry for steering 3.1.1
//Halil Faruk Karagöz

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <signal.h>
#include <cmath>
#include <valarray>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"


using namespace std;
#define d1vector valarray<double> 
#define d2vector valarray<valarray<double>>


void mySigintHandler(int sig) {ros::shutdown();}

class odometry
{
private:
    ros::NodeHandle nh;
    ros::Subscriber imuSub,encoderSub;
    double encoder_msg[4];
    ros :: Publisher odomPub;
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped tf;
    tf::TransformBroadcaster tf_br;
    geometry_msgs::Quaternion quat;
    double Pi = M_PI,length=.824,width=.914;
    float wheel_speed[4], steer_angle[4];
    double dt = 0;
    double x,y,z;
    double x_speed,y_speed,z_speed;
    double angular_z = 0;
    double yaw,yaw_prev,yaw_change,yaw_start=0;
    bool first_yaw=1;
    ros::Time current_time;
    d1vector Vmean={0,0};
    d2vector Vn,wheel_locations;

public:
    odometry(int argc, char *argv[]){
        ros::Rate rate(200); // 200
        //imuSub = nh.subscribe("/imu1/data",10,&odometry::imu_cb,this);
        
        imuSub = nh.subscribe("/imu1/data",10,&odometry::imu_new_cb,this);
        encoderSub = nh.subscribe("/drive_system/encoder/steer_recieved",10, &odometry::encoder_cb, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("drive_system/odometry",10);
        cout<< "Odometry Started!"<<endl;
        x=0,y=0,z=0;
        x_speed = 0,y_speed = 0,z_speed = 0, angular_z = 0;
        current_time = ros::Time::now();
        init_params();
        yaw_prev = yaw;
        double dist_btw_wheels = 0.85;
        cout <<endl;
        while (ros::ok)
        {
            dt = (ros::Time::now() - current_time).toSec();
            current_time = ros::Time::now();

            double right_wheel = encoder_msg[2];
            double left_wheel = encoder_msg[0];           

            x_speed = (right_wheel + left_wheel) / 2;

            angular_z = (right_wheel- left_wheel) / dist_btw_wheels;

            if(angular_z < 0){
                yaw_change = abs(angular_z) * dt * -1;
            }
            else{
                yaw_change = abs(angular_z) * dt;
            }

            yaw += yaw_change;

            
            // cout<<speed<<" "<< angular_z << endl;
            // cout<<"radius: "<< radius << endl;

            
            quat = tf::createQuaternionMsgFromYaw(yaw);
            //publish odom
            publish_odom();
            //boradcast tf
            //broadcast_tf();
            std::cout<<x  << " " << y << std::endl;
	        //cout<< "the yaw is : " << this->yaw<<endl;
            ros::spinOnce();
            rate.sleep();
        }
    }

    
    void broadcast_tf(){ // publishing odom msg
        tf.header.stamp = current_time;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = quat;

        tf_br.sendTransform(tf);
    }

    void publish_odom (){ // broadcasting tf
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = quat;
        
        odom.twist.twist.linear.x = x_speed;
        odom.twist.twist.linear.y = y_speed;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = angular_z;

        odomPub.publish(odom);
    }

    double find_l2_norm(double x ,double y ){
        return sqrt(x*x+y*y);
    }
    
    void find_global_speeds(double x,double y){
        double relx = x;
        double rely = y;
        //double norm = find_l2_norm(relx,rely);
        //cout <<  cos(yaw) * rely << " " <<  sin(yaw) * rely << endl;
        x_speed = (cos(yaw) * relx  - sin(yaw) * rely) ; // constant may change 
        y_speed = (sin(yaw) * relx + cos(yaw) * rely) ; // constant may change 
    }


    void init_params(){
        Vn   = createMatrix(4,2) ;
        wheel_locations  = createMatrix(4,2) ;
        wheel_locations[0] = {length/ 2.,width/ 2.};
        wheel_locations[1] = {length/-2.,width/ 2.};
        wheel_locations[2] = {length/-2.,width/-2.};
        wheel_locations[3] = {length/ 2.,width/-2.};
    }


    void encoder_cb(const std_msgs::Float64MultiArray& msg){
        for (int i = 0; i < 4; i++){
            encoder_msg[i] = msg.data[i];     
        }
    }

    void imu_cb(const sensor_msgs::Imu& msg){
        tf::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, this->yaw);
        //this->yaw += M_PI_2;
	if (this->yaw<10 && first_yaw )
    {
	    this->yaw_start=this->yaw;
	    first_yaw=0;
    }
	this->yaw -= this->yaw_start;
    }
    void imu_new_cb(const sensor_msgs::Imu& msg){
        angular_z = msg.angular_velocity.z;
        //this->yaw += M_PI_2;
        if (dt != 0 ){
            this->yaw += angular_z*dt;
        }
    }
    

    d2vector createMatrix(int n0,int n1){
        d2vector arr(n0);
        for(int k=0;k<arr.size();k++)arr[k].resize(n1);
        return arr;}

    void Disp4_2Mat(d2vector arr,char text='*'){ 
        cout<<"//////////////////////////////////////////////////"<<endl;
        cout<<"+++++++++++++++++++"<<text<<"+++++++++++++++++++++"<<endl;
        cout<<"x 0----- "<< arr[0][0]<<"  y------ "<<arr[0][1]<<endl;
        cout<<"x 1----- "<< arr[1][0]<<"  y------ "<<arr[1][1]<<endl;
        cout<<"x 2----- "<< arr[2][0]<<"  y------ "<<arr[2][1]<<endl;
        cout<<"x 3----- "<< arr[3][0]<<"  y------ "<<arr[3][1]<<endl;
        cout<<"---------------------------------------------------"<<endl;
    }
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "Steering_odometry_new",ros::init_options::NoSigintHandler);
    odometry han_odometry(argc,argv);
    signal(1, mySigintHandler);

    return 0;
}