#include <ros/ros.h> 
#include <visualization_msgs/Marker.h> // need this for publishing markers
#include <geometry_msgs/Point.h> //data type used for markers
#include <string.h>
#include <stdio.h>  
#include <rom_rviz/SimpleFloatSrvMsg.h> //a custom message type defined in this package
using namespace std;
int break_count = 30;

void init_marker_vals(visualization_msgs::Marker &marker) {
    marker.header.frame_id = "map"; // reference frame for marker coords
    marker.header.stamp = ros::Time();
    marker.ns = "ROM ROBOTICS";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    // use SPHERE_LIST for a group of markers
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y,   				    
    //        marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below

    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rom_rviz");
    ros::NodeHandle nh;
    ros::Publisher r_pub = nh.advertise<visualization_msgs::Marker>("red", 0);
    ros::Publisher g_pub = nh.advertise<visualization_msgs::Marker>("green", 0);
    ros::Publisher b_pub = nh.advertise<visualization_msgs::Marker>("blue", 0);
    ros::Publisher t_pub = nh.advertise<visualization_msgs::Marker>("text1", 0);
    ros::Publisher t_pub2 = nh.advertise<visualization_msgs::Marker>("text2", 0);
    visualization_msgs::Marker r_marker; // instantiate a marker object
    visualization_msgs::Marker g_marker;
    visualization_msgs::Marker b_marker;
    double iteration = 0.0;
    ros::param::get("/rom_marker/brakecount", break_count);

    visualization_msgs::Marker rom_robotics;
    rom_robotics.header.frame_id = "map";
    rom_robotics.header.stamp = ros::Time::now();
    rom_robotics.ns = "basic_shapes";
    rom_robotics.id = iteration;
    rom_robotics.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    rom_robotics.action = visualization_msgs::Marker::ADD;

    rom_robotics.pose.position.x = 3.5;
    rom_robotics.pose.position.y = 0.0;
    rom_robotics.pose.position.z = 8.5;
    rom_robotics.pose.orientation.x = 0.0;
    rom_robotics.pose.orientation.y = 0.0;
    rom_robotics.pose.orientation.z = 0.0;
    rom_robotics.pose.orientation.w = 1.0;
    rom_robotics.text = "ROM Robotics";
    rom_robotics.scale.x = 1.0;
    rom_robotics.scale.y = 1.0;
    rom_robotics.scale.z = 1.0;
    
    rom_robotics.color.r = 1.0f;
    rom_robotics.color.g = 1.0f;
    rom_robotics.color.b = 0.0f;
    rom_robotics.color.a = 1.0;
    
    visualization_msgs::Marker osrfm;
    osrfm.header.frame_id = "map";
    osrfm.header.stamp = ros::Time::now();
    osrfm.ns = "basic_shapes";
    osrfm.id = iteration;
    osrfm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    osrfm.action = visualization_msgs::Marker::ADD;

    osrfm.pose.position.x = 3.5;
    osrfm.pose.position.y = 0.0;
    osrfm.pose.position.z = 8;
    osrfm.pose.orientation.x = 0.0;
    osrfm.pose.orientation.y = 0.0;
    osrfm.pose.orientation.z = 0.0;
    osrfm.pose.orientation.w = 1.0;
    osrfm.text = "Open Source Robotics For Myanmar";
    osrfm.scale.x = 0.4;
    osrfm.scale.y = 0.4;
    osrfm.scale.z = 0.4;
    osrfm.color.r = 1.0f;
    osrfm.color.g = 0.0f;
    osrfm.color.b = 0.0f;
    osrfm.color.a = 1.0;

    geometry_msgs::Point point; // points will be used to specify where the markers go
    
    init_marker_vals(r_marker);
    g_marker = r_marker;
    b_marker = r_marker;

    r_marker.color.r = 1.0; // red
    g_marker.color.g = 1.0; // green
    b_marker.color.r = 1.0; // red+green=yellow
    b_marker.color.g = 1.0;
   
    //double z_des;

    // build a wall of markers; set range and resolution
    double x_min = 7.0;
    double x_max = 9.0;
    double y_min = 0.0;
    double y_max = 2.0;
    double density = 0.035;
    double z_min = 8.0;
    double z_max = 10.0;

    
    while (ros::ok()) 
    {
            ros::Time current_time= ros::Time::now();
    	    r_marker.header.stamp = current_time;
    	    g_marker.header.stamp = current_time;
    	    b_marker.header.stamp = current_time;
            r_marker.points.clear(); g_marker.points.clear(); b_marker.points.clear();
            
                for(double x=7.0;x<x_max;x+=density){
	        		for(double y=y_min;y<y_max;y+=density){
	        			for(double z=z_min;z<(z_min+0.66);z+=density){
	        				point.x = x;
	        				point.y = y;
	        				point.z = z;
	        				r_marker.points.push_back(point);
	        			}
	        		}
	        	}  
                for(double x=7.0;x<x_max;x+=density){
                    for(double y=y_min;y<y_max;y+=density){
                        for(double z=z_min+0.66;z<(z_min+1.33);z+=density){
                            point.x = x;
                            point.y = y;
                            point.z = z;
                            g_marker.points.push_back(point);
                        }
                    }
                }
                for(double x=7.0;x<x_max;x+=density){
                    for(double y=y_min;y<y_max;y+=density){
                        for(double z=z_min+1.33;z<z_max;z+=density){
                            point.x = x;
                            point.y = y;
                            point.z = z;
                            b_marker.points.push_back(point);
                        }
                    }
                }  
        
        ros::Duration(0.1).sleep();
        //ROS_INFO("publishing...");
        r_pub.publish(r_marker);
        g_pub.publish(g_marker);
        b_pub.publish(b_marker);
        t_pub.publish(rom_robotics);
        t_pub2.publish(osrfm);
        ros::spinOnce(); 
        break_count -= 1;
        if(break_count < 0) { break; }
    }
    ROS_INFO("rom robotics marker node will end..");
    ros::shutdown();
    return 0;
}



