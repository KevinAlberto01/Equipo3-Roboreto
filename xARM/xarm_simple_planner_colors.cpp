/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <stdlib.h>
#include <vector>
#include <geometry_msgs/Point.h>

geometry_msgs::Pose objeto;

bool request_plan(ros::ServiceClient& client, xarm_planner::joint_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service joint_plan");
		return false;
	}
}

bool request_plan(ros::ServiceClient& client, xarm_planner::pose_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service pose_plan");
		return false;
	}
}

bool request_plan(ros::ServiceClient& client, xarm_planner::single_straight_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service single_straight_plan");
		return false;
	}
}

bool request_exec(ros::ServiceClient& client, xarm_planner::exec_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service exec_plan");
		return false;
	}
}

// Definir callback de las coordenadas recibidas por el nodo de detección de objetos

void trackerCallback(const geometry_msgs::Point &msg)
{	
	//Definir valores mínimos de X y Z. Y se ajusta para que corresponda con las coordenadas de la base
	objeto.position.x = msg.x +0.18;
    objeto.position.y = msg.y -0.04;
	objeto.position.z = msg.z +0.2;

	objeto.orientation.x = 1;
	objeto.orientation.y = 0;
	objeto.orientation.z = 0;
	objeto.orientation.w = 0;	
	
	//Ajustar el rango de valores máximos y mínimos de las coordenadas (Asegurar que no haya colisiones)
	if (objeto.position.x < 0.18){
		objeto.position.x = 0.18;
	}
	if (objeto.position.x > 0.41){
		objeto.position.x = 0.41;
	}
	if (objeto.position.y < -0.23){
		objeto.position.y = -0.24;
	}
	if (objeto.position.y > 0.15){
		objeto.position.y = 0.15;
	}
	if (objeto.position.z > 0.5){
		objeto.position.z = 0.5;
	}
	if (objeto.position.z < 0.25){
		objeto.position.z = 0.25;
	}

	
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;

	// Declarar los clientes de planning y ejecución del xARM
	ros::ServiceClient client_exec = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
	ros::Subscriber objPos = nh.subscribe("object_position", 1000, trackerCallback);
	ros::Publisher exec_pub = nh.advertise<std_msgs::Bool>("xarm_planner_exec", 10);
	std_msgs::Bool msg;
	xarm_planner::exec_plan srv_exec;

	//********************************************************************
	// Definir el movimiento del planning como una línea recta (asegurar movimientos seguros)
	ros::ServiceClient client22 = nh.serviceClient<xarm_planner::pose_plan>("xarm_straight_plan");

	xarm_planner::single_straight_plan srv22;
	xarm_planner::pose_plan srv3;

	double slp_t = 0.5;
	double u;

	// Definir una posición inicial
	geometry_msgs::Pose initial_target;
	initial_target.position.x = 0.2;
	initial_target.position.y = 0.0;
	initial_target.position.z = 0.25;

	initial_target.orientation.x = 1;
	initial_target.orientation.y = 0;
	initial_target.orientation.z = 0;
	initial_target.orientation.w = 0;

	//Declarar la posición inicial del objeto como la posición inicial del xARM
	objeto.position.x = 0.2;
	objeto.position.y = 0.0;
	objeto.position.z = 0.25;

	objeto.orientation.x = 1;
	objeto.orientation.y = 0;
	objeto.orientation.z = 0;
	objeto.orientation.w = 0;

	//Ejectuar el planning y mover el xARM a la posición inicial
	srv22.request.target = initial_target;
	if(request_plan(client22, srv22))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		// msg.data = true;
		// ros::Duration(1.0).sleep();
		// exec_pub.publish(msg);
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}
	ros::Duration(slp_t).sleep(); // Wait for last execution to finish

	//Actualizar la posición actual al objetivo inicial
	geometry_msgs::Pose currentPose = initial_target;
	while(ros::ok()){
		
		/*Cuando la pose del objeto cambia, se realiza el planning de las nuevas coordendas del objeto
		  En caso del que el planning sea exitoso, se ejecuta el movimiento y se actualizan las coordenadas
		  del objeto*/
		if (objeto != currentPose)
		{
			srv22.request.target = objeto;
			if(request_plan(client22, srv22))
			{
			ROS_INFO("Plan SUCCESS! Executing... ");
			// 	// msg.data = true;
			// 	// ros::Duration(1.0).sleep();
			// 	// exec_pub.publish(msg);
			ROS_INFO("%f", objeto.position.x);
			ROS_INFO("%f", objeto.position.y);
			ROS_INFO("%f", objeto.position.z);
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
			currentPose = objeto;
			}
		}
		ros::Duration(slp_t).sleep(); // Wait for last execution to finish	}
		
		ros::spinOnce();
	}

	return 0;

}

