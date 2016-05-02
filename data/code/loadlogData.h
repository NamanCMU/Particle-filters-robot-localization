#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define total_readings 180

typedef struct 
{
	int data_source; // 0 for Odometry and 1 for Lidar
	float x,y,theta; // pose of robot
	float xl,yl,thetal; // pose of lidar
	float* readings;
	float timestamp;
} logdata;

void loadlogdata(char*, std::vector<logdata>&);
