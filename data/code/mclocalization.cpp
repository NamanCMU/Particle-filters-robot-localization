#include "mclocalization.h"


mclocalization::mclocalization()
{
	// alphas
	_alpha1 = 0.01;
	_alpha2 = 0.01;
	_alpha3 = 0.1;
	_alpha4 = 0.1;
	_alpha_slow = 0.001;
	_alpha_fast = 1.0;
	
	// Measurement model parameters
	_sigmahit = 4;
	_lambdashort = 1.5;
	_zHit = 0.3;
	_zRandom = 0.3;
	_zMax = 0.2;
	_zshort = 0.2;

	// Miscellaneous
	_numParticles = 3000;
	_threshold_particles = 0.9;
	_lidar_offset = 25; // In cm
	_ray_tracing_step = 1;
	_resolution = 10;
	_max_range = 800;
	_threshold = 0.7;
}

mclocalization::~mclocalization()
{

}

// Initialize and load the map
void mclocalization::initialize_map(map_type* map)
{
	_map = map;
	_image_map = Mat::zeros(_map->size_x, _map->size_y, CV_32FC1);
	_image_robot = Mat::zeros(_map->size_x, _map->size_y, CV_32FC1);

	for(int i = 0;i < _image_map.rows;i++)
	{
		for(int j = 0;j < _image_map.cols;j++)
		{
			if(_map->prob[i][j] > 0.0){
				_image_map.at<float>(i,j) = _map->prob[i][j];
				_image_robot.at<float>(i,j) = _map->prob[i][j];
			}
		}
	}
}

// Initialize logfile data
void mclocalization::initialize_logdata(std::vector<logdata>& logfiledata)
{
	_logfiledata = logfiledata;
}

// Initialize random particles to begin with
void mclocalization::initialize_particles()
{

	int count = 1;
	while (count <= _numParticles)
	{
		particles pTemp;
		pTemp.x =  rand() / (float)RAND_MAX * (_map->max_x - _map->min_x) + _map->min_x; // Random x position
		pTemp.y = rand() / (float)RAND_MAX * (_map->max_y - _map->min_y) + _map->min_y; // Random y position

		if (_map->prob[(int) pTemp.x][(int) pTemp.y] <= _threshold_particles)
			continue;

		count++;

		pTemp.theta = rand() / (float)RAND_MAX * 2 * pi; // Theta
		pTemp.weight = 1.0/_numParticles; // Uniform weights

		particles_pf.push_back(pTemp);
	}

}

// Visualize Particles
void mclocalization::visualize()
{
	Mat Image;
	cvtColor(_image_map,Image,CV_GRAY2RGB);

	for(int i = 0;i < _numParticles;i++){
		circle(Image,Point(particles_pf[i].y,particles_pf[i].x),3,Scalar(0,0,255),-1);
	}

	namedWindow("mapImage",WINDOW_AUTOSIZE);
	imshow("mapImage",Image);

	waitKey(10);
}

// Visualize log data and the robot
void mclocalization::visualize_log_robot()
{
	Mat Image;
	cvtColor(_image_robot,Image,CV_GRAY2RGB);

	circle(Image,Point(robot_position.y,robot_position.x),8,Scalar(255,0,0),-1); // Robot position

	// Position of the lidar
	float laser_x = robot_position.x + (_lidar_offset/_resolution)*cos(robot_position.theta); 
	float laser_y = robot_position.y + (_lidar_offset/_resolution)*sin(robot_position.theta);

	float laser_end_x,laser_end_y;
	for (int i = 0;i < total_readings; i++)
	{
		
		float thetastep = (float)i * (pi /180) + robot_position.theta;
		laser_end_x = laser_x + mmm_measure.readings[i]*cos(thetastep - pi/2.0);
		laser_end_y = laser_y + mmm_measure.readings[i]*sin(thetastep - pi/2.0);
		line(Image,Point(laser_y,laser_x),Point(laser_end_y,laser_end_x),Scalar(0,255,0),3); // Drawing lines to come up with lidar scans
	}

	namedWindow("RobotImage",WINDOW_AUTOSIZE);
	imshow("RobotImage",Image);

	waitKey(10);
}

// Get the weighted robot position using the particles state
void mclocalization::getRobotposition()
{	
	robot_position.x = 0;
	robot_position.y = 0;
	robot_position.theta = 0;
	float weightsum = 0;

	for(int i = 0;i < _numParticles; i++)
	{
		robot_position.x += particles_pf[i].x*particles_pf[i].weight; // X Position
		robot_position.y += particles_pf[i].y*particles_pf[i].weight; // Y Position
		robot_position.theta += particles_pf[i].theta*particles_pf[i].weight; // Theta
		weightsum += particles_pf[i].weight;
	}

	// Final weighted robot position
	robot_position.x /= weightsum;
	robot_position.y /= weightsum;
	robot_position.theta /= weightsum;


}

// MCL Algorithm
void mclocalization::MCLalgorithm()
{
	int input_data_size = _logfiledata.size();
	particles temp_particle_state;
	float weight;
	
	for (int i = 1;i < input_data_size; i++)
	{
		vector<particles> temp_particles;
		// Input state = (xt-1,xt) = (omm_control.*l,omm_control.*h)
		omm_control.xl = _logfiledata[i-1].x;
		omm_control.yl = _logfiledata[i-1].y;
		omm_control.thetal = _logfiledata[i-1].theta;
		omm_control.xh = _logfiledata[i].x;
		omm_control.yh =_logfiledata[i].y;
		omm_control.thetah = _logfiledata[i].theta;

		// cout << " Control (" << i << "/" << input_data_size << "): " <<  "(" << omm_control.xl << "," << omm_control.yl << "," 
		// 	<< omm_control.thetal << "," << omm_control.xh << "," 
		// 	<< omm_control.yh << "," << omm_control.thetah << ")" << endl;

		long total_weight = 0;
		for(int j = 0; j < _numParticles; j++)
		{
			// Using Odometry motion model to sample next state
			temp_particle_state = Odometry_motion_model(particles_pf[j]);
			temp_particles.push_back(temp_particle_state); // Add these particles to temp particle state vector	
		}	

		cout << "Reading (" << i << "/" << input_data_size << ") log data! ";
		cout << "Robot Position (X,Y,theta): (" << robot_position.x << "," << robot_position.y << "," << robot_position.theta << ")" << std::endl; 

		particles_pf = temp_particles;		
		
		if(_logfiledata[i].data_source == 0) // if 0, No measurement reading so go back
			continue;
		// cout << "New Measurement Reading! " << endl;
		
		mmm_measure.readings = _logfiledata[i].readings; // CUrrent measurement reading

		for(int j = 0; j < _numParticles; j++){	
			weight = measurement_model2(particles_pf[j]); // Using Measurement model to find the weight of the particle state
			particles_pf[j].weight = weight;
			// cout << "W: " << particles_pf[j].weight << endl;
			total_weight += weight;
		}
		
		// Normalizing the weights
		for(int k = 0; k < _numParticles; k++){
			particles_pf[k].weight /= total_weight;
		}

		/// Resampling step -- Low invariance sampler
		low_invariance_sampler();
		visualize(); // Visualize particles
		getRobotposition(); // Get the robot position
		visualize_log_robot(); // Visualize log data and robot position
		
	}
}

// Augmented MCL Algorithm - Add random particles whenever short term average likelihood is worse than
// long term average likelihood - This solves Kidnapped Robot problem and makes the algorithm more
// robust.
void mclocalization::Augmented_MCLalgorithm()
{
	int input_data_size = _logfiledata.size();
	particles temp_particle_state;
	float weight;
	
	float weight_fast = 0, weight_slow = 0, weight_avg = 0;

	for (int i = 1;i < input_data_size; i++)
	{

		vector<particles> temp_particles;
		// Input state = (xt-1,xt) = (omm_control.*l,omm_control.*h)
		omm_control.xl = _logfiledata[i-1].x;
		omm_control.yl = _logfiledata[i-1].y;
		omm_control.thetal = _logfiledata[i-1].theta;
		omm_control.xh = _logfiledata[i].x;
		omm_control.yh =_logfiledata[i].y;
		omm_control.thetah = _logfiledata[i].theta;

		// cout << " Control (" << i << "/" << input_data_size << "): " <<  "(" << omm_control.xl << "," << omm_control.yl << "," 
		// 	<< omm_control.thetal << "," << omm_control.xh << "," 
		// 	<< omm_control.yh << "," << omm_control.thetah << ")" << endl;

		cout << "Reading (" << i << "/" << input_data_size << ") log data! ";
		cout << "Robot Position (X,Y,theta): (" << robot_position.x << "," << robot_position.y << "," << robot_position.theta << ")" << std::endl; 

		long total_weight = 0;
		for(int j = 0; j < _numParticles; j++)
		{
			// Using Odometry motion model to sample next state
			temp_particle_state = Odometry_motion_model(particles_pf[j]);
			temp_particles.push_back(temp_particle_state); // Add these particles to temp particle state vector	
		}	

		particles_pf = temp_particles;		
		
		if(_logfiledata[i].data_source == 0) // If 0, no Measurement reading so go back
			continue;
		// cout << "New Measurement Reading! " << endl;
		
		mmm_measure.readings = _logfiledata[i].readings; // Current measurement reading

		for(int j = 0; j < _numParticles; j++){	
			weight = measurement_model2(particles_pf[j]); // Find weight of the current particle state
			particles_pf[j].weight = weight;
			total_weight += weight;
		}
		
		// Normalizing the weights
		for(int k = 0; k < _numParticles; k++){
			particles_pf[k].weight /= total_weight;
			weight_avg += particles_pf[k].weight/_numParticles;
		}

		weight_slow += _alpha_slow*(weight_avg - weight_slow);
		weight_fast += _alpha_fast*(weight_avg - weight_fast);

		/// Resampling step -- Low invariance sampler - Augmented - Adds random particles when short term average likelihood is worse
		// than long term average likelihood else continues normally
		low_invariance_sampler_augmented(weight_slow,weight_fast);
		visualize(); // Visualize particles
		getRobotposition(); // Get the current robot position
		visualize_log_robot(); // Visualize lidar data and the robot position
		
	}
}

// Using odometry motion model to sample next state
particles mclocalization::Odometry_motion_model(particles current_particle)
{
	float deltarot1 = atan2(omm_control.yh - omm_control.yl,omm_control.xh - omm_control.xl) - omm_control.thetal;
	float deltatrans1 = sqrt((omm_control.xl - omm_control.xh)*(omm_control.xl - omm_control.xh) + (omm_control.yl - omm_control.yh)*(omm_control.yl - omm_control.yh));
	float deltarot2 = omm_control.thetah - omm_control.thetal - deltarot1;

	float deltarot11 = deltarot1 - sample_normal_distribution(_alpha1*deltarot1 + _alpha2*deltatrans1);
	float deltatrans11 = deltatrans1 - sample_normal_distribution(_alpha3*deltatrans1 + _alpha4*(deltarot1 + deltarot2));
	float deltarot22 = deltarot2 - sample_normal_distribution(_alpha1*deltarot2 + _alpha2*deltatrans1);

	particles parti;
	parti.x = current_particle.x + (deltatrans11 / _resolution)*cos(current_particle.theta + deltarot11);
	parti.y = current_particle.y + (deltatrans11 / _resolution)*sin(current_particle.theta + deltarot11);
	parti.theta = current_particle.theta + deltarot11 + deltarot22;
	parti.weight = current_particle.weight;

	return parti;

}

// Beam based sensor model
float mclocalization::measurement_model(particles sampled_particle)
{
	float q = 1,p;
	robot_state lidar_position;

	float step_x,step_y,zktstar = 0, zkt = 0;

	// Lidar Position
	lidar_position.x = sampled_particle.x + (_lidar_offset/_resolution)*cos(sampled_particle.theta);
	lidar_position.y = sampled_particle.y + (_lidar_offset/_resolution)*sin(sampled_particle.theta);
	lidar_position.theta = sampled_particle.theta;

	if(lidar_position.x < _map->min_x || lidar_position.y < _map->min_y || lidar_position.x > _map->max_x
		|| lidar_position.y > _map->max_y)
		return 0.0;
		
	for (int i = 0; i < total_readings; i++)
	{
		zkt = mmm_measure.readings[i];
		if (zkt > _max_range)
			continue;
		
		float thetastep = i * (pi /180) + lidar_position.theta;

		/******************************* Ray Tracing **********************************/
		int step = 1;
		int var = 0;
		while(1)
		{
			step_x = lidar_position.x + step*cos( (thetastep - pi/2.0));
			step_y = lidar_position.y + step*sin( (thetastep - pi/2.0));

			if(step_x >= _map->max_x || step_y >= _map->max_y \
				|| step_x < _map->min_x || step_y < _map->min_y || _map->prob[(int)step_x][(int)step_y] < 0){
				var = 1;
				break;
			}
			else if(_map->prob[(int)step_x][(int)step_y] <= _threshold){
				zktstar = step;
				break;
			}

			step += _ray_tracing_step;
		}
		if(var == 1)
			continue;

		p = _zHit*measurement_prob_hit(zktstar,zkt) + _zshort*measurement_prob_short(zktstar,zkt) 
			+ _zRandom*measurement_prob_rand(zkt) + _zMax*measurement_prob_max(zkt);
		q = q*p;	

		/******************************************************************************/
	}
	return q;
}

// Basic measurement model - If the end of the laser beam has value less than the treshold, score = 1 else score = 0
float mclocalization::measurement_model2(particles sampled_particle)
{
	robot_state lidar_position;

	float step_x,step_y;
	float zktstar = 0, zkt = 0;

	// Lidar Position
	lidar_position.x = sampled_particle.x + (_lidar_offset/_resolution)*cos(sampled_particle.theta);
	lidar_position.y = sampled_particle.y + (_lidar_offset/_resolution)*sin(sampled_particle.theta);
	lidar_position.theta = sampled_particle.theta;

	if(lidar_position.x < _map->min_x || lidar_position.y < _map->min_y || lidar_position.x > _map->max_x
		|| lidar_position.y > _map->max_y || _map->prob[(int)lidar_position.x][(int)lidar_position.y] < _threshold)
		return 0.0;

	float score = 0;
	for (int i = 0; i < total_readings; i++)
	{
		zkt = mmm_measure.readings[i];
	
		float thetastep = (float)i * (pi /180) + lidar_position.theta;
		step_x = lidar_position.x + zkt*cos(thetastep - pi/2.0); // X end point of the lidar beam
		step_y = lidar_position.y + zkt*sin(thetastep - pi/2.0); // Y end point of the lidar beam
		
		if(step_x > _map->max_x || step_y > _map->max_y \
			|| step_x < _map->min_x || step_y < _map->min_y || _map->prob[(int)step_x][(int)step_y] <= 0)
			continue;
	
		score += _map->prob[(int)step_x][(int)step_y] < _threshold ? 1 : 0; // calculating score
	}
	
	return score;
}

// Measurment model - 3 - Find the minimum distance between the end point of the beam and the closest point which is
// occupied (_map->prob[x][y] == 0)
float mclocalization::measurement_model3(particles sampled_particle)
{
	float q = 1;
	float p;
	robot_state lidar_position;

	int zktstar = 0, zkt = 0;

	// Lidar Position
	lidar_position.x = sampled_particle.x + (_lidar_offset/_resolution)*cos(sampled_particle.theta);
	lidar_position.y = sampled_particle.y + (_lidar_offset/_resolution)*sin(sampled_particle.theta);
	lidar_position.theta = sampled_particle.theta;

	if(lidar_position.x < _map->min_x || lidar_position.y < _map->min_y || lidar_position.x > _map->max_x
		|| lidar_position.y > _map->max_y)
		return 0.0;
		
	for (int i = 0; i < total_readings; i++)
	{
		zkt = mmm_measure.readings[i];
		if (zkt > _max_range)
			continue;
		
		float thetastep = i * (pi /180) + lidar_position.theta;
		float step_x = lidar_position.x + zkt*cos( (thetastep - pi/2.0)); // X end point of the lidar beam
		float step_y = lidar_position.y + zkt*sin( (thetastep - pi/2.0)); // Y end point of the lidar beam

		float mindist = calculate_dist(step_x,step_y); // Find the minimum distance

		p = _zHit*measurement_prob_hit2(mindist) + _zshort*measurement_prob_short(zktstar,zkt) 
			+ _zRandom*measurement_prob_rand(zkt) + _zMax*measurement_prob_max(zkt);
		q = q*p;	

		/******************************************************************************/
	}
	return q;
}

// Finding the minimum distance
float mclocalization::calculate_dist(float x, float y)
{
	float minDist = 1000,dist;
	for(int i = 0;i < _map->size_x;i++)
	{
		for(int j = 0;j < _map->size_y;j++)
		{
			if(_map->prob[i][j] == 0) // if the point is occupied
			{
				dist = sqrt((i-x)*(i-x) + (j-y)*(j-y));
				if(dist < minDist)
					minDist = dist;
			}
		}
	}
	return minDist;
}

// Low variance sampling
void mclocalization::low_invariance_sampler()
{
	vector<particles> particles_temp = particles_pf;
	float r = 0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.0/_numParticles-0))); // initial random number between 0 and 1/_numParticles
	float c = particles_temp[0].weight;
	int i = 0;
	
	for (int m = 0;m < _numParticles; m++)
	{
		float u = r + (float) m/ _numParticles; // adding a constant number
		while (u > c && i < _numParticles - 1){ // Move to the next particle
			i++;
			c += particles_temp[i].weight;	
		}
		particles_pf[m] = particles_temp[i]; // New resampled particles
	}

}

// Low variance sampling - Augmented
void mclocalization::low_invariance_sampler_augmented(float w_slow, float w_fast)
{
	vector<particles> particles_temp = particles_pf;
	float r = 0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.0/_numParticles-0))); // Initial random number
	float c = particles_temp[0].weight;
	int i = 0;

	for (int m = 0;m < _numParticles; m++)
	{	
		float rand_no = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX));
		if(rand_no < max(0.0,1.0 - w_fast/w_slow)){ // Add random particle with probability max(0,1 - w_fast/w_slow)
			particles pTemp;
			while(1){
				// New Random particle
				pTemp.x =  rand() / (float)RAND_MAX * (_map->max_x - _map->min_x) + _map->min_x; 
				pTemp.y = rand() / (float)RAND_MAX * (_map->max_y - _map->min_y) + _map->min_y;
				pTemp.theta = rand() / (float)RAND_MAX * 2 * pi;
				pTemp.weight = 1.0/_numParticles;
			
				if (_map->prob[(int) pTemp.x][(int) pTemp.y] <= _threshold_particles) // Make sure the particle is in the valid region
					continue;
				else 
					break;
			}
			particles_pf[m] = pTemp; // Updating particle set
			continue;	
		}

		float u = r + (float) m/_numParticles;
		while (u > c && i < _numParticles - 1){
			i++;
			c += particles_temp[i].weight;	
		}
		particles_pf[m] = particles_temp[i];
	}

}

// Normal Distribution
float mclocalization::sample_normal_distribution(float b)
{	
	float sum = 0;
	for (int i = 0;i < 12; i++)
		//LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)))
		sum += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
	return (b/6.0)*sum;
}

// Triangular Distribution
float mclocalization::sample_triangular_distribution(float b)
{
	return b*(-1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1 - (-1))))) * (-1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1 - (-1)))));
}

float mclocalization::measurement_prob_hit(float zktstar,float zkt){
	if (zkt < 0 || zkt > _max_range)
		return 0;
	float q;
	q = (1.0/sqrt(2*pi*_sigmahit*_sigmahit)) * exp((-1/2*((zkt - zktstar)*(zkt - zktstar)))/(_sigmahit*_sigmahit));
	return q;
}

float mclocalization::measurement_prob_hit2(float a){
	float q;
	q = (1.0/sqrt(2*pi*_sigmahit*_sigmahit)) * exp((-1/2*(a*a))/(_sigmahit*_sigmahit));
	return q;
}

float mclocalization::measurement_prob_short(float zktstar,float zkt){
	if(zkt < 0 || zkt > zktstar)
		return 0;
	float q,eeta;
	eeta = 1 / (1 - exp(-1.0 * _lambdashort*zktstar));
	q = eeta*_lambdashort*exp(-1.0 * _lambdashort*zkt);
	return q;
}

float mclocalization::measurement_prob_max(float zkt){	
	if(zkt == _max_range)
		return 1;
	return 0;
}

float mclocalization::measurement_prob_rand(float zkt){
	if(zkt < 0 || zkt >= _max_range)
		return 0;
	return 1.0/_max_range;
}

int main(int argc, char** argv)
{
	cout << "MCLOCALIZATION" << endl;

	/****************** Loading the Map *******************/
	char mapName[100];  // Name of the map file
	strncpy(mapName,"../map/wean.dat",100);
    map_type* map = (map_type*) malloc(sizeof(map_type)); // Dynamic Memory Location
	read_beesoft_map(mapName,map);	// Read the map
	/******************************************************/	

	/**************** Loading the Log Data ****************/
	char logfilename[100];  // Name of the log file
	strncpy(logfilename,"../log/robotdata1.log",100);
	vector<logdata> logfiledata;
	loadlogdata(logfilename,logfiledata); // Load the log data
	/******************************************************/	

	/******************* MCL Algorithm ********************/
	vector<robot_state> init_particles;
	mclocalization MCL; 
	MCL.initialize_map(map); // Map initialization
	MCL.initialize_logdata(logfiledata); // Loading log data
	MCL.initialize_particles(); // Initialize particles
	MCL.getRobotposition(); // Get the robot position
	MCL.visualize(); // Visualize particles
	// MCL.MCLalgorithm(); // MCL Algorithm
	MCL.Augmented_MCLalgorithm(); // Augmented MCL Algorithm
	/******************************************************/	

	return 0;
}