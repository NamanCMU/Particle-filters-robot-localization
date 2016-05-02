#include <iostream>
#include <vector>
#include "bee-map.h"
#include "loadlogData.h"

// OPENCV HEADERS
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define pi 3.14159

using namespace cv;
using namespace std;

struct robot_state{
	float x,y,theta;
};

struct particles{
	float x,y,theta;
	float weight;
};

struct control{
	float xl,yl,thetal;
	float xh,yh,thetah;
};

struct measurements{
	float* readings;
};

class mclocalization
{
public:
	mclocalization();
	~mclocalization();
	void initialize_map(map_type*);
	void initialize_logdata(vector<logdata>&);
	void initialize_particles(); // Initialize particles
	void visualize();
	void getRobotposition();
	void Augmented_MCLalgorithm();
	void MCLalgorithm();
	void visualize_log_robot();
protected:
	float sample_normal_distribution(float);
	float sample_triangular_distribution(float);
	particles Odometry_motion_model(particles);
	float measurement_model(particles);
	float measurement_model2(particles);
	float measurement_model3(particles);
	float measurement_prob_hit(float,float);
	float measurement_prob_hit2(float);
	float measurement_prob_short(float,float);
	float measurement_prob_max(float);
	float measurement_prob_rand(float);
	void low_invariance_sampler();
	void low_invariance_sampler_augmented(float,float);
	float calculate_dist(float, float);
	float _alpha1,_alpha2,_alpha3,_alpha4;
	float _alpha_slow,_alpha_fast;
	float _sigmahit, _lambdashort;
	float _zHit,_zRandom,_zMax,_zshort;
	float _threshold, _threshold_particles, _lidar_offset;
	int _numParticles, _ray_tracing_step, _resolution, _max_range;
	map_type* _map;
	Mat _image_map,_image_robot;
	vector<logdata> _logfiledata;
	vector<particles> particles_pf;
	robot_state robot_position;
	measurements mmm_measure;
	control omm_control;
};
