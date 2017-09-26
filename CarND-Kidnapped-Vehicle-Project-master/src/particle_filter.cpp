/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *		Modified By : Anil devaraju
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	//total 800 particles 
	num_particles = 800;
	
	
	std::default_random_engine gen;
	
	std::normal_distribution<double> N_x(x, std[0]);
	std::normal_distribution<double> N_y(y, std[1]);
	std::normal_distribution<double> N_theta(theta, std[2]);
	
	int i = 0;
	
	//create and initialize 100 particles
	for( ;i < num_particles; ++i)
	{
		Particle p;
		p.id = i;
		p.x = N_x(gen);
		p.y = N_y(gen);
		p.theta = N_theta(gen);
		p.weight = 1;
		
		particles.push_back(p);
		weights.push_back(1);
	}
	
	//set initialization is done
	is_initialized = true; 

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	
	int i = 0;
	for( ;i < num_particles; ++i)
	{
		double n_x;
		double n_y;
		double n_theta;
		
		if(0 == yaw_rate)
		{
			n_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			n_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			n_theta = particles[i].theta + yaw_rate * delta_t; 
		}
		else
		{
		    n_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			n_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta));
			n_theta = particles[i].theta + yaw_rate * delta_t; 
		}
		
		std::normal_distribution<double> N_x(n_x, std_pos[0]);
		std::normal_distribution<double> N_y(n_y, std_pos[1]);
		std::normal_distribution<double> N_theta(n_theta, std_pos[2]);
		
		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);
		
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	int k=0;
	
	//go through each particle and update weights
	for( ;k < num_particles; ++k)
	{

		LandmarkObs lobs;
		vector<LandmarkObs> trnsfrm_observations;
		int i = 0;
		
		for( ; i< observations.size() ; ++i)
		{
			LandmarkObs transfrm_obs;
			lobs = observations[i];
			
			//transformation from vehicle to map
			transfrm_obs.x = particles[k].x + lobs.x * cos(particles[k].theta) - lobs.y * sin(particles[k].theta);
			transfrm_obs.y = particles[k].y + lobs.x * sin(particles[k].theta) + lobs.y * cos(particles[k].theta);
			trnsfrm_observations.push_back(transfrm_obs);
		}
		
		particles[k].weight = 1.0;
		
		for( i = 0 ; i < trnsfrm_observations.size(); ++i)
		{
			double closet_distance = sensor_range;
			int associated_prediction = 0;
			
			int j = 0;
			
			vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
			
			//Find the nearest landmark
			for( ; j < landmarks.size(); ++j)
			{
				double landmark_x = landmarks[j].x_f;
				double landmark_y = landmarks[j].y_f;
				
				double calculate_distance = dist(trnsfrm_observations[i].x, trnsfrm_observations[i].y, landmark_x, landmark_y);
				
				if(calculate_distance <= closet_distance)
				{
					closet_distance = calculate_distance;
					associated_prediction = j;
				}
				else
				{
					//TODO:
				}
			}
			
			if(0 != associated_prediction)
			{
				double measurement_x = trnsfrm_observations[i].x;
				double measurement_y = trnsfrm_observations[i].y;
				double mu_x = landmarks[associated_prediction].x_f;
				double mu_y = landmarks[associated_prediction].y_f;
				long double obs_w = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]) * exp(-(pow(measurement_x - mu_x, 2.0)  
																					/ (2 * pow(std_landmark[0],2.0)) + pow(measurement_y - mu_y,2.0) / (2*pow(std_landmark[1],2.0))));
		
										
				if(obs_w > 0)
				{
					particles[k].weight *= obs_w;
				}
			}
		}
		weights[k] = particles[k].weight;
	}
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	vector<Particle> new_particles;
	
	int i = 0;
	for( ; i < num_particles; ++i)
	{
		auto ind = distribution(gen);
		new_particles.push_back(particles[ind]);
	}
	particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
