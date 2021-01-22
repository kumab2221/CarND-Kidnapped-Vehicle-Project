/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <limits>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  double std_x;
  double std_y;
  double std_theta;

  std::default_random_engine gen;

  num_particles = 100;
  std_x         = std[0];
  std_y         = std[1];
  std_theta     = std[2];

  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  for(int i=0; i<num_particles; ++i){
    
    Particle particle;

    particle.id     = i;
    particle.x      = dist_x(gen);
    particle.y      = dist_y(gen);
    particle.theta  = dist_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(particle.weight);
  }
  
  is_initialized = true;

  std::cout << "x=" << x << std::endl;
  std::cout << "y=" << y << std::endl;
  std::cout << "debug state: initialize completed." << std::endl;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  double std_x;
  double std_y;
  double std_theta;

  std::default_random_engine gen;

  std_x         = std_pos[0];
  std_y         = std_pos[1];
  std_theta     = std_pos[2];

  for(int i=0; i<num_particles; ++i){
    double x_0     = particles[i].x;
    double y_0     = particles[i].y;
    double theta_0 = particles[i].theta;

    double x_f;
    double y_f;
    double theta_f;

    if(fabs(yaw_rate)<0.0001){
      x_f = x_0 + velocity * cos(theta_0) * delta_t;
      x_f = x_0 + velocity * sin(theta_0) * delta_t;
      theta_f = theta_0;
    }else{
      x_f = x_0 + (velocity/yaw_rate) * (sin(theta_0 + (yaw_rate*delta_t)) - sin(theta_0));
      y_f = y_0 + (velocity/yaw_rate) * (cos(theta_0) - cos(theta_0 + (yaw_rate*delta_t)));
      theta_f = theta_0 + (yaw_rate*delta_t);
    }

	  normal_distribution<double> dist_x(x_f, std_x);
	  normal_distribution<double> dist_y(y_f, std_y);
	  normal_distribution<double> dist_theta(theta_f, std_theta);  

    particles[i].x     = dist_x(gen);
	  particles[i].y     = dist_y(gen);
	  particles[i].theta = dist_theta(gen);

  }
  std::cout << "debug state: prediction completed." << std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {

  for (size_t i = 0; i < observations.size(); i++) {
    double obs_x = observations[i].x;
    double obs_y = observations[i].y;

    // Initialize the minimum distance
    double min_dist = std::numeric_limits<double>::max();
    int map_id;
    
    for (size_t j = 0; j < predicted.size(); j++) {
      double p_x = predicted[i].x;
      double p_y = predicted[i].y;
      double cur_dist = dist(obs_x, obs_y, p_x, p_y);

      if (cur_dist < min_dist) {
        min_dist = cur_dist;
        map_id = predicted[i].id;
      }
    }
    observations[i].id = map_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  
  double weight_normalizer = 0.0;

  for(int i=0; i<num_particles; ++i){
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    vector<LandmarkObs> map_coordinate_obs;

    // Transform observations from the VEHICLE'S coordinate system 
    // to the MAP'S coordinate system.
    for(size_t j=0; j<observations.size(); ++j){
      LandmarkObs obs;
      obs.x = p_x + (cos(p_theta) * observations[j].x) - (sin(p_theta) * observations[j].y);
      obs.y = p_y + (sin(p_theta) * observations[j].x) + (cos(p_theta) * observations[j].y);
      map_coordinate_obs.push_back(obs);
    }

    vector<LandmarkObs> predictions;

    // the map landmark locations predicted to be within sensor range of the particle
    for (size_t j=0; j<map_landmarks.landmark_list.size(); ++j) {

      int   lm_id = map_landmarks.landmark_list[j].id_i;
      float lm_x  = map_landmarks.landmark_list[j].x_f;
      float lm_y  = map_landmarks.landmark_list[j].y_f;
      
      if (fabs(lm_x-p_x)<=sensor_range && fabs(lm_y-p_y)<=sensor_range) {
        LandmarkObs obs;
        obs.id = lm_id;
        obs.x  = lm_x;
        obs.y  = lm_y;
        predictions.push_back(obs);
      }
    }

    dataAssociation(predictions, map_coordinate_obs);


    //Reset the weight of particle to 1.0
    particles[i].weight = 1.0;

    for (size_t j=0; j<map_coordinate_obs.size(); ++j) {
      
      double obs_x = map_coordinate_obs[j].x;
      double obs_y = map_coordinate_obs[j].y;
      int       id = map_coordinate_obs[j].id;

      double p_x,p_y;

      for (size_t k=0; k<predictions.size(); k++) {
        if (predictions[k].id == id) {
          p_x = predictions[k].x;
          p_y = predictions[k].y;
        }
      }

      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      double gauss_norm = 1 / (2*M_PI*sig_x*sig_y);
      double exponent =(pow(p_x-obs_x, 2) / (2*pow(sig_x, 2)))
                      +(pow(p_y-obs_y, 2) / (2*pow(sig_y, 2)));
      double weight = gauss_norm*exp(-exponent);
      particles[i].weight *= weight;
    }
    weight_normalizer += particles[i].weight;
  } 
  //Normalize the weights of all particles
  for (size_t i=0; i<particles.size(); ++i) {
    particles[i].weight /= weight_normalizer;
    weights[i]           = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  std::default_random_engine gen;
  std::uniform_int_distribution<int> particle_index(0, num_particles - 1);
  std::uniform_real_distribution<double> unirealdist(0.0, 1.0);

  vector<Particle> new_particles(num_particles);
  int    index = particle_index(gen);
  double beta  = 0;
  double w_max = *max_element(weights.begin(), weights.end()); 

  for(int i=0; i<num_particles; ++i){
      beta += unirealdist(gen)*(2*w_max);
      while(beta>weights[index]){
          beta-=weights[index];
          index=(index+1) % num_particles;
      }
      new_particles[i]=particles[index];
  }
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}