/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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
#include <limits>
#include <cstdlib>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = 500;

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  default_random_engine gen;
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  particles.resize(num_particles);
  weights.resize(num_particles);
  double init_weight = 1.0/num_particles;

  for(int i=0; i<num_particles; i++){
    double sample_x = dist_x(gen);
    double sample_y = dist_y(gen);
    double sample_theta = dist_theta(gen);

    Particle part;
    part.id = i;
    part.x = sample_x;
    part.y = sample_y;
    part.theta = sample_theta;
    part.weight = init_weight;

    particles[i] = part;
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  for(int i=0; i<num_particles; i++){
    double x0 = particles[i].x;
    double y0 = particles[i].y;
    double theta0 = particles[i].theta;

    double xf, yf, thetaf;
    if(fabs(yaw_rate) < 0.0001){
      xf = x0 + velocity*delta_t*cos(theta0);
      yf = x0 + velocity*delta_t*sin(theta0);
      thetaf = theta0;
    }else{
      xf = x0 + (velocity/yaw_rate)*(sin(theta0+yaw_rate*delta_t)-sin(theta0));
      yf = y0 + (velocity/yaw_rate)*(cos(theta0)-cos(theta0+yaw_rate*delta_t));
      thetaf = theta0 + yaw_rate*delta_t;
    }

    normal_distribution<double> dist_x(xf, std_x);
    normal_distribution<double> dist_y(yf, std_y);
    normal_distribution<double> dist_theta(thetaf, std_theta);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
    std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.
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

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  for(int i=0; i<num_particles; i++){
    // Particle position in global coord
    double x_p = particles[i].x;
    double y_p = particles[i].y;

    double sin_theta = sin(particles[i].theta);
    double cos_theta = cos(particles[i].theta);

    // weight of this particle
    double weight_exp = 0.0;

    for(int j=0; j<observations.size(); j++){
      LandmarkObs observation;
      observation.id = observations[j].id;

      // Landmark observation in global coordinate
      observation.x = x_p + (cos_theta*observations[j].x) - (sin_theta*observations[j].y);
      observation.y = y_p + (sin_theta*observations[j].x) + (cos_theta*observations[j].y);

      // Find the nearest landmark
      Map::single_landmark_s nearest_landmark;
      double min_dist = numeric_limits<double>::max();
      bool in_range = false;
      for(int k=0; k<map_landmarks.landmark_list.size(); k++){
        Map::single_landmark_s cond_landmark = map_landmarks.landmark_list[k];
        double distance = dist(observation.x, observation.y, cond_landmark.x_f, cond_landmark.y_f);
        if(distance < min_dist){
          min_dist = distance;
          nearest_landmark = cond_landmark;
          if(distance < sensor_range)
            in_range = true;
        }
      }

      // Sum of weight exponent
      if(in_range){
        double dx = observation.x - nearest_landmark.x_f;
        double dy = observation.y - nearest_landmark.y_f;
        weight_exp += dx*dx/(2*std_x*std_x) + dy*dy/(2*std_y*std_y);
      }else{
        // weight value will be small after exponential
        weight_exp += 100.0;
      }

    }

    particles[i].weight = exp(-weight_exp) / (2*M_PI*std_x*std_y);
    weights[i] = particles[i].weight;
  }

}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  srand(time(NULL));
  int index = rand() % num_particles;
  double beta = 0.0;
  double max_weight = numeric_limits<double>::min();

  for(int i=0; i<num_particles; i++){
    max_weight = max(max_weight, weights[i]);
  }

  vector<double> resample_weights(num_particles);
  vector<Particle> new_particles(num_particles);

  for(int i=0; i<num_particles; i++){
    beta = ((double) rand() / RAND_MAX) * 2.0 * max_weight;
    while(weights[index] < beta){
      beta -= weights[index];
      index = (index+1) % num_particles;
    }
    resample_weights[i] = weights[index];
    new_particles[i] = particles[index];
  }

  weights = resample_weights;
  particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
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
