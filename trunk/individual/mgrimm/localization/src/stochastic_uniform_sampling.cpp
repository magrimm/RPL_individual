/*
 * stochastic_uniform_sampling.cpp
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#include <resample.h>

stochastic_uniform_sampling::stochastic_uniform_sampling (std::vector<geometry_msgs::Pose>* the_particles)
{
	particles = the_particles;
}

void stochastic_uniform_sampling::resample_distribution()
{

}
