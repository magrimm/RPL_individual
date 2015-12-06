/*
 * triangular_distribution_approximation.cpp
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#include <distribution/distribution.h>

triangular_distribution_approximation::triangular_distribution_approximation()
{
}

float triangular_distribution_approximation::sample_distribution(float b_sq, float mean, float st_dev)
{
	// Construct math_util class and get random numbers
	math_util math;
	float num = math.random_uniform_sampling(sqrt(b_sq)) + math.random_uniform_sampling(sqrt(b_sq));

	return (sqrt(6)*num/2);
}


