/*
 * normal_distribution_approximation.cpp
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#include <distribution/distribution.h>

normal_distribution_approximation::normal_distribution_approximation()
{
}

float normal_distribution_approximation::sample_distribution(float b_sq, float mean, float st_dev)
{
	// Construct math_util class
	math_util math;

	float num_sum;
	for (int i = 0; i < 13; ++i)
	{
		num_sum += math.random_uniform_sampling(sqrt(b_sq));
	}
	return (num_sum/2);
}
