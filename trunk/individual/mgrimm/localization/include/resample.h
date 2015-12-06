/*
 * resample.h
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#ifndef _RESAMPLE_H_
#define _RESAMPLE_H_

#include <geometry_msgs/PoseStamped.h>

class resample
{
public:
	virtual void resample_distribution() = 0;

private:
};

class stochastic_uniform_sampling : public resample
{
public:
	stochastic_uniform_sampling(std::vector<geometry_msgs::Pose>* particles);
	void resample_distribution();
	virtual ~stochastic_uniform_sampling() {};

private:
	std::vector<geometry_msgs::Pose>* particles;
};

#endif /* _RESAMPLE_H_ */
