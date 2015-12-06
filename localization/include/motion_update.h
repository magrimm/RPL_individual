/*
 * motion_update.h
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#ifndef _MOTION_UPDATE_H_
#define _MOTION_UPDATE_H_

#include <tf/transform_datatypes.h>
#include <distribution/distribution.h>
#include <parameter/motion_update_bag.h>
#include <localization_processor.h>

struct robot_control;
struct pose;

class motion_update
{
public:
	motion_update(motion_update_bag motion_update_params);
	void particle_motion(robot_control a_control, pose& a_particle);

private:
	motion_update_bag motion_update_params;
};



#endif /* _MOTION_UPDATE_H_ */
