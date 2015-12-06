/*
 * math_util.h
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#ifndef _MATH_UTIL_H_
#define _MATH_UTIL_H_

#include <stdlib.h>
#include <time.h>
#include <math.h>

class math_util
{
public:
	math_util ();
	float random_uniform_sampling (float a_b_max);
	float random_uniform_sampling_positive (float a_b_max);
};



#endif /* _MATH_UTIL_H_ */
