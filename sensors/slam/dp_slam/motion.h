/*
 * probabilistic_robotics.h
 * Author: Aven Bross
 * 
 * Probabilistic robot motion model.
*/

#ifndef __MOTION_H__
#define __MOTION_H__

#include <cmath>
#include <random>
#include "../mapping/location.h"

class control_t {
	public:
		control_t(const location_t & current_odometry, const location_t & last_odometry);
		
		bool null() const;
		const location_t & current() const;
		const location_t & last() const;
	private:
		location_t odometry, last_odometry;
};

class odometry_motion_t {
	public:
		odometry_motion_t(double a1 = 0.001, double a2 = 0.001, double a3 = 0.001, double a4 = 0.001);
		
		location_t sample(const control_t & odometry, const location_t & last_pos);
		double get_probability(const location_t & pos, const control_t & odometry,
			const location_t & last_pos) const;
	
	private:
		double normal(double b2);
		double prob(double x, double b) const;
		
		double a1, a2, a3, a4;
		std::mt19937 random;
};

#endif