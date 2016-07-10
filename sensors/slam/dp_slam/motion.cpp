/*
 * probabilistic_robotics.h
 * Author: Aven Bross
 * 
 * Probabilistic robot motion model.
*/

#include "motion.h"

control_t::control_t(const location_t & current_odometry, const location_t & last_odometry)
	: odometry(current_odometry), last_odometry(last_odometry) {}

bool control_t::null() const {
	return odometry == last_odometry;
}

const location_t & control_t::current() const {
	return odometry;
}

const location_t & control_t::last() const {
	return last_odometry;
}


odometry_motion_t::odometry_motion_t(double a1, double a2, double a3, double a4)
	: a1(a1), a2(a2), a3(a3), a4(a4) {}

location_t odometry_motion_t::sample(const control_t & odometry, const location_t & last_pos) {
	if(odometry.current() == odometry.last()) return last_pos;
	double dx_bar = odometry.current().get_x() - odometry.last().get_x();
	double dy_bar = odometry.current().get_y() - odometry.last().get_y();
	
	double delta_rot1 = std::atan2(dy_bar, dx_bar) - odometry.last().get_direction();
	//std::cout << "\tdr1 = " << delta_rot1 << ", ";
	double delta_trans = std::sqrt(std::pow(dx_bar, 2) + std::pow(dy_bar, 2));
	double delta_rot2 = odometry.current().get_direction() - odometry.last().get_direction() - delta_rot1;
	//std::cout << delta_rot2 << "\n";
	
	double dhat_rot1 = delta_rot1 - normal(a1 * std::pow(delta_rot1, 2) + a2 * std::pow(delta_trans, 2));
	double dhat_trans = delta_trans -
		normal(a3 * std::pow(delta_trans, 2) + a4 * std::pow(delta_rot1, 2) + a4 * std::pow(delta_rot2, 2));
	double dhat_rot2 = delta_rot2 - normal(a1 * std::pow(delta_rot2, 2) + a2 * std::pow(delta_trans, 2));
	
	location_t new_loc(last_pos, dhat_trans, dhat_rot1);
	location_t rotation(0, 0, dhat_rot2);
	
	return new_loc + rotation;
}

double odometry_motion_t::get_probability(const location_t & pos, const control_t & odometry,
	const location_t & last_pos) const
{
	double dx_bar = odometry.current().get_x() - odometry.last().get_x();
	double dy_bar = odometry.current().get_y() - odometry.last().get_y();
	
	double delta_rot1 = std::atan2(dy_bar, dx_bar) - odometry.last().get_direction();
	double delta_trans = std::sqrt(std::pow(dx_bar, 2) + std::pow(dy_bar, 2));
	double delta_rot2 = odometry.current().get_direction() - odometry.last().get_direction() - delta_rot1;
	
	double dx = pos.get_x() - last_pos.get_x();
	double dy = pos.get_y() - last_pos.get_y();
	
	double dhat_rot1 = std::atan2(dy, dx) - last_pos.get_direction();
	double dhat_trans = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
	double dhat_rot2 = pos.get_direction() - last_pos.get_direction() - dhat_rot1;
	
	double p1 = prob(delta_rot1 - dhat_rot1, a1 * std::pow(dhat_rot1, 2) + a2 * std::pow(dhat_trans, 2));
	double p2 = prob(delta_trans - dhat_trans,
		a3 * std::pow(dhat_trans, 2) + a4 * std::pow(dhat_rot1, 2) + a4 * std::pow(dhat_rot2, 2));
	double p3 = prob(delta_rot2 - dhat_rot2, a1 * std::pow(dhat_rot2, 2) + a2 * std::pow(dhat_trans, 2));
	
	return p1 * p2 * p3;
}

double odometry_motion_t::normal(double b2) {
	return std::normal_distribution<double>(0, std::sqrt(b2))(random);
}

double odometry_motion_t::prob(double a, double b) const {
	return (1.0 / std::sqrt(2 * M_PI * std::pow(b, 2))) * std::exp(-0.5 * pow((a / b), 2));
}