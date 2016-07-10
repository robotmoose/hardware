/*
 * location.cpp
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Robot location representation
*/

#include "location.h"

/* 
 * location_t
 * Stores a point as cartesian coordinates and a normal vector as angle
*/

location_t::location_t(): x(0.0), y(0.0), direction(0.0) {}

location_t::location_t(double x, double y, const angle_t & dir): x(x), y(y), direction(dir) {}

location_t::location_t(double r, const angle_t & dir): direction(dir) {
	// Convert to cartesian and add vectors
	x = std::cos(direction) * r;
	y = std::sin(direction) * r;
}

location_t::location_t(const location_t & location, double r, const angle_t & dir) {
	// Correct for robot orientation
	direction = dir + location.get_direction();
	
	// Convert to cartesian and add vectors
	x = std::cos(direction) * r + location.get_x();
	y = std::sin(direction) * r + location.get_y();
}

location_t & location_t::operator+=(const location_t & other) {
	this->x += other.get_x();
	this->x += other.get_y();
	this->direction += other.get_direction();
	
	return *this;
}

location_t operator+(const location_t & p1, const location_t & p2) {
	return location_t(p1) += p2;
}

double location_t::get_x() const {
	return x;
}

double location_t::get_y() const {
	return y;
}

const angle_t & location_t::get_direction() const {
	return direction;
}

bool operator==(const location_t & l1, const location_t & l2) {
	return (l1.x == l2.x) && (l1.y == l2.y) && (l1.direction == l2.direction);
}