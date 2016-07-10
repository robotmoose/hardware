/*
 * location.h
 * Author: Aven Bross, Max Hesser-Knoll
 * Date: 11/29/2015
 * 
 * Description:
 * Robot location representation
*/

#ifndef __LOCATION_H__
#define __LOCATION_H__

#include<cmath>
#include<utility>
#include "angle.h"

// Stores a point as cartesian coordinates and a normal vector as angle
class location_t {
public:
	location_t();
	location_t(double x, double y, const angle_t & dir);
	location_t(double r, const angle_t & dir);
	location_t(const location_t & location, double r, const angle_t & direction);
	
	double get_x() const;
	double get_y() const;
	const angle_t & get_direction() const;
	
	location_t & operator+=(const location_t & other);  
	
	friend bool operator==(const location_t & l1, const location_t & l2);
private:
	double x, y;	// Cartesian coordinates
	angle_t direction;	// Angle representing normal vector
};

location_t operator+(const location_t & p1, const location_t & p2);
bool operator==(const location_t & l1, const location_t & l2);

#endif