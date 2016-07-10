/*
 * angle_t.h
 * Author: Aven Bross, Max Hesser-Knoll
 * 
 * Simple angle class
*/

#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

#ifndef __ANGLE_H__
#define __ANGLE_H__

#include<cmath>

// Stores an angle_t in radians between 0 and 2pi
class angle_t {
public:
	angle_t();
	angle_t(double angle_t);
	
	angle_t & operator+=(const angle_t & other);
	angle_t & operator-=(const angle_t & other);
	angle_t & operator*=(const angle_t & other);
	angle_t & operator/=(const angle_t & other);
	
	operator double() const { return radians; }
	
private:
	double radians;
};

#endif