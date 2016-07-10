/*
 * dp_slam.h
 * Author: Aven Bross
 * 
 * Implementation of distributed particle tree.
*/

#ifndef __DP_SLAM_H__
#define __DP_SLAM_H__

#include <vector>
#include <set>
#include <map>
#include <memory>
#include <limits>
#include <iostream>
#include <random>
#include <chrono>

#include "../mapping/location.h"
#include "motion.h"

struct coord_t {
	coord_t(std::size_t x, std::size_t y);
	std::size_t x, y;
};

class dp_map_t {
	public:
		dp_map_t(std::size_t size = 500, std::size_t particles = 100, std::size_t num_readings = 15);
		
		void update(const std::vector<double> & z_t, const control_t & u_t);
		void sample_map(std::vector<std::vector<int> > & map, location_t & location);
		std::size_t size() const;
	
	private:
		struct node_t {
			node_t(std::size_t id);
			
			void trim(dp_map_t & map);
			
			location_t location;
			unsigned int children;
			unsigned int id;
			bool leaf;
			std::vector<coord_t> modified_cells;
			node_t* parent;
		};
		
		void weight_particles(const std::vector<double> & z_t);
		void resample();
	    void rename(const coord_t & coord, unsigned int old_id, unsigned int new_id);
	    void erase(const coord_t & coord, unsigned int id);
	    int occupied(const coord_t & coord, node_t* node) const;
	    double range_model(const std::vector<double> & z_t, node_t* node) const;
	    double cast_ray(angle_t direction, node_t* node) const;
	    void update_node(const std::vector<double> & z_t, node_t* node);
		void range_sensor_update(const location_t & begin, const location_t & end, node_t* node);
		node_t* make_child(const location_t & loc, std::size_t id, node_t* parent);
		
		std::vector<std::vector<std::map<unsigned int, bool> > > grid;
		std::mt19937 random;
		std::uniform_int_distribution<std::size_t> pdist;
		std::uniform_real_distribution<double> rdist;
		double range_std_dev;
		odometry_motion_t motion;
		node_t* root;
		std::vector<node_t*> particles;
		std::vector<double> weights;
		unsigned int id_count, start_index, range_size;
};

#endif