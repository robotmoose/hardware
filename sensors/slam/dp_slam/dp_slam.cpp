/*
 * dp_slam.cpp
 * Author: Aven Bross
 * 
 * Implementation of distributed particle tree.
*/

#include "dp_slam.h"

coord_t::coord_t(std::size_t x, std::size_t y) : x(x), y(y) {}

bool operator<(const coord_t & c1, const coord_t & c2) {
	if(c1.x == c2.x) return c1.y < c2.y;
	return c1.x < c2.x;
}

dp_map_t::dp_map_t(std::size_t size, std::size_t nparticles, std::size_t num_readings)
	: id_count(1), start_index(0)
{
	range_size = 360 / num_readings;
	//std::cout << "constructing dp_map object ";
	grid.resize(size);
	
	for(std::size_t i = 0; i < grid.size(); ++i){
		grid[i].resize(size);
	}
	
	particles.resize(nparticles);
	weights.resize(nparticles);
	root = new node_t(0);
	root->location = location_t(size/2.0 + 0.5, size/2.0 + 0.5, 0.0);
	for(std::size_t i = 0; i < nparticles; ++i) {
		particles[i] = root;
		weights[i] = (1.0 / (double) nparticles);
	}
	
	pdist = std::uniform_int_distribution<std::size_t>(0, nparticles - 1);
	rdist = std::uniform_real_distribution<double>(0.0, 1.0 / (double) nparticles);
}

void dp_map_t::update(const std::vector<double> & z_t, const control_t & u_t) {
	//std::cout << "transforming particles ";
	{
		//auto start = std::chrono::high_resolution_clock::now();
		std::vector<node_t*> transformed_particles(particles.size());
		for(std::size_t i = 0; i < particles.size(); ++i) {
			location_t x_t;
			do {
				x_t = motion.sample(u_t, particles[i]->location);
			} while(occupied(coord_t(x_t.get_x(), x_t.get_y()), particles[i]) == 1);
			transformed_particles[i] = make_child(x_t, id_count++, particles[i]);
		}
	
		std::swap(transformed_particles, particles);
		//auto end = std::chrono::high_resolution_clock::now();
		//std::cout << " time = " << (((double)(end - start).count()) / 1000000000.0) << "s, ";
	}
	
	//std::cout << "weighting particles ";
	{
		//auto start = std::chrono::high_resolution_clock::now();
		weight_particles(z_t);
		//auto end = std::chrono::high_resolution_clock::now();
		//std::cout << " time = " << (((double)(end - start).count()) / 1000000000.0) << "s, ";
	}
	
	if((++start_index % range_size) == 0) {
		start_index = 0;
		//std::cout << "resampling particles ";
		//auto start = std::chrono::high_resolution_clock::now();
		resample();
		//auto end = std::chrono::high_resolution_clock::now();
		//std::cout << " time = " << (((double)(end - start).count()) / 1000000000.0) << "s, ";
	}
	else {
		//std::cout << "trimming tree ";
		//auto start = std::chrono::high_resolution_clock::now();
		// Trim to condense parent nodes
		for(node_t* particle : particles) {
			particle->leaf = true;
			particle->trim(*this);
		}
		//auto end = std::chrono::high_resolution_clock::now();
		//std::cout << " time = " << (((double)(end - start).count()) / 1000000000.0) << "s, ";
	}
	
	//std::cout << "updating maps ";
	{
		//auto start = std::chrono::high_resolution_clock::now();
		// Update range readings for each particle
		unsigned int last_id = 0;
		for(node_t* particle : particles) {
			if(particle->id != last_id) {
				update_node(z_t, particle);
			}
			last_id = particle->id;
		}
		//auto end = std::chrono::high_resolution_clock::now();
		//std::cout << " time = " << (((double)(end - start).count()) / 1000000000.0) << "s, ";
	}
}

void dp_map_t::sample_map(std::vector<std::vector<int> > & map, location_t & location) {
	//std::cout << "sampling map ";
	double r = rdist(random), c = weights.front();
	double inv_size = 1.0 / (double) particles.size();
	std::size_t i = 0, m = pdist(random);
	
	double u = r + m * inv_size;
	while(u > c) {
		++i;
		c += weights[i];
	}
	node_t* particle = particles[i];
	
	for(std::size_t i = 0; i < size(); ++i) {
		for(std::size_t j = 0; j < size(); ++j) {
			map[i][j] = occupied(coord_t(i,j), particle);
		}
	}
	location = particle->location;
}

std::size_t dp_map_t::size() const {
	return grid.size();
}

void dp_map_t::weight_particles(const std::vector<double> & z_t)
{
	double psum = 0.0;
	for(std::size_t i = 0; i < weights.size(); ++i) {
		double p = range_model(z_t, particles[i]);
		weights[i] *= p;
		psum += weights[i];
	}
	
	// Normalize weights
	for(std::size_t i = 0; i < particles.size(); ++i) {
		if(psum <= 0.000000001) weights[i] = 1.0 / (double) particles.size();
		else weights[i] /= psum;
	}
}

void dp_map_t::resample() {
	std::vector<node_t*> new_particles(particles.size());
	
	double r = rdist(random), c = weights.front();
	double inv_size = 1.0 / (double) particles.size();
	std::size_t i = 0;
	
	for(std::size_t m = 0; m < particles.size(); ++m) {
		double u = r + m * inv_size;
		while(u > c) {
			++i;
			c += weights[i];
		}
		new_particles[m] = particles[i];
		particles[i]->leaf = true;
	}
	
	for(std::size_t m = 0; m < weights.size(); ++m) {
		weights[m] = inv_size;
	}
	
	for(node_t* particle : particles) {
		particle->trim(*this);
	}
	
	std::swap(particles, new_particles);
}

void dp_map_t::rename(const coord_t & coord, unsigned int old_id, unsigned int new_id) {
	grid[coord.x][coord.y][new_id] = grid[coord.x][coord.y][old_id];
	grid[coord.x][coord.y].erase(old_id);
}

void dp_map_t::erase(const coord_t & coord, unsigned int id) {
	grid[coord.x][coord.y].erase(grid[coord.x][coord.y].find(id));
}

int dp_map_t::cell_count(const coord_t & coord, node_t* node) const {
	do {
		if(grid[coord.x][coord.y].count(node->id)) {
			return grid[coord.x][coord.y].at(node->id);
		}
		node = node->parent;
	} while(node != nullptr);
	
	return 0;
}

int dp_map_t::occupied(const coord_t & coord, node_t* node) const {
	int value = cell_count(coord, node);
	if(value > 0) return 1;
	if(value < 0) return 0;
	return -1;
}

double dp_map_t::range_model(const std::vector<double> & z_t, node_t* node) const {
	angle_t delta = 2.0 * M_PI / z_t.size();
	angle_t direction = -1.0 * delta * start_index;
	
	double q = 1.0;
	
	for(std::size_t i = start_index; i < z_t.size(); i += range_size) {
		if(z_t[i] != 0.0) {
			double z_ei = cast_ray(direction, node);
			if(z_ei >= 0.0) {
				double p = (1 / (std::sqrt(2 * M_PI * 25.0))) *
					std::exp(-0.5 * std::pow((z_t[i] - z_ei), 2) / 25.0) / 0.08;
				q *= p;
			}
		}
		direction -= (delta * range_size);
	}
	
	return q;
}

double dp_map_t::cast_ray(angle_t direction, node_t* node) const {
	location_t begin = node->location;
	location_t end = location_t(begin, 100.0, direction);
	
	double x0 = begin.get_x(), y0 = begin.get_y(); 
	double x1 = end.get_x(), y1 = end.get_y();
	double dx = fabs(x1 - x0), dy = fabs(y1 - y0);

	std::size_t x = floor(x0), y = floor(y0);
	std::size_t n = 1;
	
	int x_inc, y_inc;
	double error;

	if(dx == 0) {
		x_inc = 0;
		error = std::numeric_limits<double>::infinity();
	}
	else if (x1 > x0) {
		x_inc = 1;
		n += int(floor(x1)) - x;
		error = (floor(x0) + 1 - x0) * dy;
	}
	else {
		x_inc = -1;
		n += x - int(floor(x1));
		error = (x0 - floor(x0)) * dy;
	}

	if(dy == 0) {
		y_inc = 0;
		error -= std::numeric_limits<double>::infinity();
	}
	else if(y1 > y0) {
		y_inc = 1;
		n += int(floor(y1)) - y;
		error -= (floor(y0) + 1 - y0) * dx;
	}
	else {
		y_inc = -1;
		n += y - int(floor(y1));
		error -= (y0 - floor(y0)) * dx;
	}

	for(; n > 0; --n) {
		// Update grid cells seen through by this vector
		if(y >= 0 && y < size() && x >= 0 && x < size()) {
			int state = occupied(coord_t(x,y), node);
			if(state == 1) {
				return std::sqrt(std::pow(((double) x + 0.5) - x0, 2) + std::pow(((double) y + 0.5) - y0, 2));
			}
		}
		else {
			return -1.0;
		}
		
		// If we are are "below" the line, update y
		if(error > 0) {
			y += y_inc;
			error -= dx;
		}
		// Otherwise, we are "above" the line, update x
		else {
			x += x_inc;
			error += dy;
		}
	}
	
	return -1.0;
}

void dp_map_t::update_node(const std::vector<double> & z_t, node_t* node)
{
	angle_t delta = 2.0 * M_PI / z_t.size();
	angle_t direction = -1.0 * delta * start_index;
	
	location_t x_t = node->location;
	
	for(std::size_t i = start_index; i < z_t.size(); i += range_size)
	{
		if(z_t[i] != 0.0)
		{
			range_sensor_update(x_t, location_t(x_t, z_t[i], direction), node);
		}
		direction -= (delta * range_size);
	}
}

void dp_map_t::range_sensor_update(const location_t & begin, const location_t & end, node_t* node)
{
	double x0 = begin.get_x(), y0 = begin.get_y(); 
	double x1 = end.get_x(), y1 = end.get_y();
	double dx = fabs(x1 - x0), dy = fabs(y1 - y0);

	std::size_t x = floor(x0), y = floor(y0);
	std::size_t n = 1;
	
	int x_inc, y_inc;
	double error;

	if(dx == 0) {
		x_inc = 0;
		error = std::numeric_limits<double>::infinity();
	}
	else if (x1 > x0) {
		x_inc = 1;
		n += int(floor(x1)) - x;
		error = (floor(x0) + 1 - x0) * dy;
	}
	else {
		x_inc = -1;
		n += x - int(floor(x1));
		error = (x0 - floor(x0)) * dy;
	}

	if(dy == 0) {
		y_inc = 0;
		error -= std::numeric_limits<double>::infinity();
	}
	else if(y1 > y0) {
		y_inc = 1;
		n += int(floor(y1)) - y;
		error -= (floor(y0) + 1 - y0) * dx;
	}
	else {
		y_inc = -1;
		n += y - int(floor(y1));
		error -= (y0 - floor(y0)) * dx;
	}
	
	for(; n > 0; --n) {
		// Update grid cells seen through by this vector
		if(y >= 0 && y < size() && x >= 0 && x < size()) {
			int value = cell_count(coord_t(x,y), node);
			if(n > 1) {
				if(value > -10) {
					grid[x][y][node->id] = value - 1;
					node->modified_cells.emplace(x, y);
				}
			}
			else {
				if(value < 10) {
					grid[x][y][node->id] = value + 2;
					node->modified_cells.emplace(x, y);
				}
			}
			if(value > 0) return;
		}
		
		// If we are are "below" the line, update y
		if(error > 0) {
			y += y_inc;
			error -= dx;
		}
		// Otherwise, we are "above" the line, update x
		else {
			x += x_inc;
			error += dy;
		}
	}
}


dp_map_t::node_t::node_t(std::size_t id) : location(0, 0, 0.0), children(0), id(id), leaf(false),
	parent(nullptr) {}

dp_map_t::node_t* dp_map_t::make_child(const location_t & location, std::size_t id, node_t* parent) {
	parent->leaf = false;
	
	node_t* temp = new node_t(id);
	temp->location = location;
	temp->parent = parent;
	++(parent->children);
	
	return temp;
}

void dp_map_t::node_t::trim(dp_map_t & map) {
	if(parent == nullptr) {
		return;
	}
	
	if(!leaf && children == 0) {
		node_t* the_parent = parent;
		--(the_parent->children);
		
		for(const coord_t & cell : modified_cells) {
			map.erase(cell, id);
		}
		
		delete this;
		the_parent->trim(map);
	}
	// NEVER MERGE WITH THE ROOT
	else if(parent->children == 1 && parent->id != 0) {
		for(const coord_t & cell : modified_cells) {
			map.rename(cell, id, parent->id);
			parent->modified_cells.insert(cell);
		}
		std::swap(modified_cells, parent->modified_cells);
		id = parent->id;
		
		node_t* new_parent = parent->parent;
		delete parent;
		parent = new_parent;
		parent->trim(map);
	}
}
