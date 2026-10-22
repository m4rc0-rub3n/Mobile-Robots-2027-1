/*
 * MOBILE ROBOTS - UNAM, FI, 2027-1
 * LOCALIZATION BY PARTICLE FILTERS
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */
#include "particle_filter/ray_tracer.h"
#include <cmath> // Necesario para std::isfinite y exp
#define FULL_NAME "Marco Ruben Guerrero Nieva"

class ParticleFilter
{
public:
    ParticleFilter(){}

    static std::vector<geometry_msgs::msg::Pose2D> get_initial_distribution(
    int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
    {
	random_numbers::RandomNumberGenerator rnd;
	std::vector<geometry_msgs::msg::Pose2D> particles(N);
	
	// Generar N partículas con distribución uniforme
	for(int i = 0; i < N; i++) {
	    particles[i].x = rnd.uniformReal(min_x, max_x);
	    particles[i].y = rnd.uniformReal(min_y, max_y);
	    particles[i].theta = rnd.uniformReal(min_a, max_a);
	}
	
	return particles;
    }

    static void move_particles(std::vector<geometry_msgs::msg::Pose2D>& particles,
			       float delta_x, float delta_y, float delta_t, float sigma2)
    {
	random_numbers::RandomNumberGenerator rnd;
	
	// Actualizar cada partícula con el desplazamiento y ruido
	for(size_t i = 0; i < particles.size(); i++) {
	    float theta = particles[i].theta;
	    // Rotar el desplazamiento al marco de referencia de la partícula
	    float dx = delta_x * cos(theta) - delta_y * sin(theta);
	    float dy = delta_x * sin(theta) + delta_y * cos(theta);

	    // Actualizar posición con ruido gaussiano
	    particles[i].x += dx + rnd.gaussian(0.0, sigma2);
	    particles[i].y += dy + rnd.gaussian(0.0, sigma2);
	    particles[i].theta += delta_t + rnd.gaussian(0.0, sigma2);
	}
    }

    static std::vector<sensor_msgs::msg::LaserScan> simulate_particle_scans(
	std::vector<geometry_msgs::msg::Pose2D>& particles,
	nav_msgs::msg::OccupancyGrid& map,
	sensor_msgs::msg::LaserScan& sensor_specs)
    {
	// Esta función ya estaba completa
	std::vector<sensor_msgs::msg::LaserScan> simulated_scans(particles.size());
	for(size_t i=0; i < particles.size(); i++)
	{
	    geometry_msgs::msg::Pose sensor_pose;
	    sensor_pose.position.x    = particles[i].x;
	    sensor_pose.position.y    = particles[i].y;
	    sensor_pose.orientation.w = cos(particles[i].theta/2);
	    sensor_pose.orientation.z = sin(particles[i].theta/2);
	    
	    simulated_scans[i] = ray_tracer::simulateRangeScan(map, sensor_pose, sensor_specs);
	    
	}
	return simulated_scans;
    }

    static std::vector<double> get_particle_similarities(
	std::vector<sensor_msgs::msg::LaserScan>& simulated_scans,
	sensor_msgs::msg::LaserScan& real_scan,
	int downsampling, float sigma2)
    {
	std::vector<double> similarities;
	similarities.resize(simulated_scans.size());
	
	// Calcular similitud y normalizar
	double sum_sim = 0.0;
	for(size_t i = 0; i < simulated_scans.size(); i++) {
	    double error = 0.0;
	    int valid_count = 0;

	    for(size_t j = 0; j < simulated_scans[i].ranges.size(); j++) {
		float sim_r = simulated_scans[i].ranges[j];
		float real_r = real_scan.ranges[j * downsampling];

		// Verificar que ambas lecturas sean valores finitos
		if(std::isfinite(sim_r) && std::isfinite(real_r)) {
		    double diff = sim_r - real_r;
		    error += diff * diff;
		    valid_count++;
		}
	    }

	    if(valid_count > 0) {
		// Similitud usando función Gaussiana
		similarities[i] = exp(-error / (2.0 * sigma2 * valid_count));
	    } else {
		similarities[i] = 0.0;
	    }
	    sum_sim += similarities[i];
	}

	// Normalizar para que sumen 1.0
	if(sum_sim > 0.0) {
	    for(size_t i = 0; i < similarities.size(); i++) {
		similarities[i] /= sum_sim;
	    }
	} else {
	    // Si todas fallaron, distribución uniforme
	    for(size_t i = 0; i < similarities.size(); i++) {
		similarities[i] = 1.0 / similarities.size();
	    }
	}
	
	return similarities;
    }
    
    static int random_choice(std::vector<double>& probabilities)
    {
	random_numbers::RandomNumberGenerator rnd;
	
	// Selección aleatoria basada en probabilidad acumulada
	double r = rnd.uniformReal(0.0, 1.0);
	double cum_sum = 0.0;
	for(size_t i = 0; i < probabilities.size(); i++) {
	    cum_sum += probabilities[i];
	    if(r <= cum_sum) {
		return i;
	    }
	}
	return probabilities.size() - 1;
    }

    static std::vector<geometry_msgs::msg::Pose2D> resample_particles(
	std::vector<geometry_msgs::msg::Pose2D>& particles, std::vector<double>& probabilities, float sigma2)
    {

	random_numbers::RandomNumberGenerator rnd;
	std::vector<geometry_msgs::msg::Pose2D> resampled_particles(particles.size());
	
	// Remuestreo con reemplazo y adición de ruido
	for(size_t i = 0; i < particles.size(); i++) {
	    int idx = random_choice(probabilities);
	    resampled_particles[i] = particles[idx];

	    // Agregar ruido gaussiano a las partículas remuestreadas
	    resampled_particles[i].x += rnd.gaussian(0.0, sigma2);
	    resampled_particles[i].y += rnd.gaussian(0.0, sigma2);
	    resampled_particles[i].theta += rnd.gaussian(0.0, sigma2);
	}
	
	return resampled_particles;
    }
    
};
