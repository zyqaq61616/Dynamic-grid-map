#include <iostream>
#include <sstream>
//ros相关
#include "ros/ros.h"
#include "std_msgs/String.h"

#include<vector>
#include<string>
#include<map>

using namespace std;
using namespace ros;
//定义命名空间
namespace dogm_holo
{
class DOGM_HOLO
{
public:
    /**
     * Parameters used for the DOGM_HOLO
     */
   struct Params
    {
        // Grid size [m]
        float size;

        // Grid cell size [m/cell]
        float resolution;

        // Number of persistent particles
        int particle_count;

        // Number of birth particles
        int new_born_particle_count;

        // Probability of persistence
        float persistence_prob;

        // Process noise position
        float stddev_process_noise_position;

        // Process noise velocity
        float stddev_process_noise_velocity;

        // Probability of birth
        float birth_prob;

        // Velocity to sample birth particles from (normal distribution) [m/s]
        float stddev_velocity;

        // Velocity to sample the initial particles from (uniform distribution) [m/s]
        float init_max_velocity;
    };
        /**
     * Constructor.
     * @params params parameter used for the grid map and the particle filter.
     */
    DOGM_HOLO(const Params& params);

    /**
     * Destructor.
     */
    ~DOGM_HOLO();
};
}
/* namespace dogm */