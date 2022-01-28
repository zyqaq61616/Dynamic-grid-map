#include<vector>

namespace holo
{
namespace perception
{
class OccupancyGridMap
{
    struct OccupancyGridCell
    {
        float x_vector;
        float y_vector;
        float x_position;
        float y_position;
        int grid_class;
        bool is_pposite_direction;
    };
    using Timestamp    = holo::common::details::Timestamp;
    using MapVector = std::vector<OccupancyGridCell>;



    MapVector occupancy_grid_map;

};
}
}
/* namespace dogm_holo */
