#include "holo/perception/obstacle_gridmap/grid_map_types.h"

namespace holo
{
namespace gridmap
{
template <typename C, size_t W, size_t H>
class GridMap
{
public:
    using CellType = C;
    static size_t const WIDTH_VALUE = W;
    static size_t const HEIGHT_VALUE = H;
    GridMap(float32_t  resolution,Timestamp const& timestamp,uint32_t origin_x, uint32_t origin_y)
    {
        this->resolution_ = resulution;
        this->timestamp_ = timestamp;
        this->origin_x_ = origin_x;
        this->origin_y_ = origin_y;
    }
    GridMap(){};
    CellType const& operator()(int32_t x, int32_t y) const;
    CellType& operator()(int32_t x, int32_t y);
    CellType const& at(int32_t x, int32_t y) const;
    CellType& at(uint32_t x, uint32_t y);
    
    CellType const& operator()(float32_t x, float32_t y) const;
    CellType& operator()(float32_t x, float32_t y);
    CellType const& at(float32_t x, float32_t y) const;
    CellType& at(float32_t x, float32_t y);
    inline uint32_t GetOriginX()
    {
        return this->origin_x_;
    }
    inline uint32_t GetOriginY()
    {
        return this->origin_y_;
    }
    inline float32_t GetResolution()
    {
        return this->resolution_;
    }
private:
    holo::common::Timestamp timestamp_;
    
    float32_t resolution_;
    uint32_t origin_x_;
    uint32_t origin_y_;
    
    std::array<CellType, W * H> cells_;
};

class GridMapCell
{   
public:
    Point2f center_; //栅格中心点在车身坐标系下的坐标，单位m
    Point2f relative_velocity_;
    Classification classification_;
    bool is_occupied_;
    MotionStatus motion_status;
    uint32_t obstacle_id_;
};
}
}