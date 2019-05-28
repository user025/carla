#pragma once

#include "carla/rpc/Location.h"

#include <cstdint>
#include <vector>


namespace carla {
namespace sensor {
namespace s11n {
class LidarGroundTruth {

    friend class LidarSerializer;
    friend class LidarHeaderView;

    public:
        explicit LidarGroundTruth(){};


        LidarGroundTruth &operator=(LidarGroundTruth&&) = default;
        void Reset(uint32_t total_point_count) {
            _points.clear();
            _points.reserve(3u * total_point_count);
        }

        void WriteId(int id) {
            _points.emplace_back(id);
        }

    private:

    std::vector<uint32_t> _header;

    std::vector<float> _points;
  };
};
}
}
}
