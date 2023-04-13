#ifndef SMT_CONTROLLER_PATHFINDER_HPP
#define SMT_CONTROLLER_PATHFINDER_HPP

#include <vector>

#include <tl/optional.hpp>

namespace smt {
    namespace controller {
        namespace algorithm {
            class PathFinder {
            public:
                PathFinder(const float measurement_min, const float measurement_max, const float angle_min,
                           const float angle_max, const float angle_increment, const float min_distance) :
                        measurement_range(measurement_min, measurement_max),
                        angle_range(angle_min, angle_max),
                        angle_increment(angle_increment), min_distance(min_distance) {};

                tl::optional<float> calculateNewHeading(const std::vector<float> &measurements);

            private:
                template<typename T>
                struct MinMax {
                    MinMax(const T min, const T max) : min(min), max(max) {};

                    const T min;
                    const T max;
                };

                const MinMax<float> measurement_range;
                const MinMax<float> angle_range;
                const float angle_increment;

                const float min_distance;

                tl::optional<float> previousHeading;

                MinMax<tl::optional<float>> findMinAndMaxDistances(const std::vector<float> &measurements) const;

                std::vector<float> extractValidMeasurements(const std::vector<float> &measurements) const;

                float headingToNearestPointAtDistance(const std::vector<float> &measurements,
                                                      float desired_range) const;
            };
        } // algorithm
    } // smt
} // controller

#endif //SMT_CONTROLLER_PATHFINDER_HPP
