#include <algorithm>

#include "PathFinder.hpp"

namespace smt {
    namespace controller {
        namespace algorithm {

            tl::optional<float>
            PathFinder::calculateNewHeading(const std::vector<float> &measurements) {
                const auto minAndMaxDistances = this->findMinAndMaxDistances(measurements);

                if (minAndMaxDistances.max.has_value() && minAndMaxDistances.max > this->min_distance) {
                    if (minAndMaxDistances.min.has_value() && *minAndMaxDistances.min <= this->min_distance) {
                        const auto newHeading = this->headingToNearestPointAtDistance(measurements, *minAndMaxDistances.max);
                        this->previousHeading = newHeading;
                        return newHeading;
                    } else {
                        return this->previousHeading;
                    }
                }

                // cannot find a way out of here!
                return tl::nullopt;
            }

            PathFinder::MinMax<tl::optional<float>>
            PathFinder::findMinAndMaxDistances(const std::vector<float> &measurements) const {
                const auto valid_ranges = measurements; // this->extractValidMeasurements(measurements);
                if (valid_ranges.empty()) {
                    return {tl::nullopt, tl::nullopt};
                } else {
                    const auto min = std::min_element(valid_ranges.cbegin(), valid_ranges.cend());
                    const auto max = std::max_element(valid_ranges.cbegin(), valid_ranges.cend());
                    return {*min, *max};
                }
            }

            std::vector<float> PathFinder::extractValidMeasurements(const std::vector<float> &measurements) const {
                std::vector<float> valid_ranges;
                const auto &measurement_range_ = this->measurement_range;
                std::copy_if(measurements.cbegin(), measurements.cend(), std::back_inserter(valid_ranges),
                             [&measurement_range_](float f) {
                                 return f >= measurement_range_.min && f <= measurement_range_.max;
                             });
                return valid_ranges;
            }

            float PathFinder::headingToNearestPointAtDistance(const std::vector<float> &measurements,
                                                              const float desired_range) const {
                const auto it = std::find(measurements.cbegin(), measurements.cend(), desired_range);
                const auto pos = std::distance(measurements.cbegin(), it);

                return this->angle_range.min + pos * this->angle_increment;
            }
        } // algorithm
    } // smt
} // controller