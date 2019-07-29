#ifndef PHIDGETS_SPATIAL_PHIDGETS_SPATIAL_NODELET_H
#define PHIDGETS_SPATIAL_PHIDGETS_SPATIAL_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_spatial/spatial_ros_i.h"

namespace phidgets {

class PhidgetsSpatialNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<SpatialRosI> spatial_;
};

}  // namespace phidgets

#endif  // PHIDGETS_SPATIAL_PHIDGETS_SPATIAL_NODELET_H
