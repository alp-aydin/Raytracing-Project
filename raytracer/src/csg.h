#pragma once
#include "geometry.h"
#include <memory>

enum class CSGOp { Union, Intersection, Difference };

class CSG final : public Primitive {
public:
    CSG(CSGOp op, std::shared_ptr<Primitive> a, std::shared_ptr<Primitive> b)
        : op_(op), A_(std::move(a)), B_(std::move(b)) {}

    // Closest hit used by the renderer
    bool intersect(const Ray& ray, double tmin, double tmax, Hit& out) const override;

    // Entry/exit interval used by boolean composition
    bool interval(const Ray& ray,
                  double& tEnter, double& tExit,
                  Hit& enterHit, Hit& exitHit) const override;

private:
    CSGOp op_;
    std::shared_ptr<Primitive> A_, B_;
};
