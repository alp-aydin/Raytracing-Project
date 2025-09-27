#pragma once
#include "geometry.h"
#include <memory>

/// CSGOp: boolean operation applied to child solids.
enum class CSGOp { Union, Intersection, Difference };

/**
 * @brief Constructive Solid Geometry node combining two primitives.
 * Holds two child solids and composes them via Union/Intersection/Difference;
 * supports closest-hit queries and interval computation for CSG evaluation.
 */
class CSG final : public Primitive {
public:
    /// Create a CSG node with operation @p op over children @p a and @p b.
    CSG(CSGOp op, std::shared_ptr<Primitive> a, std::shared_ptr<Primitive> b)
        : op_(op), A_(std::move(a)), B_(std::move(b)) {}

    /**
     * @brief Closest-hit used by the renderer for primary/secondary rays.
     * @param ray Query ray.
     * @param tmin Inclusive lower bound for valid t.
     * @param tmax Exclusive upper bound for valid t.
     * @param out Filled with nearest hit in [tmin, tmax).
     * @return true if a surface of the composed solid is hit.
     */
    bool intersect(const Ray& ray, double tmin, double tmax, Hit& out) const override;

    /**
     * @brief Entry/exit interval for the composed solid along @p ray.
     * @param ray Query ray.
     * @param tEnter Entry parameter (<= tExit on success).
     * @param tExit Exit parameter.
     * @param enterHit Hit record at entry boundary (position/normal/material).
     * @param exitHit Hit record at exit boundary.
     * @return true if a valid inside interval exists.
     */
    bool interval(const Ray& ray,
                  double& tEnter, double& tExit,
                  Hit& enterHit, Hit& exitHit) const override;

private:
    /// Selected boolean operation.
    CSGOp op_;
    /// Left child solid.
    std::shared_ptr<Primitive> A_;
    /// Right child solid.
    std::shared_ptr<Primitive> B_;
};
