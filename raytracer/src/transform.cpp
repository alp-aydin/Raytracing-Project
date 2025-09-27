#include "transform.h"
#include <cmath>

// Simplified direct 3D implementation that follows the spec mathematically
// but avoids the matrix inverse complexity for now

// -------- Translation --------

/**
 * @brief Intersect child under a translation transform.
 * Applies inverse translation to the ray, queries the child, then maps the hit back to world.
 * @param r World-space ray.
 * @param tmin Inclusive lower bound for valid t.
 * @param tmax Exclusive upper bound for valid t.
 * @param out Filled with closest hit in range, with oriented normal.
 * @return true if an intersection in (tmin,tmax) is found.
 */
bool Translation::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    if (!child) return false;
    
    // Direct translation: transform ray to local space
    // For translation by vector t, the inverse is translation by -t
    Point3 local_origin(r.o.x - transform_matrix.m[0][3], 
                       r.o.y - transform_matrix.m[1][3], 
                       r.o.z - transform_matrix.m[2][3]);
    Ray local_ray(local_origin, r.d); // Direction unchanged for translation
    
    Hit h;
    if (!child->intersect(local_ray, 0.0, kINF, h)) return false;
    
    // Transform hit point back to world space
    Point3 world_point(h.p.x + transform_matrix.m[0][3],
                      h.p.y + transform_matrix.m[1][3], 
                      h.p.z + transform_matrix.m[2][3]);
    Dir3 world_normal = h.n; // Normal unchanged for translation
    
    double world_t = project_t_world(r, world_point);
    if (!(world_t > tmin && world_t < tmax)) return false;
    
    out = h;
    out.p = world_point;
    out.set_face_normal(r, world_normal);
    out.t = world_t;
    return true;
}

/**
 * @brief Entry/exit interval for translated child.
 * Transforms the ray to local space, queries the child interval, then maps both hits back.
 * @param r World-space ray.
 * @param tEnter Entry parameter (output).
 * @param tExit Exit parameter (output).
 * @param enterHit Entry boundary hit (position/normal oriented to ray).
 * @param exitHit Exit boundary hit (position/normal oriented to ray).
 * @return true if the child provides a valid interval.
 */
bool Translation::interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const {
    if (!child) return false;
    
    Point3 local_origin(r.o.x - transform_matrix.m[0][3], 
                       r.o.y - transform_matrix.m[1][3], 
                       r.o.z - transform_matrix.m[2][3]);
    Ray local_ray(local_origin, r.d);
    
    bool ok = child->interval(local_ray, tEnter, tExit, enterHit, exitHit);
    if (!ok) return false;
    
    // Transform both hits back to world space
    enterHit.p = Point3(enterHit.p.x + transform_matrix.m[0][3],
                       enterHit.p.y + transform_matrix.m[1][3],
                       enterHit.p.z + transform_matrix.m[2][3]);
    exitHit.p = Point3(exitHit.p.x + transform_matrix.m[0][3],
                      exitHit.p.y + transform_matrix.m[1][3],
                      exitHit.p.z + transform_matrix.m[2][3]);
    
    enterHit.set_face_normal(r, enterHit.n);
    exitHit.set_face_normal(r, exitHit.n);
    
    tEnter = project_t_world(r, enterHit.p);
    tExit = project_t_world(r, exitHit.p);
    return true;
}

// -------- Scaling --------

/**
 * @brief Intersect child under non-uniform scaling.
 * Uses inverse scale on ray to query child; maps hit back with inverse-transposed normal.
 * @param r World-space ray.
 * @param tmin Inclusive lower bound for valid t.
 * @param tmax Exclusive upper bound for valid t.
 * @param out Filled with closest hit in range.
 * @return true on hit; false if degenerate scales or miss.
 */
bool Scaling::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    if (!child) return false;
    
    // Extract scale factors from matrix
    double sx = transform_matrix.m[0][0];
    double sy = transform_matrix.m[1][1]; 
    double sz = transform_matrix.m[2][2];
    
    if (std::abs(sx) < kEPS || std::abs(sy) < kEPS || std::abs(sz) < kEPS) return false;
    
    // Transform ray to local space: scale down
    Point3 local_origin(r.o.x / sx, r.o.y / sy, r.o.z / sz);
    Dir3 local_direction(r.d.x / sx, r.d.y / sy, r.d.z / sz);
    Ray local_ray(local_origin, local_direction);
    
    Hit h;
    if (!child->intersect(local_ray, 0.0, kINF, h)) return false;
    
    // Transform hit point back to world space: scale up
    Point3 world_point(h.p.x * sx, h.p.y * sy, h.p.z * sz);
    
    // Transform normal back to world space: scale down and normalize
    Dir3 world_normal(h.n.x / sx, h.n.y / sy, h.n.z / sz);
    world_normal = world_normal.normalized();
    
    double world_t = project_t_world(r, world_point);
    if (!(world_t > tmin && world_t < tmax)) return false;
    
    out = h;
    out.p = world_point;
    out.set_face_normal(r, world_normal);
    out.t = world_t;
    return true;
}

/**
 * @brief Entry/exit interval for scaled child.
 * Transforms the ray by inverse scale, queries child, and maps boundary hits and normals back.
 * @param r World-space ray.
 * @param tEnter Entry parameter (output).
 * @param tExit Exit parameter (output).
 * @param enterHit Entry boundary hit (position/normal oriented to ray).
 * @param exitHit Exit boundary hit (position/normal oriented to ray).
 * @return true if the child provides a valid interval; false for degenerate scales.
 */
bool Scaling::interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const {
    if (!child) return false;
    
    double sx = transform_matrix.m[0][0];
    double sy = transform_matrix.m[1][1]; 
    double sz = transform_matrix.m[2][2];
    
    if (std::abs(sx) < kEPS || std::abs(sy) < kEPS || std::abs(sz) < kEPS) return false;
    
    Point3 local_origin(r.o.x / sx, r.o.y / sy, r.o.z / sz);
    Dir3 local_direction(r.d.x / sx, r.d.y / sy, r.d.z / sz);
    Ray local_ray(local_origin, local_direction);
    
    bool ok = child->interval(local_ray, tEnter, tExit, enterHit, exitHit);
    if (!ok) return false;
    
    // Transform both hits back to world space
    enterHit.p = Point3(enterHit.p.x * sx, enterHit.p.y * sy, enterHit.p.z * sz);
    exitHit.p = Point3(exitHit.p.x * sx, exitHit.p.y * sy, exitHit.p.z * sz);
    
    Dir3 enter_normal(enterHit.n.x / sx, enterHit.n.y / sy, enterHit.n.z / sz);
    Dir3 exit_normal(exitHit.n.x / sx, exitHit.n.y / sy, exitHit.n.z / sz);
    
    enterHit.set_face_normal(r, enter_normal.normalized());
    exitHit.set_face_normal(r, exit_normal.normalized());
    
    tEnter = project_t_world(r, enterHit.p);
    tExit = project_t_world(r, exitHit.p);
    return true;
}

// -------- Rotation --------  

/**
 * @brief Intersect child under axis-angle rotation.
 * Applies inverse rotation to ray, queries child, and rotates hit back to world.
 * @param r World-space ray.
 * @param tmin Inclusive lower bound for valid t.
 * @param tmax Exclusive upper bound for valid t.
 * @param out Filled with closest hit in range.
 * @return true on hit in (tmin,tmax).
 */
bool Rotation::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    if (!child) return false;
    
    // Apply inverse rotation to ray (rotate by -angle)
    Matrix4 inv_rotation = inverse_matrix;
    
    Vec4 local_origin_4d = inv_rotation * Vec4(r.o.x, r.o.y, r.o.z, 1.0);
    Vec4 local_direction_4d = inv_rotation * Vec4(r.d.x, r.d.y, r.d.z, 0.0);
    
    Point3 local_origin(local_origin_4d.x, local_origin_4d.y, local_origin_4d.z);
    Dir3 local_direction(local_direction_4d.x, local_direction_4d.y, local_direction_4d.z);
    Ray local_ray(local_origin, local_direction);
    
    Hit h;
    if (!child->intersect(local_ray, 0.0, kINF, h)) return false;
    
    // Transform hit point and normal back to world space
    Vec4 world_point_4d = transform_matrix * Vec4(h.p.x, h.p.y, h.p.z, 1.0);
    Vec4 world_normal_4d = transform_matrix * Vec4(h.n.x, h.n.y, h.n.z, 0.0);
    
    Point3 world_point(world_point_4d.x, world_point_4d.y, world_point_4d.z);
    Dir3 world_normal(world_normal_4d.x, world_normal_4d.y, world_normal_4d.z);
    
    double world_t = project_t_world(r, world_point);
    if (!(world_t > tmin && world_t < tmax)) return false;
    
    out = h;
    out.p = world_point;
    out.set_face_normal(r, world_normal.normalized());
    out.t = world_t;
    return true;
}

/**
 * @brief Entry/exit interval for rotated child.
 * Uses inverse rotation on the ray, queries child interval, and rotates boundary hits back.
 * @param r World-space ray.
 * @param tEnter Entry parameter (output).
 * @param tExit Exit parameter (output).
 * @param enterHit Entry boundary hit (position/normal oriented to ray).
 * @param exitHit Exit boundary hit (position/normal oriented to ray).
 * @return true if a valid interval is produced by the child.
 */
bool Rotation::interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const {
    if (!child) return false;
    
    Matrix4 inv_rotation = inverse_matrix;
    
    Vec4 local_origin_4d = inv_rotation * Vec4(r.o.x, r.o.y, r.o.z, 1.0);
    Vec4 local_direction_4d = inv_rotation * Vec4(r.d.x, r.d.y, r.d.z, 0.0);
    
    Point3 local_origin(local_origin_4d.x, local_origin_4d.y, local_origin_4d.z);
    Dir3 local_direction(local_direction_4d.x, local_direction_4d.y, local_direction_4d.z);
    Ray local_ray(local_origin, local_direction);
    
    bool ok = child->interval(local_ray, tEnter, tExit, enterHit, exitHit);
    if (!ok) return false;
    
    // Transform both hits back to world space
    Vec4 enter_point_4d = transform_matrix * Vec4(enterHit.p.x, enterHit.p.y, enterHit.p.z, 1.0);
    Vec4 exit_point_4d = transform_matrix * Vec4(exitHit.p.x, exitHit.p.y, exitHit.p.z, 1.0);
    Vec4 enter_normal_4d = transform_matrix * Vec4(enterHit.n.x, enterHit.n.y, enterHit.n.z, 0.0);
    Vec4 exit_normal_4d = transform_matrix * Vec4(exitHit.n.x, exitHit.n.y, exitHit.n.z, 0.0);
    
    enterHit.p = Point3(enter_point_4d.x, enter_point_4d.y, enter_point_4d.z);
    exitHit.p = Point3(exit_point_4d.x, exit_point_4d.y, exit_point_4d.z);
    
    enterHit.set_face_normal(r, Dir3(enter_normal_4d.x, enter_normal_4d.y, enter_normal_4d.z).normalized());
    exitHit.set_face_normal(r, Dir3(exit_normal_4d.x, exit_normal_4d.y, exit_normal_4d.z).normalized());
    
    tEnter = project_t_world(r, enterHit.p);
    tExit = project_t_world(r, exitHit.p);
    return true;
}
