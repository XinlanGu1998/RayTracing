#pragma once
#include "Utils.hpp"
#include "Ray.hpp"
#include "Material.hpp"
#include "Bounds3.hpp"
#include <vector>

class Surface{
public:
    Material* material;
    Bounds3 bounds;
    /**
     * @brief Find non-negative t s.t. ray(t) is on surface.
     */
    virtual bool intersect(const Ray& ray, float& t) = 0;

    /**
     * @brief Print information of the object.
     */
    virtual void print() = 0;

    /**
     * @brief Get nomal vector at a certain point on surface.
     */
    virtual Vector3f getNormal(const Vector3f& point) = 0;
    virtual Bounds3 getBounds() = 0;

};

class ImplicitSurface: public Surface{};

class Object{
    vector<Surface> sfcs;
};