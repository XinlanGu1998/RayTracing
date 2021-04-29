#pragma once
#include "Object.hpp"

class Sphere: public ImplicitSurface{
public:
    Vector3f center;
    float radius;
    Sphere(Vector3f c, float r, Material& m): center(c),radius(r){
        material = &m;
        bounds = getBounds();
    }
    float equation(const Vector3f& point){
        Vector3f dist = point - center;
        float norm_sqr = dist.norm();
        return norm_sqr * norm_sqr - radius * radius;
    }
    
    inline bool onSurface(const Vector3f& point){
        return fEquals(equation(point), 0);
    }
    
    inline bool inside(const Vector3f& point){
        return equation(point) < 0;
    }


    bool intersect(const Ray& ray, float& t) override{
        float a = ray.dir.dot(ray.dir);
        Vector3f dist = ray.origin - center;
        float b = 2 * dist.dot(ray.dir);
        float c = dist.dot(dist) - radius * radius;
        float t0, t1;
        if (!solveQuadratic(a,b,c,t0,t1)) return false;
        if (t0 < 0 && t1 < 0) return false;
        if (t0 > 0) t = t0; else t = t1;
        return true;
    }

    void print() override{
        printf("Sphere. Center at (%.2f,%.2f,%.2f), radius=%.2f.\n", center.x(), center.y(),center.z(), radius);
    }

    Vector3f getNormal(const Vector3f& point) override {
        return (point - center).normalized();
    }
    Bounds3 getBounds() override {
        return Bounds3(center - Vector3f(radius, radius, radius),
                       center + Vector3f(radius, radius, radius));
    }
};