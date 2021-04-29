#pragma once
#include "Object.hpp"

class Cylinder: public ImplicitSurface{
public:
    Vector3f center;
    float halfHeight;
    float radius;
    Cylinder(Vector3f c, float h, float r, Material& m):center(c), halfHeight(h), radius(r){
        material = &m;
    }

    bool yInside(const Vector3f& rel_p){
        return (fabs(rel_p.y()) <= halfHeight);
    }

    bool xzInside(const Vector3f& rel_p){
        return pow(rel_p.x(), 2) + pow(rel_p.z(), 2) <= radius * radius;
    }

    bool intersectTop(Ray& rel_ray, float& t){
        t = (halfHeight - rel_ray.origin.y()) / rel_ray.dir.y();
        return  t>0 && xzInside(rel_ray.get(t));
    }

    bool intersectBottom(Ray& rel_ray, float& t){
        t = (-halfHeight - rel_ray.origin.y()) / rel_ray.dir.y();
        return  t>0 && xzInside(rel_ray.get(t));
    }

    bool intersect(const Ray& ray, float& t) override{
        if (!bounds.intersect(ray)) return false;
        Vector3f rel_o = ray.origin - center;
        Ray rel_ray(rel_o, ray.dir);
        
        float t0, t1;
        bool ret = false;
        if (intersectTop(rel_ray,t0)){ret = true;t = t0;}
        if (intersectBottom(rel_ray, t1)){
            if (!ret || t1 < t) t = t1;
            ret = true;
        }

        float a = pow(ray.dir.x(),2) + pow(ray.dir.z(),2);
        float b = 2 * (rel_o.x() * ray.dir.x() + rel_o.z() * ray.dir.z());
        float c = pow(rel_o.x(),2) + pow(rel_o.z(), 2) - radius * radius;
  
        if (!solveQuadratic(a,b,c,t0,t1)) return ret;
        if (t0 < 0 && t1 < 0) return ret;
       
        if (t0 > 0){
            if (yInside(rel_ray.get(t0))){
                if (!ret || t0 < t) t = t0;
                ret = true;
            }
        }
        if (t1 > 0){
            if (yInside(rel_ray.get(t1))){
                if (!ret || t1 < t) t = t1;
                ret = true;
            }
        }
        return ret;
    }

    void print() override{
        printf("Cylinder. Center at (%.2f,%.2f,%.2f), radius=%.2f.\n", center.x(), center.y(),center.z(), radius);
    }

    Vector3f getNormal(const Vector3f& point) override {
        if (fEquals(fabs(point.y() - center.y()), halfHeight)){
            return Vector3f(0,point.y() - center.y(),0).normalized();
        }else{
            return Vector3f(point.x() - center.x(),0,point.z() - center.z()).normalized();
        }
    }

    Bounds3 getBounds() override {
        return Bounds3(center - Vector3f(radius, halfHeight, radius),
                        center + Vector3f(radius, halfHeight, radius));
    }
};