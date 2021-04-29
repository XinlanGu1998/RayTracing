#pragma once
#include "Utils.hpp"
#include "Ray.hpp"

inline Vector3f maxV3f(const Vector3f& v1, const Vector3f& v2){
    return Vector3f(max(v1.x(), v2.x()), max(v1.y(), v2.y()), max(v1.z(), v2.z()));
}

inline Vector3f minV3f(const Vector3f& v1, const Vector3f& v2){
    return Vector3f(min(v1.x(), v2.x()), min(v1.y(), v2.y()), min(v1.z(), v2.z()));
}

class Bounds3{
public:
    Vector3f pMin, pMax;
    Bounds3(){
        pMin = Vector3f(MAXDIST, MAXDIST, MAXDIST);
        pMax = Vector3f(-MAXDIST, -MAXDIST, -MAXDIST);
    }
    Bounds3(const Vector3f& p1, const Vector3f& p2){
        pMin = minV3f(p1,p2);
        pMax = maxV3f(p1,p2);
    }
    
    Bounds3 Union(const Vector3f& p){
        pMin = minV3f(pMin,p);
        pMax = maxV3f(pMax,p);
        return *this;
    }

    bool intersect(const Ray& ray){
        Vector3f o = ray.origin, d = ray.dir;

        float txmin = (pMin.x() - o.x())/d.x();
        float tymin = (pMin.y() - o.y())/d.y();
        float tzmin = (pMin.z() - o.z())/d.z();

        float txmax = (pMax.x() - o.x())/d.x();
        float tymax = (pMax.y() - o.y())/d.y();
        float tzmax = (pMax.z() - o.z())/d.z();

        if (txmin > txmax) swap(txmin, txmax);
        if (tymin > tymax) swap(tymin, tymax);
        if (tzmin > tzmax) swap(tzmin, tzmax);

        float t_enter = max(max(txmin, tymin), tzmin);
        float t_exit = min(min(txmax, tymax), tzmax);

        return t_enter <= t_exit && t_exit >= 0;
    }

};