#pragma once
#include "Utils.hpp"

class Ray{
public:
    Vector3f origin;
    Vector3f dir;   //should be normalized
    Ray(Vector3f o, Vector3f d):origin(o),dir(d){
        dir.normalize();
    }
    Vector3f get(float t) const{
        assert(t>=0);
        //if (t<0) return origin;
        return origin + t * dir;
    }
};