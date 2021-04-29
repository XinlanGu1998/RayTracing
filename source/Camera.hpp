#pragma once
#include "Utils.hpp"

class Camera{
public:
    Vector3f pos;
    Vector3f lookAt;
    float fov;

    Camera(){
        pos = Vector3f(0,0,-5);
        lookAt = Vector3f(0,0,1);
        fov = 90;

    }
};