#pragma once
#include "Utils.hpp"

class Light{
public:
    Vector3f color;
    Vector3f position;
    float intensity;
    Light(Vector3f p, float i, Vector3f color): position(p),intensity(i), color(color){}
    void print(){
        printf("Light. pos at (%.2f,%.2f,%.2f), intensity=%.2f.\n",
        position.x(),position.y(),position.z(),intensity);
    }
};