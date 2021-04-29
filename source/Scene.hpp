#pragma once
#include "Sphere.hpp"
#include "Cylinder.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Camera.hpp"
#include <memory>
#include <vector>

class Scene{
public:
    vector<Surface*> objects;
    vector<Light*> lights;
    Camera camera;
    
    Scene(){}
    void print();
    void addObject(Surface& obj);
    void addLight(Light& light);
};