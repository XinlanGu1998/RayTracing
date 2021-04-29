#include "Scene.hpp"

void Scene::print(){
    cout<<"Scene:\n";
    for(auto iter = objects.begin();iter!=objects.end();iter++){
        (*iter)->print();
    }
}

void Scene::addObject(Surface& obj){
    objects.push_back(&obj);
    //cout<<"added object: ";obj.print();
}

void Scene::addLight(Light& light){
    lights.push_back(&light);
    //cout<<"added light: ";light.print();
}