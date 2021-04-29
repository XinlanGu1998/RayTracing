#pragma once
#include "Scene.hpp"

class Renderer{
public:
    string filename;
    int height;
    int width;
    int n_sampling;

    vector<Vector3d> buffer; 

    Vector3f bgColor;//background

    /**
     * @brief hi
     * @param bg V3f backgroud color
     * @param size 1, 2, or larger
     * @param sampling [1,100]
     */
    Renderer(Vector3f bg, int size, int s):bgColor(bg){
        if (size==1){
            height = 120;
            width = 160;
        }else if (size==2){
            height = 480;
            width = 640;
        }else{
            height = 960;
            width = 1280;
        }

        n_sampling = max(s,1);

        filename = "output.ppm";
        buffer.resize(height*width);
        /*
        for (auto iter=buffer.begin();iter!=buffer.end();iter++){
            (*iter) = bgColor;
        }*/
    }

    Vector3f shade(Scene& scene, Vector3f& point, Vector3f& normal, Material& mat);

    Vector3f trace(Ray& ray, Scene& scene);
    
    void render(Scene& scene);
    void write();

    inline Vector3d getPixel(int u,int v){
        return buffer[u*height+v];
    }

    inline void setPixel(int u, int v, Vector3f color){
        float rf = max(0.0f, min(color.x(),255.0f))/255.0f;
        int r = int(sqrt(rf)*255.99);
        float gf = max(0.0f, min(color.y(),255.0f))/255.0f;
        int g = int(sqrt(gf)*255.99);
        float bf = max(0.0f, min(color.z(),255.0f))/255.0f;
        int b = int(sqrt(bf)*255.99);
        buffer[u*height+v] = Vector3d(r,g,b);
    }

};