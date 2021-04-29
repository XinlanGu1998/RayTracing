#pragma once
#include "Object.hpp"

class Triangle:public Surface{
public:
    Vector3f v0,v1,v2;
    Vector3f normal;
    Triangle (Vector3f& v0, Vector3f& v1, Vector3f& v2, Material& m):v0(v0),v1(v1),v2(v2){
        material = &m;
        normal = (v1 - v0).cross(v2 - v1);
        normal = normal.normalized();
        bounds = getBounds();
    }
    bool intersect(const Ray& ray, float& t) override{
        if (normal.dot(ray.dir)==0) return false;
        t = normal.dot(v0 - ray.origin) / normal.dot(ray.dir);
        if (t < 0) return false;
        Vector3f p = ray.get(t);
        Vector3f cross0 = (p - v0).cross(v2 - v0);
        Vector3f cross1 = (p - v1).cross(v0 - v1);
        Vector3f cross2 = (p - v2).cross(v1 - v2);
        return cross0.dot(cross1)>=0 && cross1.dot(cross2)>=0;
    }
    void print() override{
        printf("Triangle.\n");
    }

    Vector3f getNormal(const Vector3f& point) override {
        return normal;
    }

    Bounds3 getBounds() override {
        Bounds3 b(v0,v1);
        return b.Union(v2);
    }
};

class TriangleMesh:public Surface{
public:
    int numTriangles;
    vector<Vector3f> vertices;
    int* vertexIndex;
    vector<Triangle*> tris;
    Triangle* hit_tri;
    TriangleMesh(int n, vector<Vector3f>& v, int* vidx, Material& m):numTriangles(n),vertices(v),vertexIndex(vidx){
        assert(numTriangles > 0);
        material = &m;
        bounds = getBounds();
        hit_tri = nullptr;
        Triangle* tri;
        for (int i = 0;i<n;i++){
            tri = new Triangle(vertices[vertexIndex[i]], 
                        vertices[vertexIndex[i+1]],
                        vertices[vertexIndex[i+2]], m);
            tris.push_back(tri);
        }
    }
    ~TriangleMesh(){
        for (int i = 0;i<numTriangles;i++){
            delete tris[i];
        }
    }
    bool intersect(const Ray& ray, float& t) override{
        bool ret = false;
        for (int i = 0; i < numTriangles; i++){
            Triangle* tri = tris[i];
            if (!tri->bounds.intersect(ray)) continue;
            float tmp = MAXDIST;
            if (tri->intersect(ray, tmp) && tmp < t){
                t = tmp;
                hit_tri = tri;
                ret = true;
            }
        }
        return ret;
    }
    Bounds3 getBounds() override {
        Bounds3 b(vertices[0],vertices[0]);
        for (int i = 1; i < vertices.size(); i++){
            b.Union(vertices[i]);
        }
        return b;
    }

    Vector3f getNormal(const Vector3f& point) override {
        if (hit_tri) return hit_tri->normal;
        return Vector3f(0,0,0);
    }
};