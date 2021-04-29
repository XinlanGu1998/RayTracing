#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

#define epsilon 1e-6
#define MAXDIST 1e10
#define MINDIST 1e-4
#define MY_PI 3.1415926535

/**
 * @param t0 result0
 * @param t1 result1
 * @return Returns if quadratic equation ax^2+bx+c=0 is satisfiable.
 */
inline bool solveQuadratic(const float& a, const float& b, const float& c, float& t0, float& t1){
    float delta = b * b - 4 * a * c;
    if (delta < 0) return false;
    float sqrt_delta = sqrt(delta);
    t0 = (-b + sqrt_delta)/(2*a);
    t1 = (-b - sqrt_delta)/(2*a);
    if (t0>t1) swap(t0, t1);
    return true;
}

/**
 * @return Returns if 2 float params are equal. epsilon = 1e-6.
 */
inline bool fEquals(const float& a, const float& b){
    return fabs(a-b)<epsilon;
}

inline float deg2rad(const float& deg){
    return deg*MY_PI/180.0f;
}

inline float rand1(){
    return drand48()*2-1;
}

inline Vector3f mul_vec3(const Vector3f& a, const Vector3f& b){
    return Vector3f(a.x()*b.x(), a.y()*b.y(),a.z()*b.z());
}

inline Vector3f random_in_unit_sphere(){
    Vector3f v;
    do{
        v = 2.0* Vector3f(drand48(),drand48(),drand48()) - Vector3f(1,1,1);
    }while(v.norm() >= 1.0);

    return v;
}