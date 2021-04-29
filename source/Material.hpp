#pragma once
#include "Utils.hpp"
#include "Ray.hpp"
#include "Light.hpp"

inline bool getRefract (const Vector3f& d_in, const Vector3f& normal, float eta, Vector3f& d_out){
    float cosi = d_in.dot(normal);
    float discriminant = 1.0 - eta * eta * (1 - cosi * cosi);
    if (discriminant > 0){
        d_out = eta * (d_in - normal * cosi) - normal * sqrtf(discriminant);
        return true;
    }else{
        return false;
    }
}

inline bool getReflect (const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out){
    d_out = d_in - 2 * normal * normal.dot(d_in);
    return d_out.dot(normal) > 0;
}

inline float fresnel(const Vector3f& d_in, const Vector3f& normal, const float ref_idx){
    float cosi = d_in.dot(normal);
    float ni, nt;
    if (cosi > 0){
        ni = ref_idx;nt = 1;
    }else{
        ni = 1; nt = ref_idx;
    }
    float sint = ni / nt * sqrtf(max(0.0f, 1 - cosi * cosi));
    if (sint >= 1) return 1;
    float cost = sqrtf(max(0.0f, 1 - sint*sint));
    cosi = fabs(cosi);
    float Rs = (nt * cosi - ni * cost) / (nt * cosi + ni * cost);
    float Rp = (ni * cosi - nt * cost) / (ni * cosi + nt * cost);
    return (Rs * Rs + Rp * Rp)/2; 
    
}


class Material{
public:
    Vector3f ks, kd, ka;
    float attenuation;

    Material(){
        ks = Vector3f(1,1,1);
        kd = Vector3f(1,1,1);
        ka = Vector3f(0.01, 0.01, 0.01);
    }
    Material(Vector3f ks, Vector3f kd, Vector3f ka):ks(ks),kd(kd),ka(ka){}

    virtual bool reflect(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) = 0;
    virtual bool refract(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) = 0;
};

class Lambertian: public Material{
public:
    Lambertian(Vector3f ks, Vector3f kd, Vector3f ka):Material(ks,kd,ka){
        attenuation = 0.3;
    }
    bool reflect(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) override{
        d_out = normal + Vector3f::Random();
        return true;
    }
    bool refract(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) override{
        return false;
    }
};

class Metal: public Material{
public:
    Metal (Vector3f ks, Vector3f kd, Vector3f ka):Material(ks,kd,ka){
        attenuation = 1;
    }
    bool reflect(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) override{
        return getReflect(d_in, normal, d_out);
    }
    bool refract(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) override{
        return false;
    }
};

class Dialectric: public Material{
public:
    float ref_idx;  //refraction index
    Dialectric(Vector3f ks, Vector3f kd, Vector3f ka, float ri):Material(ks,kd,ka),ref_idx(ri){
        attenuation = 1;
    }

    bool reflect(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) override{
        return getReflect(d_in, normal, d_out);
    }

    bool refract(const Vector3f& d_in, const Vector3f& normal, Vector3f& d_out) override{
        Vector3f n; //outward normal
        float eta;  //eta_i over eta_t

        if (d_in.dot(normal) > 0){
            n = - normal;
            eta = ref_idx;
        }else{
            n = normal;
            eta = 1.0f / ref_idx;
        }
        return getRefract(d_in, n, eta, d_out);
    }
};