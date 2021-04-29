#include "Renderer.hpp"
#include <fstream>
#include <ctime>

void Renderer::write(){
    ofstream fout(filename);
    fout << "P3\n"<<width<<" "<<height<<"\n255\n"; 

    for (int j = height-1;j>=0;j--){
        for (int i = 0;i<width;i++){
            Vector3d color = buffer[i*height+j];
            fout << color.x() << " " << color.y() <<" "<< color.z()<<endl;
        }
    }

    cout<<"Image output: "+filename<<endl;
    fout.close();
}

bool hit(Ray& ray, Scene& scene, float& t, Surface* &sfc){
    bool ret = false;
    for (auto sfc_ptr:scene.objects){
        float tmp = MAXDIST;
        if (!sfc_ptr->bounds.intersect(ray)) continue;
        if (sfc_ptr->intersect(ray,tmp) && tmp<t && tmp>0.001){
            t = tmp;
            sfc = sfc_ptr;
            ret = true;
        }
    }
    return ret;
}

Vector3f Renderer::shade(Scene& scene, Vector3f& point, Vector3f& normal, Material& mat){
    Camera* cam = &scene.camera;
    Vector3f color = mul_vec3(mat.ka, bgColor);  //ambient

    for (auto light_ptr:scene.lights){
        float radius = (light_ptr->position - point).norm();// / 5.0f;
        float intensity = 1/(radius*radius) * light_ptr->intensity;

        Vector3f l_vec = (light_ptr->position - point).normalized();
        Vector3f v_vec = (cam->pos - point).normalized();
        Vector3f h_vec = (l_vec + v_vec).normalized();
        Ray shadow_ray(point,l_vec);
        float tmp = MAXDIST; 
        Surface* sfc_ptr;
        if (!hit(shadow_ray,scene,tmp,sfc_ptr)) {
            Vector3f diffuse = mat.kd * max(0.0f, l_vec.dot(normal)) * intensity;
            Vector3f specular = mat.ks * pow(max(0.0f, h_vec.dot(normal)), 100) * intensity;
            color += mul_vec3(diffuse+specular, light_ptr->color);
        }
            
    }
    return color;
}

Vector3f Renderer::trace(Ray& ray, Scene& scene){
    float tmin = MAXDIST;
    Surface* sfc = nullptr;
    if (!hit(ray,scene,tmin,sfc)) return bgColor;
    assert(sfc!=nullptr);
    
    Vector3f p = ray.get(tmin);
    Vector3f normal = sfc->getNormal(p);
    
    Vector3f color = shade(scene, p, normal,*(sfc->material));

    Vector3f d_reflect, d_refract;
    bool refracted = sfc->material->refract(ray.dir, normal, d_refract);
    bool reflected = sfc->material->reflect(ray.dir, normal, d_reflect);
    Vector3f reflect_color(0,0,0), refract_color(0,0,0);
    float kr = 1;
    if (refracted){
        Dialectric* mat = (Dialectric*)sfc->material;
        kr = fresnel(ray.dir, normal, mat->ref_idx);
        Vector3f origin;
        if (d_refract.dot(normal) < 0){
            origin = p - (0.001f * normal);
        }else{
            origin = p + (0.001f * normal);
        }
        Ray refracted_ray(origin, d_refract);
        refract_color = sfc->material->attenuation * mul_vec3(sfc->material->kd, trace(refracted_ray, scene)); 
    }
    if (reflected){
        Ray reflected_ray(p, d_reflect);
        reflect_color = sfc->material->attenuation * mul_vec3(sfc->material->kd, trace(reflected_ray, scene)); 
    }
    assert(kr>=0 && kr<=1);
    color += reflect_color * kr + refract_color * (1 - kr);

    return color;
}

void Renderer::render(Scene& scene){

    clock_t start_t = clock();

    Vector3f camera_pos = scene.camera.pos;
    float camera_z = fabs(camera_pos.z());
    float aspect_ratio = (float)width / (float)height;
    float half_fov_rad = deg2rad(scene.camera.fov/2.0f);
    float ymax = camera_z * tan(half_fov_rad);
    float xmax = ymax * aspect_ratio;
    float unit = ymax / ((float)height/2.0f);
    Vector3f start(-xmax,-ymax,0);

    int progress = 0;

    for (int i = 0;i<width;i++){
        for (int j = 0;j<height;j++){
            Vector3f pixel_pos = start + Vector3f(i*unit, j*unit,0);
            Ray ray(camera_pos, pixel_pos - camera_pos);
            Vector3f color = trace(ray, scene);

            for (int s = 1;s<n_sampling;s++){
                float u = float(i) + rand1();
                float v = float(j) + rand1();
                Vector3f pixel_pos = start + Vector3f(u*unit, v*unit,0);
                Ray ray(camera_pos, pixel_pos - camera_pos);
                color += trace(ray, scene);
            }
            color /= (float)(n_sampling);
            setPixel(i,j,color);
        }
        int p = (i+1)* 100 / width;
        if (p > progress){
            progress = p;
            cout<<p<<"% ";fflush(stdout);
        }
    }
    clock_t end_t = clock();
    float time = (end_t - start_t) / (float) CLOCKS_PER_SEC;
    printf("\nRendering completed. size = %d * %d, sampling = %d, time = %.2f s.\n", 
            width, height, n_sampling, time);
}