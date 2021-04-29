#include "Renderer.hpp"

void testSolveQuadratic(){
    float t0,t1;
    if (solveQuadratic(1,2,-1,t0,t1)) cout<<t0<<" "<<t1<<endl;
}

void testfEqual(){
    float t0 = sqrt(3);
    float t1 = 3.0f/t0;
    cout<<"t0-t1="<<t0-t1<<endl;
    if (t0==t1) cout<<"t0==t1"<<endl; else cout<<"t0!=t1"<<endl;
    if (fEquals(t0,t1)) cout<<t0<<"=="<<t1<<endl;
    else cout<<t0<<"!="<<t1<<endl;
}


void testrand1(){
    for (int i = 0;i<100;i++) cout<<rand1()<<endl;
}

void testScene(){

    Vector3f white(255,255,255);
    Vector3f lightBlue(120,220,220);

    Vector3f ka(0.01,0.01,0.01);

    Scene scene;

    Metal mat0(Vector3f(1,1,1), Vector3f(0.8,0.8,0.8), ka);
    Lambertian mat1(Vector3f(1,0,0),Vector3f(1,0,0), ka);
    Lambertian mat2(Vector3f(0.3,0.3,0.8),Vector3f(0.3,0.3,0.8),ka);
    Dialectric mat3(Vector3f(1,1,1),Vector3f(1,1,1), ka, 0.95);
    Dialectric mat4(Vector3f(1,1,1),Vector3f(1,1,1), ka, 1.5);

    Lambertian mat5(Vector3f(1,1,0),Vector3f(1,1,0), ka);
    
    Sphere s0(Vector3f(2,0,1), 2, mat0);    scene.addObject(s0);
    //cout<<s0.bounds.pMin << endl << s0.bounds.pMax<<endl;
    
    //Sphere s1(Vector3f(-2,-1,3), 2, mat1);    scene.addObject(s1);
    //Sphere s2(Vector3f(0,-203,0),200, mat2);    scene.addObject(s2);
    Sphere s3(Vector3f(0,1,-2),0.8,mat3);    scene.addObject(s3);
    Sphere s4(Vector3f(-3,-1.2,-0.5),0.8,mat4);    scene.addObject(s4);
    //Sphere s4(Vector3f(0,0,0),2,mat4);    scene.addObject(s4);

    Cylinder c0(Vector3f(-0.5,-1.5,-1), 0.5, 1, mat5);  scene.addObject(c0);
    //Cylinder c1(Vector3f(0,-2.1,0), 0.1, 15, mat2);  scene.addObject(c1);
    Cylinder c2(Vector3f(-2,0,2), 2, 1, mat1);  scene.addObject(c2);
  
    Vector3f v0(-4,-2,-2.5), v1(-4,-2,5.5),v2(4,-2,5.5),v3(4,-2,-2.5);
    Triangle t0(v0, v1, v2, mat2);scene.addObject(t0);
    Triangle t1(v2, v3, v0, mat2);scene.addObject(t1);


    Light light1(Vector3f(-5,10, -3),20,white); scene.addLight(light1);
    //Light light2(Vector3f(5,10,-2),5);
    
    //scene.addLight(light2);

    Vector3f bgColor(lightBlue);
    int size = 2;   //[1,3]
    int n_sampling = 1; //[1,100]

    Renderer renderer(bgColor, size, n_sampling);
    renderer.render(scene);

    renderer.write();
}


int main(){
    testScene();

    return 0;
}