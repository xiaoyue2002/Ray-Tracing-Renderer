
#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <random>
#include "vecmath.h"
#include "ray.hpp"
#include "hit.hpp"
#include "glut.h"
#include "texture.hpp"
#include <iostream>
#include "utils.hpp"

const int DIFF = 42;
const int SPEC = 43;
const int REFR = 44;


class Material {
public:

    void set_type(){
        if(Type != Vector3f::ZERO){
           // cout<<RND2<<endl;
            if(Type.x() > RND2)
                type = DIFF;
            else if(Type.x() + Type.y() > RND2)
                type = SPEC;
            else
                type = REFR;
        }
    }
    
    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, double s = 0,
                      const Vector3f &e_color = Vector3f::ZERO, double r = 1,Vector3f t = Vector3f(0,0,0),
                      const char *textureFile = "" , const char *bumpFile = "") :
            diffuseColor(d_color),
            specularColor(s_color),
            shininess(s),
            emission(e_color),
            refract(r),
            Type(t),
            texture(textureFile),
            bump(bumpFile){
                set_type();
            }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    double clamp(double x){
        if(x >= 0){
            return x;
        }else{
            return 0;
        }
    }
    
    int ReflectType(){
        return true;
    };
    
    Vector3f getColor(double u,double v){
        if(!texture.pic){
            return diffuseColor;
        }else{
            //cout<<"get texture color"<<endl;
            return texture.getColor(u,v);
        }
    }
    
    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f Lx = dirToLight.normalized();  //Lx:从相交处指向光源
        Vector3f N = hit.getNormal();           //N:法向量
        Vector3f V = -ray.getDirection().normalized();      //V
        Vector3f Rx = (2 * (Vector3f::dot(Lx, N)) * N - Lx).normalized();  //物体表面的反射光线方向
        Vector3f shaded = lightColor * (diffuseColor * clamp(Vector3f::dot(Lx,N)) + specularColor * (pow(clamp(Vector3f::dot(V,Rx)), shininess)));  //Phong模型公式
        return shaded;
    }

    // For OpenGL, this is fully implemented
    void Use() {
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Vector4f(diffuseColor, 1.0f));
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, Vector4f(specularColor, 1.0f));
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, Vector2f(shininess * 4.0, 1.0f));
    }
    
    /* ====== 材质的参数 ====== */
    Vector3f diffuseColor;          //颜色
    Vector3f specularColor;         //镜面反射系数
    Vector3f emission;              //发光系数
    double shininess;                //高光指数
    double refract;                  //折射率
    Vector3f Type;
    int type;                  //种类
    Texture texture,bump;
};


#endif // MATERIAL_H
