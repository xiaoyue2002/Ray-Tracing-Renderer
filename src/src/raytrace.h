//
//  raytrace.h
//  Graphic_HW
//
//  Created by 徐潇悦 on 2022/6/24.
//

#ifndef raytrace_h
#define raytrace_h

//#include <omp.h>
#include <cmath>
#include "hit.hpp"
#include "render.h"
#include "Vector3f.h"
#include <stdlib.h>
#include <stdio.h>
#include "object3d.hpp"
#include "utils.hpp"
using namespace std;

const int samps = 2;

const double E_MATH = 2.71828;
const Vector3f frog_color(0.2,0.2,0.2);
inline double f_atmo(double distance){
    return pow(E_MATH, -(0.01 * 0.01 * distance * distance));
}
inline double clamp(double x){return x < 0 ? 0 : x > 1 ? 1 : x;};
inline int toInt(double x){ return int(pow(clamp(x),1/2.2)*255+.5); }

class PathTrace:public Render{
public:
    PathTrace( Camera* cam,Group& grp,SceneParser& sce):Render(cam,grp,sce){};
    
    Vector3f radiance(Ray ray,int depth,unsigned short *seeds,int E = 1){
        /*
         * return : 亮度估计 Vector3f
         * ray : 正在传播的光线
         * depth : 递归深度
         * seeds : 随机数种子
         * E : 是否考虑自发光颜色
         */
        
        Hit hit;
        /* 如果超过深度，直接返回 */
        
        if(depth > 10) return Vector3f();
        /* 如果没有相交，更新为背景颜色，直接返回 */
        if(!this->group.intersect(ray,hit,0)){ return Vector3f();}

        /* 交点位置 */
        Vector3f intersection_point = ray.origin + ray.direction * hit.getT();
        /* 交点处的法向量 */
        Vector3f normal = hit.normal;
        /* 朝向物体外部的法向量 */
        Vector3f normal_outside = Vector3f::dot(normal,ray.direction) < 0 ? normal: normal*(-1);
        /* 物体颜色 */
        Vector3f f = hit.material_color;
        bool trace = false;
        if(f != Vector3f(0.9,0.9,0.9) && f != Vector3f(0.25,0.25,0.75) && f != Vector3f(0.75,0.25,0.25) && f != Vector3f(0.25,0.25,0.75)) trace = 0;
        
        /* 计算当前物体颜色的最大值 */
        /* 俄罗斯轮盘赌 : 当递归深度较深的时候,以p概率减少当前光线的亮度，以1-p概率停止递归 */
        double p = f.x()>f.y() && f.x()>f.z() ? f.x() : f.y()>f.z() ? f.y() : f.z();
        if(++depth > 5){
            if(erand48(seeds) < p){
                f = f * (1/p);
            }else
                return hit.material->emission;
        }
        
        hit.material->set_type();
        /* 漫反射、镜面反射、折射 */
        if(hit.material->type == DIFF){
            /* 漫反射，随机sample光线 */
            double r1 = 2 * M_PI * erand48(seeds);
            double r2 = erand48(seeds);
            double r2s = sqrt(r2);
            Vector3f w = normal_outside;
            Vector3f u = ((fabs(w.x())>0.1 ? Vector3f(0,1,0):Vector3f(1,0,0))%w).normalized();
            Vector3f v = w % u;
            Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
            /* 返回值是自己的亮度 + 发射到物体的射线的强度 * 物体的颜色 */
            
            Vector3f debug =  hit.material->emission + f * radiance(Ray(intersection_point,d), depth, seeds);
            
//            if(depth <= 1){
//                double distance = (hit.position - camera->center).length();
//                double fTemp = f_atmo(distance);
//                debug = hit.material->emission + fTemp * radiance(Ray(intersection_point,d), depth, seeds) + frog_color * (1 - fTemp);
//            }
            
            return debug;
        }else if(hit.material->type == SPEC){
            /* 按照公式计算出射光线 ray_2 */
            Ray ray_2(intersection_point,(ray.direction - normal * 2 * Vector3f::dot(ray.direction,normal)));
//            if(depth <= 1){
//                double distance = (hit.position - camera->center).length();
//                double fTemp = f_atmo(distance);
//                return hit.material->emission + fTemp * radiance(ray_2, depth, seeds) + frog_color * (1 - fTemp);
//            }
            return  hit.material->emission + f * radiance(ray_2, depth, seeds);
        }else if(hit.material->type == REFR){
            /* 折射，判断光线是否是摄入物体内部的 */
            bool into = Vector3f::dot(normal , normal_outside) > 0 ;
            /* 折射系数 */
            double refract_n = hit.material->refract;
            /* 根据光线方向确定使用空气到物体还是物体到空气的折射率 */
            double refract_t = into ? 1/refract_n : refract_n;
            /* 根据公式计算中间参数 */
            double cosine_1 =  Vector3f::dot(ray.direction,normal_outside);
            double cos2t  = 1.0 - refract_t * refract_t * (1.0 - cosine_1 * cosine_1);
            double nc = 1, nt = refract_n,nnt = refract_t,ddn = cosine_1;
            /* 折射光线 */
            Vector3f tdir = (ray.direction * nnt - normal * ((into? 1 : -1)*(ddn * nnt + sqrt(cos2t)))).normalized();
            /* 反射光线 */
            Ray refRay(intersection_point,ray.direction - normal * 2 * Vector3f::dot(normal,ray.direction));
            double a = nt - nc ,b = nt + nc;
            double R0 = a * a / (b * b);
            double c = 1 - (into ? -ddn : Vector3f::dot(tdir,normal));
            double Re = R0 + (1 - R0)*c*c*c*c*c;
            double Tr = 1 - Re;
            double P = 0.25 + 0.5 * Re;
            double RP = Re / P;
            double TP = Tr / (1 - P);
            
            if(cos2t < 0){
                /* 内部的反射 */
                return hit.material->emission + f * radiance(refRay,depth,seeds);
            }
            /* 折射、反射，俄罗斯轮盘赌 */

//            if(depth <= 1){
//                double distance = (hit.position - camera->center).length();
//                double fTemp = f_atmo(distance);
//                return hit.material->emission + frog_color * (1 - fTemp) + fTemp * (depth > 2 ? (erand48(seeds) < P ? radiance(refRay,depth,seeds)*RP : radiance(Ray(intersection_point,tdir),depth,seeds)*TP):
//                            radiance(refRay,depth,seeds) * Re + radiance(Ray(intersection_point,tdir),depth,seeds)*Tr);
//            }
            
            return hit.material->emission + f * (depth > 2 ? (erand48(seeds) < P ?
                radiance(refRay,depth,seeds)*RP : radiance(Ray(intersection_point,tdir),depth,seeds)*TP):
            radiance(refRay,depth,seeds) * Re + radiance(Ray(intersection_point,tdir),depth,seeds)*Tr);
            
            

        }
        
        cout<<"no type"<<endl;
        return Vector3f();
    }
    
    void Operate(Image &image) override{
        int height = image.Height();
        int width = image.Width();
        Vector3f r;
        /* 每个像素点采样 4 * samps次 */
        
        
        //cout<<"camera"<<camera->center<<" "<<camera->direction<<endl;
        
        int y = 0;
        /* ====== 遍历image plane 上的所有像素点 ====== */
#pragma omp parallel for schedule(dynamic, 1) num_threads(64) private(r) //OpenMP
        for( y = 0 ; y < height; y++){
            fprintf(stderr,"\rRendering (%d spp) %5.2f%%",samps*4,100.*y/(height-1));
            unsigned short seeds[3] = {0,0,(unsigned short)(y*y*y)};
            for(int x = width - 1 ; x >= 0 ; x--){
                /* === subpixel 用来抗锯齿 === */
                for(int sub_y = 0; sub_y < 2 ;sub_y++){
                    for(int sub_x = 0 ; sub_x < 2 ; sub_x++){
                        r = Vector3f();
                        for(int s = 0; s< samps; s++){
                            /* Tent filter */
                            double r1 = 2 * erand48(seeds);
                            double dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                            double r2 = 2 * erand48(seeds);
                            double dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                            double y_in = (sub_y + 0.5 + dy)/2 + y;
                            double x_in = (sub_x + 0.5 + dx)/2 + x;
                            Ray camRay = camera->generateRay(Vector2f(x_in,y_in));
                            
                            /* add 是在这个像素点上增加的color */
                            Vector3f add = radiance(camRay,0,seeds);
                            // cout<<"add"<<add<<endl;
                            r = r + add;
                        }
                        /* 对 samps 求和 */
                        r = r / samps;
                        Vector3f old_color = image.GetPixel(x, y);
                        old_color = old_color + Vector3f(clamp(r.x()),clamp(r.y()),clamp(r.z())) * 0.25;
                        image.SetPixel(x, y, old_color);
                    }
                }
            }
        }
    }
    
};

#endif /* raytrace_h */
