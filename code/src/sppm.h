#ifndef sppm_h
#define sppm_h

#include <cmath>
#include "hit.hpp"
#include "render.h"
#include "Vector3f.h"
#include <stdlib.h>
#include <stdio.h>
#include <climits>
#include "image.hpp"
#include "object3d.hpp"
#include "utils.hpp"
#include "Kd_tree.hpp"
#include "parameters.h"
#include <ctime>
#include <vector>
using namespace std;


/*
    * ====== Stochastic Progressive Photon Mapping 随机渐进光子映射 ======
    * SPPM每次随机发送一些Observe Ray,用当前的观察点来更新整个像素的值
    * 伪代码：
    * for round in rounds:
    *   for pixel in image:
    *       RayTrace_GenerateObservePoints();
    *   Change_Observe_Points_2_KDTree();
    *   for photon in photon_nums:
    *       PhotonTrace_Update_data_with_observePoints();
    *   save_ckpt_image();
    *
    * 具体实现:
    * 光子半径 r_observe ：ObserveRay在 diffuse 表面收集r_observe半径内的光子，计算局部能量
    * 光子能量计算公式 : Li = (1/(n*pi*r_observe^2))*sum{p_e*p_fr}  -- p_e : 光子能量 ; p_fr : 辐射率
    * 像素内光子数 : N_{i+1} = N_i + alpha * M_i -- M_i : photons to add in this turn
    * 光子半径更新 : R_{i+1} = R_{i} * sqrt( (N_i + alpha * M_i) / (N_i + M_i) )
    * 多轮光子能量累积 : tao_n+m = (t_n + t_m)*((N + ALPHA*M) / (N + M))
    * 最终像素结果计算 : Final_color = tao / (pi * r * N)
*/

class SPPM : public Render{
private:
    vector<Hit*> ObservePoints;
    KDTree * tree;
    string outputfile;
    vector<Object3D*> illuminants;
    
public:
    
    SPPM(Camera* cam,Group& grp,SceneParser& sce,string od):Render(cam,grp,sce),outputfile(od){
        tree = nullptr;
       // cout<<"1"<<endl;
        for(int x = 0 ; x < camera->width ; x++){           //初始化ObservePoints
            for(int y = 0 ; y < camera->height; y++){
                ObservePoints.push_back(new Hit());
            }
        }
        illuminants = group.getIlluminant();                //设置光源
    };
    
    void SetTree() {
        if (tree) delete tree;
        this->tree = new KDTree(&ObservePoints);
    }
    
    void RayTrace_GenerateObservePoints(Ray ray,Hit& hit,int depth,unsigned short * seeds,Vector3f decrease){
        /*
         * hit : 当前(x,y)的观察点
         * depth : 递归深度
         * decrease : 随着递归深度减少的衰减参数
        */
        
        if(++depth > 10) return;              // 超过深度，停止递归
        if(decrease.max()<1e-3) return;       // 已经衰减到太少，停止递归
        
        hit.t = INT_MAX;
        if(!this->group.intersect(ray,hit,0)){
            hit.fluxLight = hit.fluxLight + hit.decrease * scene.getBackgroundColor();
            return;                           // 没有交点，用背景值更新,停止迭代
        }
        
        Vector3f intersection_point = ray.origin + ray.direction * hit.getT();   //相交位置
        Vector3f normal = hit.normal;                                            //交点处的法向量
        Vector3f normal_outside = Vector3f::dot(normal,ray.direction) < 0 ? normal: normal*(-1);
        
        Vector3f f = hit.material_color;      // 物体颜色
        
        hit.material->set_type();
        if(hit.material->type == DIFF){       // 漫反射，停止递归
            hit.decrease = decrease * f;      // 更新观察点的衰减值
            hit.fluxLight = hit.fluxLight + hit.decrease * hit.material->emission;    // 更新观察点的fluxLight
            return;
        }else if(hit.material->type == SPEC){ // 镜面反射
            Ray ray_2(intersection_point,(ray.direction - normal * 2 * Vector3f::dot(ray.direction,normal)));
            decrease = decrease * f;          // 随着递归深度更新衰减值
            RayTrace_GenerateObservePoints(ray_2, hit, depth, seeds, decrease);
        }else if(hit.material->type == REFR){ // 折射、反射
            decrease = decrease * f;          // 随着递归深度更新衰减值
            
            /* === 计算反射方向的 === */
            bool into = Vector3f::dot(normal , normal_outside) > 0 ;
            double refract_n = hit.material->refract;
            double refract_t = into ? 1/refract_n : refract_n;
            double cosine_1 =  Vector3f::dot(ray.direction,normal_outside);
            double cos2t  = 1.0 - refract_t * refract_t * (1.0 - cosine_1 * cosine_1);
            double nc = 1;
            double nt = refract_n;
            double nnt = refract_t;
            double ddn = cosine_1;
            Vector3f tdir = (ray.direction * nnt - normal * ((into? 1 : -1)*(ddn * nnt + sqrt(cos2t)))).normalized();
            Ray refRay(intersection_point,ray.direction - normal * 2 * Vector3f::dot(normal,ray.direction));
            double a = nc - nt , b = nc + nt;
            double R0 = a * a / (b * b);
            double c = 1 - (into ? -ddn : Vector3f::dot(tdir,normal));
            double Re = R0 + (1 - R0)*c*c*c*c*c;
            double P = 0.25 + 0.5 * Re;
            
            if(cos2t > 0 && R0 < RND2){
                RayTrace_GenerateObservePoints(Ray(intersection_point,tdir),hit,depth,seeds,decrease);
            }else{
                RayTrace_GenerateObservePoints(refRay,hit,depth,seeds,decrease);
            }
        }
    }
    
    void PhotonTrace(Ray ray,int depth,unsigned short * seeds,Vector3f energy){
        
        if(++depth > 10) return;                    // 超过深度，停止递归
        if(energy.max() < 1e-3) return;             // 衰减到太小，停止递归
        
        Hit hit;
        if(!group.intersect(ray, hit, 0)) return;   // 光子和场景没有交点，停止递归
        
        Vector3f intersection_point = ray.origin + ray.direction * hit.getT();   // 相交位置
        Vector3f normal = hit.normal;                                            // 交点处的法向量
        Vector3f normal_outside = Vector3f::dot(normal,ray.direction) < 0 ? normal: normal*(-1);
        Vector3f f = hit.material_color;
        
        hit.material->set_type();
            /* 漫反射 */
        if(hit.material->type == DIFF){             // 漫反射
            /* 根据击中的位置在KDTree中查找观察点并更新 */
            tree->update(tree->root,hit.position,energy,ray.direction);
            /* 随着递归深度更新衰减系数*/
            energy = energy * f;
            
            /* === 计算漫反射出射方向 === */
            double r1 = 2 * M_PI * erand48(seeds);
            double r2 = erand48(seeds);
            double r2s = sqrt(r2);
            Vector3f w = normal_outside;
            Vector3f u = ((fabs(w.x())>0.1 ? Vector3f(0,1,0):Vector3f(1,0,0))%w).normalized();
            Vector3f v = w % u;
            Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
            
            PhotonTrace(Ray(intersection_point,d), depth, seeds, energy);
        }else if(hit.material->type == SPEC){       // 镜面反射
            Ray ray_2(intersection_point,(ray.direction - normal * 2 * Vector3f::dot(ray.direction,normal)));
            energy = energy * f;                    // 随着递归深度更新衰减系数
            PhotonTrace(ray_2, depth, seeds, energy);
        }else if(hit.material->type == REFR){       // 反射 + 折射
            energy = energy * f;                    // 随着递归深度更新衰减系数
            
            /*  === 计算反射方向 === */
            bool into = Vector3f::dot(normal , normal_outside) > 0 ;
            double refract_n = hit.material->refract;
            double refract_t = into ? 1/refract_n : refract_n;
            double cosine_1 =  Vector3f::dot(ray.direction,normal_outside);
            double cos2t  = 1.0 - refract_t * refract_t * (1.0 - cosine_1 * cosine_1);
            double nc = 1, nt = refract_n,nnt = refract_t,ddn = cosine_1;
            Vector3f tdir = (ray.direction * nnt - normal * ((into? 1 : -1)*(ddn * nnt + sqrt(cos2t)))).normalized();
            Ray refRay(intersection_point,ray.direction - normal * 2 * Vector3f::dot(normal,ray.direction));
            double a = nt - nc ,b = nt + nc;
            double R0 = a * a / (b * b);
            double c = 1 - (into ? -ddn : Vector3f::dot(tdir,normal));
            double Re = R0 + (1 - R0)*c*c*c*c*c;
            
            if(cos2t > 0 && Re < RND2){
                PhotonTrace(Ray(intersection_point,tdir), depth, seeds, energy);
            }else{
                PhotonTrace(refRay, depth, seeds, energy);
            }
        }
    }
    
    void Operate(Image& image) override{
        
        /* === 设置画布 === */
        int width = image.Width();
        int height = image.Height();
        Vector3f cx = Vector3f(width * 0.5135 / height,0,0);
        Vector3f cy = (cx % camera->direction).normalized() * 0.5135;//0.5135;
        int nEmittedPhotons_per_light = nEmiittedPhotons / illuminants.size();

        time_t start = time(NULL);
        for(int i = 0 ; i < rounds ; i++){
            
            /* === 在命令行打印进程 ===*/
            double used_time = time(NULL)-start;
            double progress = (1.0 + i) / rounds;
            fprintf(stderr,"\rRendering (%d/%d Rounds) %5.2f%% Time: %.2f/%.2f sec\n",
                    i + 1, rounds, progress * 100., used_time,
                    used_time / progress);
            
            /* === 从相机发射观察射线，记录观察点 === */
#pragma omp parallel for schedule(dynamic, 1)
            for(int x = 0 ; x < width ; x++){
                for(int y = 0 ; y < height ; y++){
                   // fprintf(stderr,"\rRendering (%d) %5.2f%%",x,100.*y/(height-1));
                    unsigned short seeds[] = {0,0,static_cast<unsigned short>(i*x*(y+1))};
                    /* = 随机初始化射线 = */
                    double r1 = 2 * erand48(seeds);
                    double dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                    double r2 = 2 * erand48(seeds);
                    double dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                    double y_in = (0.5 + dy)/2 + y;
                    double x_in = (0.5 + dx)/2 + x;
                    
                    Ray camRay = camera->generateRay(Vector2f(x_in,y_in));
                    RayTrace_GenerateObservePoints(camRay,*ObservePoints[x * height + y],0,seeds,Vector3f(1,1,1));
                }
            }

            SetTree();

#pragma omp parallel for schedule(dynamic, 1)
            /* === 从光源发射光子，更新观察点 === */
            for (int x = 0; x < nEmittedPhotons_per_light ; x++) {
                for (int y = 0;y < illuminants.size(); y++) {
                    //fprintf(stderr,"\rRendering (%d) %5.2f%%",x,100.*y/(height-1));
                    unsigned short seeds[] = {0,0,static_cast<unsigned short>(y*y*y+x)};
                    /* 初始化射线，第二个参数是一个随机化种子 */
                    Ray ray = illuminants[y]->randomRay(-1, (long long)i * nEmiittedPhotons + (i + 1) * width * height + x);
                    PhotonTrace(ray, 0, seeds,  illuminants[y]->material->emission * Vector3f(250,250,250));
                }
            }
            
            if((i+1) % interations_ckpt == 0){
                Image img(width, height);
                for (int x = 0; x < width; x++)
                    for (int y = 0; y < height; y++) {
                        Hit* hit = ObservePoints[x * height + y];
                        img.SetPixel(
                            x, y,
                            hit->flux / (M_PI * hit->r2 * ( i + 1 ) * nEmiittedPhotons) +
                                hit->fluxLight / (i + 1));
                    }
                img.SaveBMP(outputfile.c_str());
            }
        }
        //img.SaveBMP(outputfile.c_str());
        cout<<"finish"<<endl;
    }
    

};


#endif
