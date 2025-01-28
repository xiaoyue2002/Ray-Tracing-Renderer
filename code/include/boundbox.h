//
//  BoundBox.h
//  WorkingPlayground
//
//  Created by 徐潇悦 on 2022/6/28.
//

#ifndef BoundBox_h
#define BoundBox_h
#include <cmath>
#include "vecmath.h"
#include "limits.h"
#include "ray.hpp"
using namespace std;

class AABB_BoundBox{
private:
    /* 三个方向的最小值 */
    Vector3f MINs;
    /* 三个方向的最大值 */
    Vector3f MAXs;
    
public:
    AABB_BoundBox(){
        MINs = Vector3f(INT_MAX);
        MAXs = Vector3f(-INT_MAX);
    }
    
    AABB_BoundBox(const Vector3f& min,const Vector3f& max){
        MINs = min;
        MAXs = max;
    }
    
    void reset(const Vector3f& min,const Vector3f& max){
        MINs = min;
        MAXs = max;
    }
    
    /* 依次更新三个维度 */
    void update(const Vector3f& vec){
        for(int i = 0 ; i < 3 ; i++){
            MINs[i] = MINs[i] < vec[i] ? MINs[i] : vec[i];
            MAXs[i] = MAXs[i] > vec[i] ? MAXs[i] : vec[i];
        }
    }
    
    /* intersect里用到的function */
    Vector3f& get(int x){
        if(x == 0){
            return MINs;
        }
        return MAXs;
    }
    
    /* 计算光线是否和包围盒相交 */
    bool intersect(const Ray ray,double& t_min){
        
        Vector3f origin = ray.getOrigin();
        Vector3f direction = ray.getDirection();
        Vector3f inv_direction = 1 / direction;
        Vector3f signals = {static_cast<float>(direction.x() < 0), static_cast<float>(direction.y() < 0), static_cast<float>(direction.z() < 0) };
        t_min = INT_MAX;
        
        double tmin, tmax, tymin, tymax, tzmin, tzmax;
        /* 首先处理yz平面,判断射线和两个yz平面的相交情况 */
        /* 获取和包围盒各平面交点的tmin和tmax值 */
        tmin = (get(signals[0]).x() - origin.x()) * inv_direction.x();
        tmax = (get(1-signals[0]).x() - origin.x()) * inv_direction.x();
        
        /* 处理zx平面 */
        tymin = (get(signals[1]).y() - origin.y()) * inv_direction.y();
        tymax = (get(1-signals[1]).y() - origin.y()) * inv_direction.y();
        
        if(tmin > tymax || tmax < tymin) return false;
        /* 更新tmin和tmax */
        tmin = tmin > tymin ? tmin : tymin;
        tmax = tmax < tymax ? tmax : tymax;
        
        /* 处理xy平面 */

        tzmin = (get(signals[2]).z() - origin.z()) * inv_direction.z();
        tzmax = (get(1-signals[2]).z() - origin.z()) * inv_direction.z();
        if(tmin > tzmax || tmax < tzmin) return false;
        /* 更新tmin和tmax */
        tmin = tmin > tzmin ? tmin : tzmin;
        tmax = tmax < tzmax ? tmax : tzmax;
        t_min = tmin;
        return true;
    }
};

#endif /* BoundBox_h */
