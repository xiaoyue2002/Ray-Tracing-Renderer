#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include "vecmath.h"
#include <cmath>
#include <iostream>
#include "utils.hpp"

using namespace std;

// TODO (PA2): Copy from PA1
// DONE!
class Triangle: public Object3D
{

public:
	Triangle() = delete;
        ///@param a b c are three vertex positions of the triangle

	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
        this->A = a;
        this->B = b;
        this->C = c;
        Vector3f AB = b - a;
        Vector3f AC = c - a;        //初始化并且计算法向量
        this->normal = Vector3f::cross(AB,AC).normalized();
        this->vertices[0] = this->A;
        this->vertices[1] = this->B;
        this->vertices[2] = this->C;
        //cout<<"material"<<material->emission<<endl;
	}
    
    bool IsIN(Vector3f P){          //用求叉积的办法判断，如果是沿一个方向转动，叉积同向
        Vector3f PA = A - P;
        Vector3f PB = B - P;
        Vector3f PC = C - P;
        Vector3f t1 = Vector3f::cross(PA,PB);
        Vector3f t2 = Vector3f::cross(PB,PC);
        Vector3f t3 = Vector3f::cross(PC,PA);
        
        if(Vector3f::dot(t1,t2) > 0 && Vector3f::dot(t2,t3) > 0 && Vector3f::dot(t1,t3)>0)
            return true;
       
        return false;
    }
    
    bool intersect( const Ray& r,  Hit& h , double tmin) override {
        
        Vector3f origin = r.getOrigin();
        Vector3f direct = r.getDirection();
        
        /* 利用Cramer法则判断射线和三角形是否相交*/
        Vector3f E1 = A - B;
        Vector3f E2 = A - C;
        Vector3f S = A - r.getOrigin();
        double t = Matrix3f(S, E1, E2).determinant() / Matrix3f(r.getDirection(), E1, E2).determinant();
        double beta = Matrix3f(r.getDirection(), S, E2).determinant() / Matrix3f(r.getDirection(), E1, E2).determinant();
        double gamma = Matrix3f(r.getDirection(), E1, S).determinant() / Matrix3f(r.getDirection(), E1, E2).determinant();
        if (beta < 0 || beta > 1 || gamma < 0 || gamma > 1 || beta + gamma > 1 || t < tmin || t > h.getT())
            return false;
        
        /* 三角插值计算 */
        Vector3f point(origin + direct * t);
        Vector3f va = (vertices[0] - point), vb = (vertices[1] - point), vc = (vertices[2] - point);
        double ra = Vector3f::cross(vb, vc).length();
        double rb = Vector3f::cross(vc, va).length();
        double rc = Vector3f::cross(va, vb).length();
        
        Vector3f N = normal;
        if(nSet) N = (ra * an + rb * bn + rc * cn).normalized();
        
        if (Vector3f::dot(normal, r.getDirection()) < 0)
            h.set(t, this->material, N, point,material->diffuseColor );
        else
            h.set(t, this->material, -N, point,material->diffuseColor );
        return true;
        
	}
    
    
	Vector3f normal;
	Vector3f vertices[3];

    void drawGL() override {
        Object3D::drawGL();
        glBegin(GL_TRIANGLES);
        glNormal3fv(normal);
        glVertex3fv(vertices[0]); glVertex3fv(vertices[1]); glVertex3fv(vertices[2]);
        glEnd();
    }

    Ray randomRay(int axis= -1, long long int seed=0) override {
        double r1 = random(axis, seed), r2 = random(axis, seed);
        if (r1 + r2 > 1) {
            r1 = 1 - r1;
            r2 = 1 - r2;
        }
        return Ray(r1 * B + r2 * C + (1 - r1 - r2) * A, diffDir(normal, axis, seed));
    }
    
    void SetVN(const Vector3f& a_, const Vector3f& b_ , const Vector3f & c_){
        an = a_ , bn = b_ , cn = c_;
        //cout<<"Set"<<endl;
        nSet = true;
    }
    

    Vector3f A;
    Vector3f B;
    Vector3f C;
    Vector3f an,bn,cn;
    bool nSet = false;
};

#endif //TRIANGLE_H
