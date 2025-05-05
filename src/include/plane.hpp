#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include "vecmath.h"
#include <cmath>

// TODO (PA2): Copy from PA1
// DONE!

class Plane : public Object3D {
public:
    Plane() {
        d = 0;
        norm = Vector3f::UP;
    }

    Plane(const Vector3f &normal, double d, Material *m) : Object3D(m) {
        this->norm = normal;
        this->d = d;
        this->material = m;
    }

    ~Plane() override = default;

    void getUV(double &u, double &v, const Vector3f &p) {
        v = p.y();
        Vector3f uaxis = Vector3f::cross(Vector3f::UP, norm);
        u = Vector3f::dot(p - d * norm, uaxis);
    }
    
    Vector3f getNormal(double u, double v) {
        Vector2f grad(0);
        Vector3f uaxis = Vector3f::cross(Vector3f::UP, norm);
        double f = material->bump.getDisturb(u, v, grad);
        if (fabs(f) < FLT_EPSILON) return norm;
        if (uaxis.squaredLength() < FLT_EPSILON) return norm;
        return Vector3f::cross(uaxis + norm * grad[0],
                               Vector3f::UP + norm * grad[1])
            .normalized();
    }
    
    bool intersect(const Ray &r, Hit &h, double tmin) override {
        Vector3f o(r.getOrigin()), dir(r.getDirection());
        // dir.normalize();
        Vector3f uaxis = Vector3f::cross(Vector3f::UP, norm);
        double cos = Vector3f::dot(norm, dir);
        // 平行
        if (cos > -1e-6) return false;
        // d = n.o + t*n.dir => t = (d-n.o)/(n.dir)
        double t = (d - Vector3f::dot(norm, o)) / cos;
        if (t < 0 || t > h.getT()) return false;
        double u, v;
        Vector3f p(o + dir * t);
        getUV(u, v, p);
        h.set(t, material, getNormal(u, v), p,material->getColor(u,v));
        return true;
        
        
//        Vector3f origin = r.getOrigin();
//        Vector3f direct = r.getDirection().normalized();
//
//        //用课件上的公式实现了t的计算
//        double t =  (d - Vector3f::dot(norm, origin)) / (Vector3f::dot(norm,direct));
//        if(t <= h.getT() && t>= tmin){
//            Vector3f pos = origin + t * direct;
//            h.set(t, material, norm, pos);
//            return true;
//        }
//        return false;
    }

    void drawGL() override {
        Object3D::drawGL();
        Vector3f xAxis = Vector3f::RIGHT;
        Vector3f yAxis = Vector3f::cross(norm, xAxis);
        xAxis = Vector3f::cross(yAxis, norm);
        const double planeSize = 10.0;
        glBegin(GL_TRIANGLES);
        glNormal3fv(norm);
        glVertex3fv(d * norm + planeSize * xAxis + planeSize * yAxis);
        glVertex3fv(d * norm - planeSize * xAxis - planeSize * yAxis);
        glVertex3fv(d * norm + planeSize * xAxis - planeSize * yAxis);
        glNormal3fv(norm);
        glVertex3fv(d * norm + planeSize * xAxis + planeSize * yAxis);
        glVertex3fv(d * norm - planeSize * xAxis + planeSize * yAxis);
        glVertex3fv(d * norm - planeSize * xAxis - planeSize * yAxis);
        glEnd();
    }


protected:
    Vector3f norm;
    double d;

};

#endif //PLANE_H
		

