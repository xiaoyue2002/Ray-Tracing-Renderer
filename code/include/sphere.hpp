#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include "vecmath.h"
#include "texture.hpp"
#include <cmath>
#include "glut.h"
using namespace std;

class Sphere : public Object3D {
public:
    Sphere() {
        radius = 0;
        center = Vector3f(0,0,0);
        // unit ball at the center
    }

    Sphere(const Vector3f &center, double radius, Material *material) : Object3D(material) {
        this->center = center;
        this->radius = radius;
        //cout<<"Sphere"<<material->emission<<" "<<material->type<<endl;
        //
    }

    ~Sphere() override = default;

    Vector3f getNormal(const Vector3f &n, const Vector3f &p, double u, double v) {
        Vector2f grad(0);
        double f = material->bump.getDisturb(u, v, grad);
        if (fabs(f) < FLT_EPSILON) return n;
        double phi = u * 2 * M_PI, theta = M_PI - v * M_PI;
        Vector3f pu(-p.z(), 0, p.x()),
            pv(p.y() * cos(phi), -radius * sin(theta), p.y() * sin(phi));
        if (pu.squaredLength() < FLT_EPSILON) return n;
        return Vector3f::cross(pu + n * grad[0] / (2 * M_PI),
                               pv + n * grad[1] / M_PI)
            .normalized();
    }
    
    bool intersect(const Ray &r, Hit &h, double tmin) override {
        /* 代数方法进行射线和球面求交 */
        Vector3f origin(r.getOrigin()), dir(r.getDirection());
        Vector3f OC = center - origin;
        /* b = Rd * (Ro - Pc)*/
        double b = -Vector3f::dot(OC, dir);
        /* c = (Ro - Pc)^2 - r^2 */
        double c = OC.squaredLength() - radius * radius;
        double delta = b * b - c;
        if (delta <= 0) return false;
        double sqrt_delta = sqrtf(delta);
        double t1 = (-b - sqrt_delta), t2 = (-b + sqrt_delta);
        double t;
        if (t1 <= h.getT() && t1 >= 0)
            t = t1;
        else if (t2 <= h.getT() && t2 >= 0)
            t = t2;
        else
            return false;
        Vector3f OP = origin + dir * t - center;
        Vector3f normal = OP.normalized();
        /* 带入公式得到u和v */
        double u = 0.5 + atan2(normal.x(), normal.z()) / (2 * M_PI),
              v = 0.5 - asin(normal.y()) / M_PI;
        
        h.set(t, material, getNormal(normal, OP, u, v),
             origin + dir * t , material->getColor(u, v));
        return true;
        
    }

    void drawGL() override {
        Object3D::drawGL();
        glMatrixMode(GL_MODELVIEW); glPushMatrix();
        glTranslatef(center.x(), center.y(), center.z());
        glutSolidSphere(radius, 80, 80);
        glPopMatrix();
    }

    Ray randomRay(int axis = -1,long long int seed = 0) override{
        double u = 2 * random(axis, seed) - 1, v = 2*random(axis, seed) - 1;
        double r2 =u * u + v * v;
        while(r2>=1) {
            ++seed;
            u = 2 * random(axis, seed) - 1;
            v = 2 * random(axis, seed) - 1;
            r2 = u * u + v * v;
        }
        Vector3f dir(2*u*sqrtf(1-r2), 2*v*sqrt(1-r2),1-2*r2);
        dir.normalize();
        return Ray(center + radius *dir, dir);
    }

protected:
    Vector3f center;
    double radius;

};


#endif

