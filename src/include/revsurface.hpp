#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include "boundbox.h"
#include <tuple>

const int resolution = 10;
const int NEWTON_STEPS = 20;
const double NEWTON_EPSILON = 1e-4;


class RevSurface : public Object3D {
private:
    Curve *pCurve;
    AABB_BoundBox AABB;
    
public:
    RevSurface(Curve *pCurve, Material* material) : pCurve(pCurve), Object3D(material) {
        // Check flat.
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        AABB.reset(Vector3f(-pCurve->radius,pCurve->ymin - 3, -pCurve->radius),
                 Vector3f(pCurve->radius,pCurve->ymax + 1,pCurve->radius));
    }

    ~RevSurface() override {
        delete pCurve;
    }
    
    Vector3f getPoint(const double &theta,const float &mu,Vector3f &delta_theta,Vector3f &delta_mu){
        Vector3f point;
        Quat4f rotation;
        rotation.setAxisAngle(theta,Vector3f::UP);
        Matrix3f rotationMat = Matrix3f::rotation(rotation);
        CurvePoint curvepoint =pCurve->getPoint(mu);
        point = rotationMat * curvepoint.V;
        delta_mu = rotationMat * curvepoint.T;
        delta_theta = Vector3f(-curvepoint.V.x() * sin(theta), 0 , -curvepoint.V.x() * cos(theta));
        return point;
    }

    bool NewtonMethod(const Ray& ray,double &t,double &theta, double &mu,Vector3f &normal,Vector3f &point){
        Vector3f delta_mu, delta_theta;
        for(int i = 0 ; i < NEWTON_STEPS ; i++){
            if(theta < 0) theta = theta + 2 * M_PI;
            if(theta >= 2 * M_PI) theta = fmod(theta, 2*M_PI);
            if(mu >= 1) mu = 1.0 - FLT_EPSILON;
            if(mu <= 0) mu = FLT_EPSILON;
        
            point = getPoint(theta, mu, delta_theta ,delta_mu);
            Vector3f f = ray.origin + t * ray.direction - point;
            double dist2 = f.squaredLength();
            normal = Vector3f::cross(delta_mu , delta_theta);
            if(dist2 < NEWTON_EPSILON ) return true;
            double D = Vector3f::dot(ray.direction,normal);
            t = t - Vector3f::dot(delta_mu, Vector3f::cross(delta_theta,f)) / D;
            mu = mu - Vector3f::dot(ray.direction , Vector3f::cross(delta_theta,f)) / D;
            theta = theta + Vector3f::dot(ray.direction, Vector3f::cross(delta_mu,f)) / D;
        }
        return false;
    }
    
    bool intersect(const Ray &ray, Hit &h, double tmin) override {
        double t, theta, mu;
        /* 包围盒加速 */
        if(!AABB.intersect(ray, t) || t > h.getT()) return false;
        
        /* 以包围盒的交点作为迭代初值 */
        Vector3f position = ray.origin + ray.direction * t;
        
        theta = atan2(-position.z(), position.x()) + M_PI;
        mu = (pCurve->ymax - position.y()) / (pCurve->ymax -pCurve->ymin);
        
        Vector3f normal,point;
        
        /* 牛顿法求交无解 */
        if(!NewtonMethod(ray, t, theta, mu, normal, point)) return false;
        if(!isnormal(mu) || !isnormal(theta) || !isnormal(t)) return false;
        /* 不符合交点条件 */
        if(t < 0 || mu < pCurve->range[0] || mu > pCurve->range[1] || t>h.getT()) return false;
        
        normal = normal.normalized();
        h.set(t,material,normal,point,material->getColor(theta / (2 * M_PI), mu));
        return true;
    }

    void drawGL() override {
        Object3D::drawGL();

        // Definition for drawable surface.
        typedef std::tuple<unsigned, unsigned, unsigned> Tup3u;
        // Surface is just a struct that contains vertices, normals, and
        // faces.  VV[i] is the position of vertex i, and VN[i] is the normal
        // of vertex i.  A face is a triple i,j,k corresponding to a triangle
        // with (vertex i, normal i), (vertex j, normal j), ...
        // Currently this struct is computed every time when canvas refreshes.
        // You can store this as member function to accelerate rendering.

        struct Surface {
            std::vector<Vector3f> VV;
            std::vector<Vector3f> VN;
            std::vector<Tup3u> VF;
        } surface;

        std::vector<CurvePoint> curvePoints;
        pCurve->discretize(30, curvePoints);
        const int steps = 40;
        for (unsigned int ci = 0; ci < curvePoints.size(); ++ci) {
            const CurvePoint &cp = curvePoints[ci];
            for (unsigned int i = 0; i < steps; ++i) {
                double t = (double) i / steps;
                Quat4f rot;
                rot.setAxisAngle(t * 2 * 3.14159, Vector3f::UP);
                Vector3f pnew = Matrix3f::rotation(rot) * cp.V;
                Vector3f pNormal = Vector3f::cross(cp.T, -Vector3f::FORWARD);
                Vector3f nnew = Matrix3f::rotation(rot) * pNormal;
                surface.VV.push_back(pnew);
                surface.VN.push_back(nnew);
                int i1 = (i + 1 == steps) ? 0 : i + 1;
                if (ci != curvePoints.size() - 1) {
                    surface.VF.emplace_back((ci + 1) * steps + i, ci * steps + i1, ci * steps + i);
                    surface.VF.emplace_back((ci + 1) * steps + i, (ci + 1) * steps + i1, ci * steps + i1);
                }
            }
        }

        glBegin(GL_TRIANGLES);
        for (unsigned i = 0; i < surface.VF.size(); i++) {
            glNormal3fv(surface.VN[std::get<0>(surface.VF[i])]);
            glVertex3fv(surface.VV[std::get<0>(surface.VF[i])]);
            glNormal3fv(surface.VN[std::get<1>(surface.VF[i])]);
            glVertex3fv(surface.VV[std::get<1>(surface.VF[i])]);
            glNormal3fv(surface.VN[std::get<2>(surface.VF[i])]);
            glVertex3fv(surface.VV[std::get<2>(surface.VF[i])]);
        }
        glEnd();
    }
};

#endif //REVSURFACE_HPP
