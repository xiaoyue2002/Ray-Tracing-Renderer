#ifndef HIT_H
#define HIT_H

#include "vecmath.h"
#include "ray.hpp"
#include "parameters.h"
class Material;



class Hit {
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
        r2 = INIT_R2;
        decrease = Vector3f(1);
        fluxLight = Vector3f(0);
        flux = Vector3f(0);
        n = 0;
    }

    Hit(double _t, Material *m, const Vector3f &_n) {
        t = _t;
        material = m;
        normal = _n;
        r2 = INIT_R2;
        decrease = Vector3f(1);
        fluxLight = Vector3f(0);
        flux = Vector3f(0);
        this->n = 0;
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
    }

    // destructor
    ~Hit() = default;

    double getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    
    void set(double _t, Material *m, const Vector3f &n,const Vector3f& pos,const Vector3f color) {
        t = _t;
        material = m;
        normal = n;
        position = pos;
        material_color = color;
    }

    double t;
    double r2 ;
    Material *material;
    Vector3f normal;
    Vector3f position;
    Vector3f material_color;
    
    Vector3f decrease;
    Vector3f flux;
    Vector3f fluxLight;
    int n;
};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H
