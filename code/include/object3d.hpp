#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"
#include "glut.h"

// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr) {}

    virtual ~Object3D() = default;

    explicit Object3D(Material *material) {
        this->material = material;
    }

    virtual bool intersect(const Ray &r, Hit &h, double tmin) = 0;

    // PA2: draw using OpenGL pipeline.
    virtual void drawGL() {
        if (material) material->Use();
    }

    virtual Ray randomRay(int axis = -1, long long int seed = 0){
        return Ray(Vector3f(0),Vector3f(0));
    };
    Material *material;
};

#endif

