#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include "vecmath.h"
#include "vecio.h"
#include "float.h"
#include <cmath>
#include "glut.h"
#include "utils.hpp"
using namespace std;

class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up);
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point){return Ray(Vector3f(),Vector3f());};
    
    virtual void setupGLMatrix() {
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();
        gluLookAt(center.x(), center.y(), center.z(),   // Position
                  center.x() + direction.x(), center.y() + direction.y(), center.z() + direction.z(),   // LookAt
                  up.x(), up.y(), up.z());    // Up direction
    }

    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

    void setCenter(const Vector3f& pos) {
        this->center = pos;
    }
    Vector3f getCenter() const {
        return this->center;
    }

    void setRotation(const Matrix3f& mat) {
        this->horizontal = mat.getCol(0);
        this->up = -mat.getCol(1);
        this->direction = mat.getCol(2);
    }
    Matrix3f getRotation() const {
        return Matrix3f(this->horizontal, -this->up, this->direction);
    }

    virtual void resize(int w, int h) {
        width = w; height = h;
    }

    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
    
    double len_radius;
    
    
    
};

class PerspectiveCamera : public Camera {
public:
    ~PerspectiveCamera() override = default;
    double getFovy() const { return fovyd; }

    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, double angle) : Camera(center, direction, up, imgW, imgH) {
        // angle is fovy in radian.
        fovyd = angle / 3.1415 * 180.0;
        fx = fy = (double) height / (2 * tanf(angle / 2));
        cx = width / 2.0f;
        cy = height / 2.0f;
    }

    void resize(int w, int h) override {
        fx *= (double) h / height;
        fy = fx;
        Camera::resize(w, h);
        cx = width / 2.0f;
        cy = height / 2.0f;
    }

    
    
    Ray generateRay(const Vector2f &point) override {

        /* 点到透镜的距离 */
        float image_to_lens_x = focal_length * (point[0] - cx) / fx;
        float image_to_lens_y = focal_length * (point[1] - cy) / fy;
        /* 在透镜平面上sample出一个点*/
        float lens_x= RND1 * aperture;
        float lens_y = RND1 * aperture;
        /* 射线方向 */
        Vector3f dir(image_to_lens_x - lens_x, -image_to_lens_y - lens_y, focal_length);
        /* 和之前相同的方法生成方向 */
        Matrix3f matrix(horizontal, -up, direction);
        dir = (matrix * dir).normalized();
        
        Ray ray(center + horizontal * lens_x - up * lens_y , dir);
        return ray;
        
    }

    void setupGLMatrix() override {
        // Extrinsic.
        Camera::setupGLMatrix();
        // Perspective Intrinsic.
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        // field of view in Y, aspect ratio, near crop and far crop.
        gluPerspective(fovyd, cx / cy, 0.01, 100.0);
    }
private:
    
    // Perspective intrinsics
    double fx;
    double fy;
    double cx;
    double cy;
    double fovyd;
    
    double focal_length = 1;   // 焦距
    double aperture = 0;        // 光圈
};


#endif //CAMERA_H
