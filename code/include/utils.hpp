#ifndef UTILS_H
#define UTILS_H
#include <cmath>
#include <cstdlib>
#include <random>

#include <vecmath.h>
#define PI 3.1415926536
const int prime[] = {
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29,
    31, 37, 41, 43, 47, 53, 59, 61, 67, 71,
    73, 79, 83, 89, 97, 101, 103, 107, 109, 113,127,131, 137, 139, 149, 151, 157, 163, 167, 173,179, 181, 191, 193, 197, 199, 211, 223, 227, 229};
// Helpers for random number generation
static std::mt19937 mersenneTwister;
static std::uniform_real_distribution<double> uniform;
#define RND1 (2.0 * uniform(mersenneTwister) - 1.0)
#define RND2 (uniform(mersenneTwister))


inline void ons(const Vector3f &v1, Vector3f &v2, Vector3f &v3)
{
    if (std::abs(v1.x()) > std::abs(v1.y()))
    {
        double invLen = 1.f / sqrtf(v1.x() * v1.x() + v1.z() * v1.z());
        v2 = Vector3f(-v1.z() * invLen, 0.0f, v1.x() * invLen);
    }
    else
    {
        double invLen = 1.0f / sqrtf(v1.y() * v1.y() + v1.z() * v1.z());
        v2 = Vector3f(0.0f, v1.z() * invLen, -v1.y() * invLen);
    }
    v3 = Vector3f::cross(v1, v2);
}

inline Vector3f hemisphere(double u1, double u2)
{
    const double r = sqrt(1.0 - u1 * u1);
    const double phi = 2 * PI * u2;
    return Vector3f(cos(phi) * r, sin(phi) * r, u1);
}

inline Vector3f cosineHemisphere(double u1, double u2)
{
    const double r = sqrt(u1);
    const double theta = 2 * PI * u2;

    const double x = r * cos(theta);
    const double y = r * sin(theta);

    return Vector3f(x, y, sqrt(std::max(0.0, 1 - u1)));
}

inline double randomQMC(int axis, long long int seed) {
    int base = prime[axis];
    double f = 1, res = 0;
    while (seed > 0) {
        f /= base;
        res += f * (seed % base);
        seed /= base;
    }
    return res;
}

inline double random(int axis=-1, long long int seed=0) {
    if (axis == -1) return RND2;
    return randomQMC(axis, seed);
}

inline Vector3f diffDir(const Vector3f &norm, int depth=0, long long int seed=0)
{
    Vector3f rotX, rotY;
    ons(norm, rotX, rotY);
    return Matrix3f(rotX, rotY, norm) * cosineHemisphere(random(2*depth+1, seed), random(2*depth+2, seed));
}

#endif // !UTILS_H
