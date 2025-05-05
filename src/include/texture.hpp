//
//  texture.h
//  Graphic_HW
//
//  Created by 徐潇悦 on 2022/6/24.
//

#ifndef texture_h
#define texture_h

#include "vecmath.h"
using namespace std;

struct Texture {  // 纹理
    unsigned char *pic;
    int w, h, c;
    Texture(const char *textureFile);
    Vector3f getColor(double u, double v) const;
    Vector3f getColor(int idx) const;
    Vector3f getColor(int u, int v) const;
    double getDisturb(double u, double v, Vector2f &grad) const;
    inline int getIdx(double u, double v) const;
    inline double getGray(int idx) const;
};



#endif /* texture_h */
