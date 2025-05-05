//
//  render.h
//  Graphic_HW
//
//  Created by 徐潇悦 on 2022/6/24.
//

#ifndef render_h
#define render_h
#include "camera.hpp"
#include "image.hpp"
#include "group.hpp"
using namespace std;

class Render{
public:
    Render(Camera* cam,Group& grp,SceneParser& sce):camera(cam),group(grp),scene(sce){};
    
    virtual void Operate(Image &image)=0;
    
protected:
    
    SceneParser& scene;
    Camera* camera;
    Group& group;

};


#endif /* render_h */
