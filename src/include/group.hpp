#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>
using namespace std;

// TODO (PA2): Implement Group - copy from PA1
// DONE! - copy


class Group : public Object3D {

public:

    Group() {
        count = 0;
    }

    explicit Group (int num_objects) {
        count = num_objects;
    }

    ~Group() override {

    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        bool mark = false;
        for(int i = 0; i < count; i++){     //用count遍历所有的物体
            if(ObjectList.at(i)->intersect(r,h,tmin)){
                mark = true;
            }
        }
        return mark;
    }

    void drawGL() override {
           for (auto *obj : ObjectList) {
               obj->drawGL();
           }
       }

    void addObject(int index, Object3D *obj) {
        count = index+1;
        ObjectList.push_back(obj);          //push back
    }

    int getGroupSize() {
        return ObjectList.size();
    }

    vector<Object3D *> getIlluminant() const {
        vector<Object3D *> illuminant;
        for (int i = 0; i < count; i++){
       // cout<<i<<endl;
           // cout<<"count"<<count<<endl;
            if (ObjectList[i]->material->emission != Vector3f::ZERO)
                illuminant.push_back(ObjectList[i]);
        }
        return illuminant;
    }
    
    int count;
    
    vector<Object3D *> ObjectList;     //用vector存储地址
};

#endif
	
