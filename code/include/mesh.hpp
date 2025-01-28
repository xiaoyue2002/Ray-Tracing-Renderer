#ifndef MESH_H
#define MESH_H

#include <vector>
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"
#include "boundbox.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>
#include "string.h"
using namespace std;

class Mesh : public Object3D {
public:
    AABB_BoundBox AABB;
    
    Mesh(const char *filename, Material *m){
        material = m;
        // Optional: Use tiny obj loader to replace this simple one.
        vector<TriangleIndex> vIdx, tIdx, nIdx;
        vector<Vector3f> v, vn;
        vector<Vector2f> vt;
        std::ifstream f;
        
        //cout<<"?"<<endl;
        f.open(filename);
        if (!f.is_open()) {
            std::cout << "Cannot open " << filename << "\n";
            return;
        }
        int len = strlen(filename);
        if(strcmp(".obj",filename + len - 4) == 0){
            std::string line;
            std::string vTok("v");
            std::string fTok("f");
            std::string vnTok("vn");
            std::string texTok("vt");
            string bslash = "/", space = " ";
            std::string tok;
            int texID;
            while (true) {
                std::getline(f, line);
                if (f.eof()) {
                    break;
                }
                if (line.size() < 3) {
                    continue;
                }
                if (line.at(0) == '#') {
                    continue;
                }
                std::stringstream ss(line);
                ss >> tok;
                if (tok == vTok) {
                    Vector3f vec;
                    ss >> vec[0] >> vec[1] >> vec[2];
                    v.push_back(vec);
                    //cout<<vec<<endl;
                    AABB.update(vec);
                } else if (tok == fTok) {
                    TriangleIndex vid, tid, nid;
                    for(int i = 0 ; i < 3 ; i++){
                        std::string str;
                        ss>>str;
                        vector<string> id = mysplit(str,bslash);
                        vid[i] = atoi(id[0].c_str()) - 1;
                        if(id.size() > 1){
                            tid[i] = atoi(id[1].c_str()) -1;
                        }
                        if (id.size() > 2) {
                            nid[i] = atoi(id[2].c_str()) - 1;
                        }
                    }
                    vIdx.push_back(vid);
                    tIdx.push_back(tid);
                    nIdx.push_back(nid);
                } else if (tok == texTok) {
                    Vector2f texcoord;
                    ss >> texcoord[0];
                    ss >> texcoord[1];
                    vt.push_back(texcoord);
                } else if (tok == vnTok){
                    Vector3f vec;
                    ss >> vec[0] >> vec[1] >> vec[2];
                    vn.push_back(vec);
                }
            }
        }

        computeNormal();

        f.close();
        
        //cout<<v.size()<<"hi"<<endl;
        //cout<<"vIdx"<<vIdx.size()<<endl;
        for (int triId = 0; triId < (int) vIdx.size(); ++triId) {
            //cout<<"hello?"<<endl;
            TriangleIndex &vIndex = vIdx[triId];
            triangles.push_back((Object3D *)new Triangle(
                v[vIndex[0]], v[vIndex[1]], v[vIndex[2]], m));
            //cout<<((Triangle*)triangles.back())->A<<endl;
            // if (tIdx.size()) {
            
            TriangleIndex &nIndex = nIdx[triId];
            if (nIndex.valid()){
                //cout<<"nIdex"<<nIndex.x[0]<<nIndex.x[1]<<endl;
                ((Triangle *)triangles.back())
                    ->SetVN(vn[nIndex[0]], vn[nIndex[1]], vn[nIndex[2]]);
            }
        }
    }

    vector<string> mysplit(std::string str, std::string pattern) {
        string::size_type pos;
        std::vector<std::string> result;
        str += pattern;
        int size = str.size();

        for (int i = 0; i < size; i++) {
            pos = str.find(pattern, i);
            if (pos < size) {
                std::string s = str.substr(i, pos - i);
                result.push_back(s);
                i = pos + pattern.size() - 1;
            }
        }
        return result;
    }
    
    static std::vector<std::string> split(char *str) {
        std::vector<std::string> ret;
        std::string cur = "";
        for (int i = 0; str[i]; ++i) {
            if (str[i] == ' ' || str[i] == '\n' || str[i] == '\r') {
                if (cur != "") ret.emplace_back(cur);
                cur = "";
            }
            else cur += str[i];
        }
        if (cur != "") ret.emplace_back(cur);
        return ret;
    }
    
    struct TriangleIndex {
        TriangleIndex() {
            x[0] = -1; x[1] = -1; x[2] = -1;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
        bool valid() { return x[0] != -1 && x[1] != -1 && x[2] != -1; }
    };
    

    bool intersect(const Ray &r, Hit &h, double tmin) override{
        // Optional: Change this brute force method into a faster one.
        bool result = false;
        /* 增加了包围盒加速 */
        double tb;
        if(!AABB.intersect(r,tb)) return false;
        if(tb > h.getT()) return false;

        for(auto triangle : triangles){
            result |= triangle->intersect(r, h, tmin);
           // cout<<result<<"A"<<((Triangle *)triangle)->A<<"B"<<((Triangle *)triangle)->B<<endl;
        }
        return result;
    }

    void drawGL() override {
        Object3D::drawGL();
        
        for (int triId = 0; triId < (int) t.size(); ++triId) {
            TriangleIndex& triIndex = t[triId];
            Triangle triangle(v[triIndex[0]],
                              v[triIndex[1]], v[triIndex[2]], material);
            triangle.normal = n[triId];
            triangle.drawGL();
        }
        
    }

    /* 线性查找 */
    //cout<<"tri"<<triangles.size()<<endl;
//        for (int triId = 0; triId < (int)t.size(); ++triId) {
//            //cout<<"hello";
//            TriangleIndex& triIndex = t[triId];
//            Triangle triangle(v[triIndex[0]],
//                              v[triIndex[1]], v[triIndex[2]], material);
//            triangle.normal = n[triId];
//            result |= triangle.intersect(r, h, tmin);
//        }
    //cout<<triangles.size()<<endl;
    
    vector<Object3D *> triangles;
    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    std::vector<Vector3f> n;

private:

    // Normal can be used for light estimation
    void computeNormal(){
        n.resize(t.size());
        for (int triId = 0; triId < (int) t.size(); ++triId) {
            TriangleIndex& triIndex = t[triId];
            Vector3f a = v[triIndex[1]] - v[triIndex[0]];
            Vector3f b = v[triIndex[2]] - v[triIndex[0]];
            b = Vector3f::cross(a, b);
            n[triId] = b / b.length();
        }
    }
};

#endif

