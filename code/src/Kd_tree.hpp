//
//  Kd_tree.hpp
//  Graphic_HW
//
//  Created by 徐潇悦 on 2022/6/22.
//

#ifndef Kd_tree_hpp
#define Kd_tree_hpp

#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include "Vector3f.h"
#include "hit.hpp"
#include "parameters.h"
using namespace std;

inline Vector3f ZextuTaiMin(Vector3f a,Vector3f b){
    return Vector3f(min(a.x(),b.x()),min(a.y(),b.y()),min(a.z(),b.z()));
}
inline Vector3f ZextuTaiMax(Vector3f a,Vector3f b){
    return Vector3f(max(a.x(),b.x()),max(a.y(),b.y()),max(a.z(),b.z()));
}


class KDTreeNode{
public:
    Hit * hit;                      // 节点记录的hit内容
    Vector3f min,max;               // 右边的最小值 & 左边的最大值
    double maxr;
    KDTreeNode *lc,*rc;             // 左右枝
    KDTreeNode() = default;
    KDTreeNode(Vector3f MIN,Vector3f MAX,double MAXR,Hit * HIT = NULL,KDTreeNode * LC = NULL,KDTreeNode * RC = NULL):hit(HIT),min(MIN),max(MAX),maxr(MAXR),lc(LC),rc(RC){};
};

class KDTree{
private:
    int size;                       //KDtree的节点数
    Hit ** ObservePoints;                    //记录KDTree包含的Hit
    
    /* === build():根据目前的ObservePoints坐标递归建树 === */
    KDTreeNode * build(int l,int r,int dimension_){
        /* 新建一个节点 */
        Vector3f INF = Vector3f(INT_MAX,INT_MAX,INT_MAX);
        KDTreeNode *p = new KDTreeNode(INF, -INF, 0);
        
        /* 找出[l,r]范围内xyz坐标的最大值、最小值 */
        for(int i = l ;i <= r; i++){
            p->min = ZextuTaiMin(p->min, ObservePoints[i]->position);
            p->max = ZextuTaiMax(p->max, ObservePoints[i]->position);
            p->maxr = max(p->maxr,ObservePoints[i]->r2);
        }

        int m = l + r >> 1;
        /* 根据dimension来进行排序 */
        if(dimension_ == 0)
            nth_element(ObservePoints + l, ObservePoints + m, ObservePoints + r + 1, compareX);
        else if (dimension_ == 1)
            nth_element(ObservePoints + l, ObservePoints + m, ObservePoints + r + 1, compareY);
        else
            nth_element(ObservePoints + l, ObservePoints + m, ObservePoints + r + 1, compareZ);
        
        /* 新建节点选择排序得到的中点 */
        p->hit = ObservePoints[m];
        
        /* 递归建树,知道所有节点纳入树中 */
        if (l <= m - 1)
            p->lc = build(l, m - 1, (dimension_ + 1) % 3);
        else
            p->lc = nullptr;
        if (m + 1 <= r)
            p->rc = build(m + 1, r, (dimension_ + 1) % 3);
        else
            p->rc = nullptr;
        return p;
    }
    
    void del(KDTreeNode *p) {
        if (p->lc) del(p->lc);
        if (p->rc) del(p->rc);
        delete p;
    }
    
public:
    
    KDTreeNode *root;
    KDTree(vector<Hit *> *ObservePoints) {
        //cout<<"buildtree"<<endl;
        size = ObservePoints->size();
        this->ObservePoints = new Hit *[size];
        for (int i = 0; i < size; ++i) this->ObservePoints[i] = (*ObservePoints)[i];
        root = build(0, size - 1, 0);
    }
    ~KDTree() {
        if (!root) return;
        del(root);
        delete[] ObservePoints;
    }

    void update(KDTreeNode *node, const Vector3f &photon,
                const Vector3f &decrease, const Vector3f &d) {
        if (!node) return;
        /* 查找位于自己附近的观察点 */
        double mind = 0, maxd = 0;
        if (photon.x() > node->max.x()){
            mind += (photon.x() - node->max.x()) * (photon.x() - node->max.x());
        }
        if (photon.x() < node->min.x()){
            mind += (node->min.x() - photon.x()) * (node->min.x() - photon.x());
        }
        if (photon.y() > node->max.y()){
            mind += (photon.y() - node->max.y()) * (photon.y() - node->max.y());
        }
        if (photon.y() < node->min.y()){
            mind += (node->min.y() - photon.y()) * (node->min.y() - photon.y());
        }
        if (photon.z() > node->max.z()){
            mind += (photon.z() - node->max.z()) * (photon.z() - node->max.z());
        }
        if (photon.z() < node->min.z()){
            mind += (node->min.z() - photon.z()) * (node->min.z() - photon.z());
        }
        if (mind > node->maxr) return;
        
        /* 如果找到自己位于其范围内的观察点 */
        if ((photon - node->hit->position).squaredLength() <= node->hit->r2) {
            Hit *hp = node->hit;
            /* 按 R_{i+1} = R_i * \sqrt{(N_i + \alpha * M_i)/(N_i + M_i)} 来更新R值 */
            /* r2是r^2所以不用根号计算 ; M_i = 1 */
            double dec_factor = (hp->n * ALPHA + ALPHA) / (hp->n * ALPHA + 1.);
            Vector3f dr = d - hp->normal * (2 * Vector3f::dot(d, hp->normal));
            hp->n++;
            hp->r2 = hp->r2 * dec_factor;
            /* tau_{i+1} = (tau_i + Phi_i) * (R_{i+1} / R_i)^2 */
            hp->flux = (hp->flux + hp->decrease * decrease) * dec_factor;
        }
        if (node->lc) update(node->lc, photon, decrease, d);
        if (node->rc) update(node->rc, photon, decrease, d);
        node->maxr = node->hit->r2;
        if (node->lc && node->lc->hit->r2 > node->maxr) node->maxr = node->lc->hit->r2;
        if (node->rc && node->rc->hit->r2 > node->maxr) node->maxr = node->rc->hit->r2;
    }
    
    static bool compareX(Hit *a, Hit *b) { return a->position.x() < b->position.x(); }
    static bool compareY(Hit *a, Hit *b) { return a->position.y() < b->position.y(); }
    static bool compareZ(Hit *a, Hit *b) { return a->position.z() < b->position.z(); }
};


#endif /* Kd_tree_hpp */
