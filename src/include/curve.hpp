#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include "vecmath.h"
#include <vector>
#include "limits.h"
#include <utility>
#include <cmath>
#include <algorithm>
using namespace std;
// TODO (PA2): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
};


int math_tool(int n , int i){
    if(i == 0)
        return 1;
    int a = 1;
    int b = 1;
    int num = i;
    for(int j = 0 ; j < num  ;j++){
        b =  b * i;
        a =  a * n;
        n = n - 1;
        i = i - 1;
    }
    return a / b;
}

class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
    
public:
    float radius;
    float ymax,ymin;
    vector<double> t;
    vector<double> tpad;
    double range[2];
    int n_,k_;
    
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {
        radius = 0;
        ymax = -INT_MAX;
        ymin = INT_MAX;
        for(auto point : controls){
            ymin = min(point.y(), ymin);
            ymax = max(point.y(), ymax);
            radius = max(radius, fabs(point.x()));
            radius = max(radius, fabs(point.z()));
        }
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }
    
    void generate_tpad(){
        int size = t.size();
        tpad.resize(size + k_);
        for(int i = 0 ; i < size ; i++) tpad[i] = t[i];
        for(int i = 0 ; i < k_ ; i++) tpad[i + size] = t.back();
    }
    
    CurvePoint getPoint(double mu){
        CurvePoint cp;
        //cout<<"12"<<endl;
        int bpos = upper_bound(t.begin(), t.end(), mu) - t.begin() - 1 ;
        vector<double> s(k_ + 2, 0), deltas(k_ + 1 ,1);
        s[k_] = 1;
        //cout<<"13"<<endl;
        for(int p = 1 ; p <= k_ ; p++){
            for(int ii = k_ - p ; ii < k_ + 1 ; ii++){
                int i = ii + bpos - k_;
                double w1, dw1, w2, dw2;
                //cout<<"15"<<endl;
                //cout<<"i"<<i<<endl;
                //cout<<"k"<<k_<<endl;
                //cout<<"size"<<tpad.size()<<endl;
                if(tpad[i + p] == tpad[i]){
                    w1 = mu;
                    dw1 = 1;
                    //cout<<"17"<<endl;
                }else{
                    w1 = (mu - tpad[i]) / (tpad[i + p] - tpad[i]);
                    dw1 = 1.0 / (tpad[i + p] - tpad[i]);
                    //cout<<"18"<<endl;
                }
                //cout<<"16"<<endl;
                if(tpad[i + p + 1] == tpad[i + 1]){
                    w2 = 1 - mu;
                    dw2 = -1;
                }else{
                    w2 = (tpad[i + p + 1] - mu) / (tpad[i + p + 1] - tpad[i+1]);
                    dw2 = -1 / (tpad[i + p + 1] - tpad[i + 1]);
                }
                //cout<<"17"<<endl;
                if(p == k_) deltas[ii] = (dw1 * s[ii] + dw2 *s[ii + 1]) * p;
                s[ii] = w1 * s[ii] + w2 * s[ii + 1];
            }
        }
        //cout<<"14"<<endl;
        s.pop_back();
        int lsk = k_ - bpos, rsk = bpos + 1 - n_;
        if (lsk > 0) {
            for (int i = lsk; i < s.size(); ++i) {
                s[i - lsk] = s[i];
                deltas[i - lsk] = deltas[i];
            }
            s.resize(s.size() - lsk);
            deltas.resize(deltas.size() - lsk);
            lsk = 0;
        }
        if (rsk > 0) {
            if (rsk < s.size()) {
                s.resize(s.size() - rsk);
                deltas.resize(deltas.size() - rsk);
            } else {
                s.clear();
                deltas.clear();
            }
        }
        for (int j = 0; j < s.size(); ++j) {
            cp.V += controls[-lsk + j] * s[j];
            cp.T += controls[-lsk + j] * deltas[j];
        }
        //cout<<"15"<<endl;
        return cp;
    }
    

    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

    void drawGL() override {
        Object3D::drawGL();
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 0);
        glBegin(GL_LINE_STRIP);
        for (auto & control : controls) { glVertex3fv(control); }
        glEnd();
        glPointSize(4);
        glBegin(GL_POINTS);
        for (auto & control : controls) { glVertex3fv(control); }
        glEnd();
        std::vector<CurvePoint> sampledPoints;
        discretize(30, sampledPoints);
        glColor3f(1, 1, 1);
        glBegin(GL_LINE_STRIP);
        for (auto & cp : sampledPoints) { glVertex3fv(cp.V); }
        glEnd();
        glPopAttrib();
    };
    
   

};

class BezierCurve : public Curve {
    
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
        int n = controls.size();
        n_ = n;
        k_ = n - 1;
        t.resize(2*n);
        for(int i = 0 ; i < n ; i++){
            t[i] = 0;
            t[i + n] = 1;
        }
        range[0] = 0;
        range[1] = 1;
        generate_tpad();
    }
    

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // TODO (PA2): fill in data vector
        int n = controls.size() - 1;                //n + 1 各控制点
        int k = n;
        resolution = resolution * (n+1) / n;
        data.resize(resolution);
        
        for (int t = 0; t < resolution; t++) {
            double t_input = double(t) / double(resolution);
            data[t].T = Vector3f::ZERO;
            data[t].V = Vector3f::ZERO;
            std::vector<double> b_t(n+1,0);
            std::vector<double> b_sec_t(n+2,0);
            std::vector<double> db_t(n+1,0);
            for(int i = 0 ; i < n+1 ; i++){
                if(t_input == 0){
                    if(i == 0){
                        b_t[i] = 1;
                    }else{
                        b_t[i] = 0;
                    }
                }else if(t_input == 1){
                    if(i == k){
                        b_t[i] = 1;
                    }else{
                        b_t[i] = 0;
                    }
                }else{
                    b_t[i] = double(math_tool(k,i)) * pow(1-t_input,k-i) * pow(t_input,i);
                }
            }
            
            db_t[0] =  - k * pow(1-t_input,k-1);
            db_t[n] = k * pow(t_input, k-1);
            for(int i = 1 ; i < n ; i++){
                double num_1 = (k-i) * pow(1-t_input,k-i-1) * pow(t_input , i);
                double num_2 = i * pow(1 - t_input , k-i) * pow(t_input , i - 1);
                db_t[i] = double(math_tool(k,i)) * (- num_1 + num_2 );
                
            }
            for(int i = 0 ; i < n+1 ; i++){
                data[t].V += controls[i] * b_t[i];
                data[t].T += controls[i] * db_t[i];
            }
        }
    }

};

class BsplineCurve : public Curve {
public:
    
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
        int n = controls.size();
        n_ = n;
        k_ = 3;
        int k = 3;
        t.resize(n + k + 1);
        for(int i = 0 ; i < n + k + 1; i++){
            t[i] = (double) i / (n + k);
        }
        generate_tpad();
        range[0] = t[k];
        range[1] = t[n];
        
    }
    
    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        
        int n = controls.size() - 1;        //定义n，控制点个数为 n + 1
        int k = 3;                          //定义k，约定为3
        
        double * knots = new double[n+k+2];   //定义B样条的knots
        
        for(int i = 0 ; i < n + k + 2 ; i++)
            knots[i] = double(i) / (n + k + 1); //计算knots
        
        resolution = resolution * n / (n+1) ;
        data.resize(resolution);
        
        for(int t = 0 ; t < resolution ; t++){       //需要计算的t值
            double t_input = double(t) / double(resolution);
            data[t].V = Vector3f::ZERO;
            data[t].T = Vector3f::ZERO;
            std::vector<double> b_t(n+k+1,0);
            std::vector<double> db_t(n+1,0);
            
            for(int i = 0 ; i < n+k+1 ; i++){
                if(t_input >= knots[i] && t_input < knots[i+1]){
                    b_t[i] = 1;
                }
            }                                         // 0 次幂系数
                            
            for(int p = 1; p <= k ; p++){
                std::vector<double> nb_t(n+k+1-p,0);
                for(int i = 0 ;i < (n + k + 1 - p); i++){
                    double weight_1 = double( t_input - knots[i]) / double(knots[i+p] - knots[i]);
                    double weight_2 = double(knots[i+p+1] - t_input) / double(knots[i+p+1] - knots[i+1]);
                    nb_t[i] = weight_1 * double(b_t[i]) + weight_2 * double(b_t[i+1]);
                }
               
                if(p == k){
                    for(int i = 0 ; i < (n + k + 1 - p) ; i++){
                        double num_1 = double(b_t[i]) / double(knots[i+p] - knots[i]);
                        double num_2 = double(b_t[i+1]) / double(knots[i+p+1] - knots[i+1]);
                        db_t[i] = double(p) * (num_1 - num_2);
                    }
                }
                b_t = nb_t;
            }
            
            for(int i = 0 ; i < n+1 ;i++){
                data[t].V += controls[i] * b_t[i];
                data[t].T += controls[i] * db_t[i];
            }
        }
        
    }

protected:

};

#endif // CURVE_HPP
