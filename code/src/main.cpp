//
//  main.cpp
//  Graphic_HW
//
//  Created by 徐潇悦 on 2022/6/22.
//

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include "scene_parser.hpp"
#include "raytrace.h"
#include "sppm.h"
#include <string>
using namespace std;

int main(int argc, const char * argv[]) {
    
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.
    
    SceneParser SP(inputFile.c_str());
    Camera * camera = SP.getCamera();
    Image image(camera->getWidth(),camera->getHeight());
    
//    SPPM sppm(camera,*SP.group,SP,outputFile);
//    sppm.Operate(image);
    PathTrace pt(camera,*SP.group,SP);
    pt.Operate(image);

    
    
    image.SaveImage(outputFile.c_str());
    
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
   
}
