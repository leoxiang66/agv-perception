#include "sensing/inference.hpp"
#include <iostream>

void consume_img()
{
    while (true) 
    {
        auto msg = img_chan.pop();
        // std::cout << "img" << std::endl;
        //todo: 2d object detection model inference
    }
}

void consume_pc2()
{
    while (true)
    {
        auto msg = pc2_chan.pop();
        // std::cout << "pc2" << std::endl;
        //todo: 3d object detection model inference
    }
    
}
