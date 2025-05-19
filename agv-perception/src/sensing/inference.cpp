#include "sensing/inference.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>


void consume_img(YoloV8* model, const YoloV8Config* config)
{
    while (true)
    {
        auto msg = img_chan.pop();
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        auto cv_image = cv_ptr->image;

        // Run inference
        const auto objects = model->detectObjects(cv_image);
        // std::cout << "Detected " << objects.size() << " objects" << std::endl;

        // Print probability and rectangle for each object
        for (const auto &obj : objects)
        {
            std::cout << "Object: " << config->classNames[obj.label] << std::endl;
            std::cout << "Probability: " << obj.probability << std::endl;
            std::cout << "Rectangle: x=" << obj.rect.x
                      << ", y=" << obj.rect.y
                      << ", width=" << obj.rect.width
                      << ", height=" << obj.rect.height << std::endl;
        }
    }
}

void consume_pc2()
{
    while (true)
    {
        auto msg = pc2_chan.pop();
        // std::cout << "pc2" << std::endl;
        // todo: 3d object detection model inference
    }
}
