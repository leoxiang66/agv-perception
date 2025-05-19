#pragma once
#include "global.hpp"
#include "yolov8/yolov8.h"

void consume_img(YoloV8*, const YoloV8Config*);
void consume_pc2();