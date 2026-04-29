/**
 * @file  SensorProc.c
 * @brief 传感信息处理与目标坐标换算实现
 */
#include "SensorProc.h"
#include "TrackingRuntime.h"
#include "VisionState.h"
#include <math.h>
#include "Initialize.h"

static Coordinate paper_corner_c[4];

Coordinate paper_to_camera(Coordinate paper){
    paper.x /= PAPERWIDE;
    paper.y /= PAPERHIGHT;
    paper_corner_c[3].x = Rect_Loc[0];
    paper_corner_c[3].y = Rect_Loc[1];
    paper_corner_c[2].x = Rect_Loc[2];
    paper_corner_c[2].y = Rect_Loc[3];
    paper_corner_c[1].x = Rect_Loc[4];
    paper_corner_c[1].y = Rect_Loc[5];
    paper_corner_c[0].x = Rect_Loc[6];
    paper_corner_c[0].y = Rect_Loc[7];

    float u = paper.x / PAPERWIDE;
    float v = paper.y / PAPERHIGHT;

    float top_x = (1.0f - u) * paper_corner_c[0].x + u * paper_corner_c[1].x;
    float top_y = (1.0f - u) * paper_corner_c[0].y + u * paper_corner_c[1].y;
    float bottom_x = (1.0f - u) * paper_corner_c[3].x + u * paper_corner_c[2].x;
    float bottom_y = (1.0f - u) * paper_corner_c[3].y + u * paper_corner_c[2].y;

    Coordinate camera_coord;
    camera_coord.x = (1.0f - v) * top_x + v * bottom_x;
    camera_coord.y = (1.0f - v) * top_y + v * bottom_y;

    return camera_coord;
}

Coordinate get_target_coordinate(){
    Coordinate target;
    float target_theta = (edge - 1) * 90.0f + sInedge;

    target.x = CIRCLERADIUS * cosf(DEG_TO_RAD(target_theta));
    target.y = CIRCLERADIUS * sinf(DEG_TO_RAD(target_theta));
    target.x += PAPERWIDE / 2.0f;
    target.y += PAPERHIGHT / 2.0f;

    return target;
}

float Grayscale_Num_To_Theta(int num){
    return RAD_TO_DEG(atan(-(num - (SENSOR_COUNT - 1) / 2.0f) * DisSensor / DisSensorToWheel));
}

float thetaGrayscale(){
    float theta = 0.0f;
    int sum = 0;

    for (int i = 0; i < SENSOR_COUNT; i++){
        if (Digital[i] == 0){
            theta += Grayscale_Num_To_Theta(i);
            sum++;
        }
    }

    if (sum == 0){
        return 0.0f;
    }

    return theta / sum;
}

bool Road_detect(int nummin, int nummax){
    int sum_ir_detect = 0;

    for (int i = 0; i < SENSOR_COUNT; i++){
        if (Digital[i] == 0){
            sum_ir_detect++;
        }
    }

    return (sum_ir_detect <= nummax && sum_ir_detect >= nummin);
}

bool half_Detect(void){
    return Road_detect(3, 6);
}

bool cross_Roads_Detect(void){
    return Road_detect(7, 8);
}

bool empty_Detect(void){
    return Road_detect(0, 0);
}

bool centerDetect(void){
    return Digital[3] == 0;
}
