#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "common/core_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    CORE_POINT2F top_left;
    CORE_POINT2F top_right;
    CORE_POINT2F bottom_right;
    CORE_POINT2F bottom_left;
} GEOMETRY_RECT2F;

CORE_POINT2F Geometry_Lerp2D(CORE_POINT2F start,
                             CORE_POINT2F end,
                             float ratio);
CORE_POINT2F Geometry_RectBilinearInterpolate(const GEOMETRY_RECT2F *rect,
                                              float u,
                                              float v);
CORE_POINT2F Geometry_PaperToRectPoint(CORE_POINT2F paper_point,
                                       float paper_width,
                                       float paper_height,
                                       const GEOMETRY_RECT2F *rect);
CORE_POINT2F Geometry_CirclePointDeg(CORE_POINT2F center,
                                     float radius,
                                     float angle_deg);
void Geometry_RectFromArray(const uint16_t rect_data[8], GEOMETRY_RECT2F *rect);

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRY_H */
