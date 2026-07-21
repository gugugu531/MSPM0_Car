#include "kinematics.h"

#include <math.h>
#include <stddef.h>

static float Kinematics_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

float Kinematics_Clamp(float value, float min_value, float max_value){
    if (min_value > max_value){
        float temp = min_value;
        min_value = max_value;
        max_value = temp;
    }

    if (value > max_value){
        return max_value;
    }

    if (value < min_value){
        return min_value;
    }

    return value;
}

float Kinematics_NormalizeAngleDeg(float angle_deg){
    while (angle_deg >= 180.0f){
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f){
        angle_deg += 360.0f;
    }

    return angle_deg;
}

float Kinematics_AngleDiffDeg(float target_deg, float current_deg){
    return Kinematics_NormalizeAngleDeg(target_deg - current_deg);
}

KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward,
                                                          float turn,
                                                          float output_limit){
    KINEMATICS_DIFFERENTIAL_OUTPUT output = {
        .left = forward + turn,
        .right = forward - turn,
    };

    if (output_limit <= 0.0f){
        return output;
    }

    float max_abs = Kinematics_Abs(output.left);
    float right_abs = Kinematics_Abs(output.right);

    if (right_abs > max_abs){
        max_abs = right_abs;
    }

    if (max_abs > output_limit){
        float scale = output_limit / max_abs;
        output.left *= scale;
        output.right *= scale;
    }

    return output;
}

