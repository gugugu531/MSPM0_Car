#ifndef ROTATION_H
#define ROTATION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} ROTATION_EULER;

typedef struct {
    float value[3][3];
} ROTATION_MATRIX;

void Rotation_MatrixIdentity(ROTATION_MATRIX *matrix);
void Rotation_EulerToMatrix(const ROTATION_EULER *euler, ROTATION_MATRIX *matrix);
void Rotation_MatrixToEuler(const ROTATION_MATRIX *matrix, ROTATION_EULER *euler);
void Rotation_MatrixMultiply(const ROTATION_MATRIX *left,
                             const ROTATION_MATRIX *right,
                             ROTATION_MATRIX *out);
void Rotation_MatrixTranspose(const ROTATION_MATRIX *matrix, ROTATION_MATRIX *out);
void Rotation_VectorApply(const ROTATION_MATRIX *matrix,
                          const float input[3],
                          float output[3]);

#ifdef __cplusplus
}
#endif

#endif /* ROTATION_H */
