//
// Created by ROG STRIX on 2025/11/10.
//

#ifndef BMI088_MATRIX_H
#define BMI088_MATRIX_H
void Matrix33fTrans(float mat[3][3], float res[3][3]);
void Matrix33fMultVector3f(float mat[3][3], const float vec[3], float res[3]);
void Vector3fCross(const float a[3], const float b[3], float res[3]);
float Vector3fNorm(const float vec[3]);
void Vector3fAdd(const float a[3], const float b[3], float res[3]);
void Vector3fSub(const float a[3], const float b[3], float res[3]);
void Vector4fUnit(const float vec[4], float res[4]);
float Vector4fNorm(const float vec[4]);
#endif //BMI088_MATRIX_H