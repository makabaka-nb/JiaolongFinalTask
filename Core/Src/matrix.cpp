//
// Created by ROG STRIX on 2025/11/10.
//
#include"matrix.h"

#include <cmath>

void Matrix33fTrans(float mat[3][3], float res[3][3]) {
    res[0][0] = mat[0][0];
    res[0][1] = mat[1][0];
    res[0][2] = mat[2][0];
    res[1][0] = mat[0][1];
    res[1][1] = mat[1][1];
    res[1][2] = mat[2][1];
    res[2][0] = mat[0][2];
    res[2][1] = mat[1][2];
    res[2][2] = mat[2][2];
}
void Matrix33fMultVector3f(float mat[3][3], const float vec[3], float res[3]) {
    res[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2];
    res[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2];
    res[2] = mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2];
}

void Vector3fCross(const float a[3], const float b[3], float res[3]) {
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
}

float Vector3fNorm(const float vec[3]) {
    return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void Vector3fAdd(const float a[3], const float b[3], float res[3]) {
    res[0] = a[0] + b[0];
    res[1] = a[1] + b[1];
    res[2] = a[2] + b[2];
}

void Vector3fSub(const float a[3], const float b[3], float res[3]) {
    res[0] = a[0] - b[0];
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
}

float Vector4fNorm(const float vec[4]) {
    return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2] +
                 vec[3] * vec[3]);
}

void Vector4fUnit(const float vec[4], float res[4]) {
    float norm = Vector4fNorm(vec);
    if (norm > 1e-8f) {
        res[0] = vec[0] / norm;
        res[1] = vec[1] / norm;
        res[2] = vec[2] / norm;
        res[3] = vec[3] / norm;
    } else {
        res[0] = vec[0];
        res[1] = vec[1];
        res[2] = vec[2];
        res[3] = vec[3];
    }
}
