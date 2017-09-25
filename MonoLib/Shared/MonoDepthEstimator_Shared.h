#pragma once
#include <ORUtils/PlatformIndependence.h>
#include <ORUtils/MathTypes.h>
#include <ORUtils/PixelPrimitives.h>
#include <ORUtils/Image.h>

// #include "../../ViewLib/Common/Intrinsics.h"
#include <iostream>
#include <iomanip>


#define UNZERO(val) ((val) < 0 ? ((val) > -1e-8f ? -1e-8f : (val)) : ((val) < 1e-8f ? 1e-8f : (val)))
#define SUCC_VAR_INC_FAC 1.01f // before an ekf-update, the variance is increased by this factor.
#define MIN_EPL_LENGTH_SQUARED 1.0f * 1.0f
#define MAX_DIFF_CONSTANT 40.0f * 40.0f
#define MAX_DIFF_GRAD_MULT 0.5f * 0.5f

// this is the distance of the sample points used for the stereo descriptor.
#define GRADIENT_SAMPLE_DIST 1.0f

#define MAX_PHOTO_ERROR 255.0f * 255.0f;

#define HUBER_D 2.0f
#define VAR_WEIGHT 1.0f
#define DEPTH_COVAR 0.001f
#define CAM_PIX_NOISE2 4.0f*4.0f
#define STEREO_EPL_VAR_FAC 2.0f

_CPU_AND_GPU_CODE_
inline float GetCombinedError(float *photo_error,
                              float d_value,
                              int x, int y, int z,
                              float theta,
                              float lambda,
                              float increment,
                              Vector2i imgSize)
{
    int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
    float smoothRes =  d_value - increment*(float)z;
    float error=lambda * photo_error[offset] +
        (smoothRes*smoothRes) / (theta * 2.0f);
    return error;
}

_CPU_AND_GPU_CODE_
inline float GetPhotoError(float *photo_error,
                           int x, int y, int z,
                           Vector2i imgSize)
{
    int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
    return photo_error[offset];
}


_CPU_AND_GPU_CODE_
inline float PhotoErrorL1(Vector4f photoCurrent,
                          Vector4u photoRef)
{
    float diffr = photoCurrent.x-(float)photoRef.x;
    diffr = diffr < 0 ? -diffr : diffr;
    float diffg = photoCurrent.y-(float)photoRef.y;
    diffg = diffg < 0 ? -diffg : diffg;
    float diffb = photoCurrent.z-(float)photoRef.z;
    diffb = diffb < 0 ? -diffb : diffb;
    float normL1=(diffr+diffg+diffb)/(3.*255.);//normL1 in [0 1]
    return normL1;
}


_CPU_AND_GPU_CODE_
inline float PhotoErrorL1Grad(Vector4f photoCurrent,
                              Vector4u photoRef,
                              float gradXRef, float gradYRef,
                              float gradXCurr, float gradYCurr)
{
    float diffr = photoCurrent.x-(float)photoRef.x;
    diffr = diffr < 0 ? -diffr : diffr;
    float diffg = photoCurrent.y-(float)photoRef.y;
    diffg = diffg < 0 ? -diffg : diffg;
    float diffb = photoCurrent.z-(float)photoRef.z;
    diffb = diffb < 0 ? -diffb : diffb;
    float diffGradX = gradXCurr - gradXRef;
    diffGradX = diffGradX < 0 ? -diffGradX : diffGradX;
    float diffGradY = gradYCurr - gradYRef;
    diffGradY = diffGradY < 0 ? -diffGradY : diffGradY;

    diffr /= 255;
    diffg /= 255;
    diffb /= 255;


    float normL1=(diffGradX + diffGradY + diffr + diffg + diffb);
    return normL1;
}

_CPU_AND_GPU_CODE_
inline float PhotoErrorL1BW(float photoCurrent,
                            Vector4u photoRef)
{
    float diffr = photoCurrent-(float)photoRef.x;
    diffr = diffr < 0 ? -diffr : diffr;
    float diffg = photoCurrent-(float)photoRef.y;
    diffg = diffg < 0 ? -diffg : diffg;
    float diffb = photoCurrent-(float)photoRef.z;
    diffb = diffb < 0 ? -diffb : diffb;
    float normL1=(diffr+diffg+diffb)/(3.*255.);//normL1 in [0 1]
    return normL1;
}

_CPU_AND_GPU_CODE_
inline bool PointInImage(Vector2f point, Vector2i imgSize)
{
    if (point.x < 0 || point.x > imgSize.x-1 ||
        point.y < 0 || point.y > imgSize.y-1)
        return  false;
    else
        return true;
}




_CPU_AND_GPU_CODE_
inline Vector3f GetQuadFit(Vector3f errors)
{
    Vector3f depths = Vector3f(-1.0f,0.0f,1.0f);
    Matrix3f D;

    D(0,0) = depths[0]*depths[0];
    D(1,0) = depths[0];
    D(2,0) = 1;

    D(0,1) = depths[1]*depths[1];
    D(1,1) = depths[1];
    D(2,1) = 1;

    D(0,2) = depths[2]*depths[2];
    D(1,2) = depths[2];
    D(2,2) = 1;


    Matrix3f A;
    (D.t()*D).inv(A);
    Vector3f p = A*D.t()*errors;

    return p;
}

_CPU_AND_GPU_CODE_
inline float colourToIntensity(Vector4u colour)
{
    return (0.2989f * colour.x + 0.587f * colour.y + 0.114f * colour.z) / 255.f;
}

_CPU_AND_GPU_CODE_
inline float colourToIntensity(Vector4f colour)
{
    return (0.2989f * colour.x + 0.587f * colour.y + 0.114f * colour.z) / 255.f;
}

_CPU_AND_GPU_CODE_
inline float abs_agnostic(float a)
{
	if (a >= 0)
		return a;
	else
		return -a;
}

_CPU_AND_GPU_CODE_
inline float max_agnostic(float a, float b)
{
    if (a > b)
        return a;
    else
        return b;
}

_CPU_AND_GPU_CODE_
inline float depthFromIndex(int i,float zmin, float depthIncrement)
{
	float idepth = zmin + depthIncrement*i;
	return 1/idepth;
}
