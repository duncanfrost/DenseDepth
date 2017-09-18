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



template <typename T>
inline void DisplayImage(ORUtils::Image<T> *image, unsigned int xStart = 0,
                         unsigned int yStart = 0, unsigned int xLimit = 0,
                         unsigned int yLimit = 0)
{
    if (xLimit == 0) xLimit = image->noDims.x - xStart;
    if (yLimit == 0) yLimit = image->noDims.y - yStart;

    T* data = image->GetData(MEMORYDEVICE_CPU);
    for (unsigned int y = yStart; y < yStart + yLimit; y++)
    {
        for (unsigned int x = xStart; x < xStart + xLimit; x++)
        {
            unsigned int index = x + image->noDims.x*y;
            std::cout << std::setw(10) << std::setprecision(4) << data[index];
        }
        std::cout << std::endl;
    } 
}




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
inline void CalculateErrorSmoothPixel(float *error, float *update, float *depths, unsigned int index,
                                      bool *valid, Vector2i imgSize, int x, int y, bool calcJacs)
{
	int xo = 1, yo = 1;
	float alpha = 5;
	float depth = depths[index];

	for (int offX = -xo; offX <= xo; offX++) for (int offY = -yo; offY <= yo; offY++)
	{
		if (offX == 0 && offY == 0) continue;

		int nX = x + offX, nY = y + offY;

		if (!(nX >= 0 && nY >= 0 && nX < imgSize.x && nY < imgSize.y)) continue;

		int nLocId = nX + imgSize.x * nY;

		float nDepth = depths[nLocId];

		float residual = alpha*(depth - nDepth);

		float dResdDepthPix = alpha;

		*error += residual*residual;
		if (calcJacs)
			*update += dResdDepthPix * residual;
	}
}

//Some of the checks here have been commented out. They tend to be a bit too strict
_CPU_AND_GPU_CODE_
inline void UpdateFilterPixel(float *mu, float *sigma2, 
                              float pixelDepth, float pixelVar, bool valid)
{
	if (pixelDepth <= 0 || !valid) return;

	float resultDepth = 1.0f / pixelDepth;

	float inputDepth = *mu;
	float inputVar = *sigma2;

	float idVar = inputVar * SUCC_VAR_INC_FAC;

	// update var with observation
	float w = pixelVar / (pixelVar + idVar);
	float outDepth = (1 - w)*resultDepth + w*inputDepth;
	*mu = UNZERO(outDepth);


	idVar = idVar * w;
	if (idVar < inputVar)
		*sigma2 = idVar;
}

_CPU_AND_GPU_CODE_
inline void UpdateFilterAveragePixel(float *mu, float *sigma2,
                                     float pixelDepth, float pixelVar, bool valid)
{
	if (pixelDepth <= 0 || !valid) return;

	float oldF = 1 / *mu;
	float newF = pixelDepth;
	float oldW = *sigma2;
	float newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, 150);

	*mu = 1 / newF;
	*sigma2 = newW;
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
inline void ComputeTVGradPixel(float *u, float *p1, float *p2, Vector2i imgSize,
                               int x, int y)
{
	float L2=8.0;
	float tau=0.02;
	float sigma=1.0/(L2*tau);

	int width = imgSize.x;
	int height = imgSize.y;
	int locId = x + imgSize.x * y;
	float uMiddle = u[locId];
	float uDown;
	if (y==height-1)
		uDown = uMiddle;
	else
		uDown = u[x + width*(y+1)];

	float uRight;
	if (x==width-1)
		uRight = uMiddle;
	else
		uRight = u[x + 1 + width*y];

	float gradX = uRight - uMiddle;
	float gradY = uDown - uMiddle;

	float *p1Pix = &p1[locId];
	float *p2Pix = &p2[locId];

	*p1Pix = *p1Pix + sigma*gradX;
	*p2Pix = *p2Pix + sigma*gradY;

	float p1PixSq = (*p1Pix)*(*p1Pix);
	float p2PixSq = (*p2Pix)*(*p2Pix);


	float normep = max_agnostic(1.0f, p1PixSq + p2PixSq);
	*p1Pix = *p1Pix / normep;
	*p2Pix = *p2Pix / normep;

}

_CPU_AND_GPU_CODE_
inline void TVComputeDivPixel(float *p1, float *p2, float *divergence,
                              Vector2i imgSize, int x, int y)
{
	int width = imgSize.x;
	int height = imgSize.y;

	int locId = x + width * y;

	float p1left, p1right, p2up, p2down;
	if (y==0)
		p2up = 0;
	else
		p2up = p2[x + width*(y-1)];

	if (x==0)
		p1left = 0;
	else
		p1left = p1[x-1 + width*y];


	if (y==height-1)
		p2down = 0;
	else
		p2down = p2[locId];


	if (x==width-1)
		p1right = 0;
	else
		p1right = p1[locId];


	float gxx1 = p1right - p1left;
	float gyy2 = p2down - p2up;

	*divergence = gxx1 + gyy2;
}

_CPU_AND_GPU_CODE_
inline void TVSmoothPixel(float *u, float *divergence, float *nim,
                          float lambda, Vector2i imgSize, int x, int y)
{
	int locId = x + imgSize.x * y;
	float tau=0.02;
	float theta=1.0;
	float lt=lambda*tau;

	float imPix = u[locId];
	float imPixOrig = nim[locId];
	float div = divergence[locId];

	float v = imPix + tau*div;

	float term1 = (v-lt)*(v-imPixOrig>lt);
	float term2 = (v+lt)*(v-imPixOrig<-lt);
	float term3 = (abs_agnostic(v-imPixOrig)<=lt);


	float imPixNew = term1 + term2 + imPixOrig*term3;
	float uNew = imPixNew + theta*(imPixNew-imPix);
	u[locId] = uNew;
}

_CPU_AND_GPU_CODE_
inline void ApplyUpdatesPixel(float error, float newError, float newDepth,
                              float update, float *depth, float *dp)
{
	if (newError < error && abs_agnostic(update) > 1e-2) {
		*depth = newDepth;
		*dp *= 1.1f;
	}
	else *dp *= 0.9f;
}

_CPU_AND_GPU_CODE_
inline float depthFromIndex(int i,float zmin, float depthIncrement)
{
	float idepth = zmin + depthIncrement*i;
	return 1/idepth;
}
