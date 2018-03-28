// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once
#ifndef __METALC__
#include <math.h>
#endif

#include "MathUtils.h"
#include "PlatformIndependence.h"

template<typename T> _CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear(const CONSTPTR(ORUtils::Vector4<T>) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	const Vector2i p((int)floor(position.x), (int)floor(position.y));
	const Vector2f delta(position.x - (float)p.x, position.y - (float)p.y);

	ORUtils::Vector4<T> a = source[p.x + p.y * imgSize.x];
	ORUtils::Vector4<T> b(T(0)), c(T(0)), d(T(0));

	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	Vector4f result;
	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline float interpolateBilinear_single_withHoles(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	const Vector2i p((int)floor(position.x), (int)floor(position.y));
	const Vector2f delta(position.x - (float)p.x, position.y - (float)p.y);

	T a = source[p.x + p.y * imgSize.x];
	T b(0), c(0), d(0);

	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a == 0 || b == 0 || c == 0 || d == 0) return 0.0f;

	float result = ((float)a * (1.0f - delta.x) * (1.0f - delta.y) + (float)b * delta.x * (1.0f - delta.y) +
		(float)c * (1.0f - delta.x) * delta.y + (float)d * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline float interpolateBilinear_single(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	const Vector2i p((int)floor(position.x), (int)floor(position.y));
	const Vector2f delta(position.x - (float)p.x, position.y - (float)p.y);

	T a = source[p.x + p.y * imgSize.x];
	T b(0), c(0), d(0);

	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	float result = ((float)a * (1.0f - delta.x) * (1.0f - delta.y) + (float)b * delta.x * (1.0f - delta.y) +
		(float)c * (1.0f - delta.x) * delta.y + (float)d * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline Vector2f interpolateBilinear_Vector2(const CONSTPTR(ORUtils::Vector2<T>) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	const Vector2i p((int)floor(position.x), (int)floor(position.y));
	const Vector2f delta(position.x - (float)p.x, position.y - (float)p.y);

	ORUtils::Vector2<T> a = source[p.x + p.y * imgSize.x];
	ORUtils::Vector2<T> b(T(0)), c(T(0)), d(T(0));

	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a == 0.0f || b == 0.0f || c == 0.0f || d == 0.0f) return 0.0f;

	Vector2f result;
	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear_withHoles(const CONSTPTR(ORUtils::Vector4<T>) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	const Vector2s p((short)floor(position.x), (short)floor(position.y));
	const Vector2f delta(position.x - (float)p.x, position.y - (float)p.y);

	const ORUtils::Vector4<T> a = source[p.x + p.y * imgSize.x];
	const ORUtils::Vector4<T> b = source[(p.x + 1) + p.y * imgSize.x];
	const ORUtils::Vector4<T> c = source[p.x + (p.y + 1) * imgSize.x];
	const ORUtils::Vector4<T> d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	Vector4f result;
	if (a.w < 0 || b.w < 0 || c.w < 0 || d.w < 0)
	{
		result.x = 0; result.y = 0; result.z = 0; result.w = -1.0f;
		return result;
	}

	result.x = ((float)a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.x * delta.x * (1.0f - delta.y) +
		(float)c.x * (1.0f - delta.x) * delta.y + (float)d.x * delta.x * delta.y);
	result.y = ((float)a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.y * delta.x * (1.0f - delta.y) +
		(float)c.y * (1.0f - delta.x) * delta.y + (float)d.y * delta.x * delta.y);
	result.z = ((float)a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.z * delta.x * (1.0f - delta.y) +
		(float)c.z * (1.0f - delta.x) * delta.y + (float)d.z * delta.x * delta.y);
	result.w = ((float)a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float)b.w * delta.x * (1.0f - delta.y) +
		(float)c.w * (1.0f - delta.x) * delta.y + (float)d.w * delta.x * delta.y);

	return result;
}

template<typename T> _CPU_AND_GPU_CODE_ inline float interpolateBilinear_withHoles_single(const CONSTPTR(T) *source,
	const THREADPTR(Vector2f) & position, const CONSTPTR(Vector2i) & imgSize)
{
	const Vector2i p((int)floor(position.x), (int)floor(position.y));
	const Vector2f delta(position.x - (float)p.x, position.y - (float)p.y);

	T a = source[p.x + p.y * imgSize.x];
	T b(0), c(0), d(0);

	if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
	if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
	if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

	if (a <= 0 || b <= 0 || c <= 0 || d <= 0) return -1;

	float result = ((float)a * (1.0f - delta.x) * (1.0f - delta.y) + (float)b * delta.x * (1.0f - delta.y) +
		(float)c * (1.0f - delta.x) * delta.y + (float)d * delta.x * delta.y);

	return result;
}

_CPU_AND_GPU_CODE_ inline void convertColourToNormalisedIntensity(DEVICEPTR(float) *imageData_out, int x, int y, Vector2i dims,
	const CONSTPTR(Vector4u) *imageData_in)
{
	const int linear_pos = y * dims.x + x;
	const Vector4u colour = imageData_in[linear_pos];

	imageData_out[linear_pos] = (0.2989f * colour.x + 0.587f * colour.y + 0.114f * colour.z) / 255.f;
}

template <typename T>
_CPU_AND_GPU_CODE_ inline void boxFilter2x2(DEVICEPTR(T) *imageData_out, int x_out, int y_out, Vector2i newDims,
    const CONSTPTR(T) *imageData_in, int x_in, int y_in, Vector2i oldDims)
{
	T pixel_out = T((uchar)(0));

	pixel_out += imageData_in[(x_in + 0) + (y_in + 0) * oldDims.x] / 4;
	pixel_out += imageData_in[(x_in + 1) + (y_in + 0) * oldDims.x] / 4;
	pixel_out += imageData_in[(x_in + 0) + (y_in + 1) * oldDims.x] / 4;
	pixel_out += imageData_in[(x_in + 1) + (y_in + 1) * oldDims.x] / 4;

	imageData_out[x_out + y_out * newDims.x] = pixel_out;
}

template <typename T>
_CPU_AND_GPU_CODE_ inline void boxFilter1x2(DEVICEPTR(T) *imageData_out, int x_out, int y_out, Vector2i newDims,
    const CONSTPTR(T) *imageData_in, int x_in, int y_in, Vector2i oldDims)
{
	T pixel_out = T((uchar)(0));

	pixel_out += imageData_in[(x_in + 0) + (y_in + 0) * oldDims.x] / 2;
	pixel_out += imageData_in[(x_in + 1) + (y_in + 0) * oldDims.x] / 2;

	imageData_out[x_out + y_out * newDims.x] = pixel_out;
}

template <typename T>
_CPU_AND_GPU_CODE_ inline void boxFilter2x1(DEVICEPTR(T) *imageData_out, int x_out, int y_out, Vector2i newDims,
    const CONSTPTR(T) *imageData_in, int x_in, int y_in, Vector2i oldDims)
{
	T pixel_out = T((uchar)(0));

	pixel_out += imageData_in[(x_in + 0) + (y_in + 0) * oldDims.x] / 2;
	pixel_out += imageData_in[(x_in + 0) + (y_in + 1) * oldDims.x] / 2;

	imageData_out[x_out + y_out * newDims.x] = pixel_out;
}

template <typename T>
_CPU_AND_GPU_CODE_ inline void boxFilter1x1(DEVICEPTR(T) *imageData_out, int x_out, int y_out, Vector2i newDims,
    const CONSTPTR(T) *imageData_in, int x_in, int y_in, Vector2i oldDims)
{
	T pixel_out = T((uchar)(0));
	pixel_out += imageData_in[(x_in + 0) + (y_in + 0) * oldDims.x];
	imageData_out[x_out + y_out * newDims.x] = pixel_out;
}

_CPU_AND_GPU_CODE_ inline void filterSubsampleWithHoles(DEVICEPTR(float) *imageData_out, int x, int y, Vector2i newDims,
	const CONSTPTR(float) *imageData_in, Vector2i oldDims)
{
	int src_pos_x = x * 2, src_pos_y = y * 2;
	float pixel_out = 0.0f, pixel_in, no_good_pixels = 0.0f;

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

	if (no_good_pixels > 0) pixel_out /= no_good_pixels;

	imageData_out[x + y * newDims.x] = pixel_out;
}

_CPU_AND_GPU_CODE_ inline void filterSubsampleWithHoles(DEVICEPTR(Vector4f) *imageData_out, int x, int y, Vector2i newDims,
	const CONSTPTR(Vector4f) *imageData_in, Vector2i oldDims)
{
	int src_pos_x = x * 2, src_pos_y = y * 2;
	Vector4f pixel_out = 0.0f, pixel_in; float no_good_pixels = 0.0f;

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in.w >= 0) { pixel_out += pixel_in; no_good_pixels++; }

	if (no_good_pixels > 0) pixel_out /= no_good_pixels;
	else { pixel_out.w = -1.0f; }

	imageData_out[x + y * newDims.x] = pixel_out;
}

_CPU_AND_GPU_CODE_ inline void gradientX(DEVICEPTR(Vector4s) *grad, int x, int y, const CONSTPTR(Vector4u) *image, Vector2i imgSize)
{
	Vector4s d1, d2, d3, d_out;

	d1.x = image[(x + 1) + (y - 1) * imgSize.x].x - image[(x - 1) + (y - 1) * imgSize.x].x;
	d1.y = image[(x + 1) + (y - 1) * imgSize.x].y - image[(x - 1) + (y - 1) * imgSize.x].y;
	d1.z = image[(x + 1) + (y - 1) * imgSize.x].z - image[(x - 1) + (y - 1) * imgSize.x].z;

	d2.x = image[(x + 1) + (y)* imgSize.x].x - image[(x - 1) + (y)* imgSize.x].x;
	d2.y = image[(x + 1) + (y)* imgSize.x].y - image[(x - 1) + (y)* imgSize.x].y;
	d2.z = image[(x + 1) + (y)* imgSize.x].z - image[(x - 1) + (y)* imgSize.x].z;

	d3.x = image[(x + 1) + (y + 1) * imgSize.x].x - image[(x - 1) + (y + 1) * imgSize.x].x;
	d3.y = image[(x + 1) + (y + 1) * imgSize.x].y - image[(x - 1) + (y + 1) * imgSize.x].y;
	d3.z = image[(x + 1) + (y + 1) * imgSize.x].z - image[(x - 1) + (y + 1) * imgSize.x].z;

	d1.w = d2.w = d3.w = 2 * 255;

	d_out.x = (d1.x + 2 * d2.x + d3.x) / 8;
	d_out.y = (d1.y + 2 * d2.y + d3.y) / 8;
	d_out.z = (d1.z + 2 * d2.z + d3.z) / 8;
	d_out.w = (d1.w + 2 * d2.w + d3.w) / 8;

	grad[x + y * imgSize.x] = d_out;
}

_CPU_AND_GPU_CODE_ inline void gradientY(DEVICEPTR(Vector4s) *grad, int x, int y, const CONSTPTR(Vector4u) *image, Vector2i imgSize)
{
	Vector4s d1, d2, d3, d_out;

	d1.x = image[(x - 1) + (y + 1) * imgSize.x].x - image[(x - 1) + (y - 1) * imgSize.x].x;
	d1.y = image[(x - 1) + (y + 1) * imgSize.x].y - image[(x - 1) + (y - 1) * imgSize.x].y;
	d1.z = image[(x - 1) + (y + 1) * imgSize.x].z - image[(x - 1) + (y - 1) * imgSize.x].z;

	d2.x = image[(x)+(y + 1) * imgSize.x].x - image[(x)+(y - 1) * imgSize.x].x;
	d2.y = image[(x)+(y + 1) * imgSize.x].y - image[(x)+(y - 1) * imgSize.x].y;
	d2.z = image[(x)+(y + 1) * imgSize.x].z - image[(x)+(y - 1) * imgSize.x].z;

	d3.x = image[(x + 1) + (y + 1) * imgSize.x].x - image[(x + 1) + (y - 1) * imgSize.x].x;
	d3.y = image[(x + 1) + (y + 1) * imgSize.x].y - image[(x + 1) + (y - 1) * imgSize.x].y;
	d3.z = image[(x + 1) + (y + 1) * imgSize.x].z - image[(x + 1) + (y - 1) * imgSize.x].z;

	d1.w = d2.w = d3.w = 2 * 255;

	d_out.x = (d1.x + 2 * d2.x + d3.x) / 8;
	d_out.y = (d1.y + 2 * d2.y + d3.y) / 8;
	d_out.z = (d1.z + 2 * d2.z + d3.z) / 8;
	d_out.w = (d1.w + 2 * d2.w + d3.w) / 8;

	grad[x + y * imgSize.x] = d_out;
}

_CPU_AND_GPU_CODE_ inline void gradientXY(DEVICEPTR(Vector2f) *grad, int x, int y, const CONSTPTR(float) *image, Vector2i imgSize)
{
	Vector2f d1, d2, d3, d_out;

	// Compute gradient in the X direction
	d1.x = image[(y - 1) * imgSize.x + (x + 1)] - image[(y - 1) * imgSize.x + (x - 1)];
	d2.x = image[(y)* imgSize.x + (x + 1)] - image[(y)* imgSize.x + (x - 1)];
	d3.x = image[(y + 1) * imgSize.x + (x + 1)] - image[(y + 1) * imgSize.x + (x - 1)];

	// Compute gradient in the Y direction
	d1.y = image[(y + 1) * imgSize.x + (x - 1)] - image[(y - 1) * imgSize.x + (x - 1)];
	d2.y = image[(y + 1) * imgSize.x + (x)] - image[(y - 1) * imgSize.x + (x)];
	d3.y = image[(y + 1) * imgSize.x + (x + 1)] - image[(y - 1) * imgSize.x + (x + 1)];

	d_out.x = (d1.x + 2.f * d2.x + d3.x) / 8.f;
	d_out.y = (d1.y + 2.f * d2.y + d3.y) / 8.f;

	grad[y * imgSize.x + x] = d_out;
}
_CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinearVec4(Vector4u *data,  float x, float y, unsigned int width)
{
	int ix = (int)x;
	int iy = (int)y;

	const Vector4u dataTLchar = *(data + ix + width*iy);
	const Vector4u dataTRchar = *(data + ix + 1 + width*iy);
	const Vector4u dataBLchar = *(data + ix + width*(iy + 1));
	const Vector4u dataBRchar = *(data + ix + 1 + width*(iy + 1));

	Vector4f dataTL, dataTR, dataBL, dataBR;

	for (unsigned int i = 0; i < 4; i++)
	{
		dataTL[i] = (float)dataTLchar[i];
		dataBR[i] = (float)dataBRchar[i];
		dataBL[i] = (float)dataBLchar[i];
		dataTR[i] = (float)dataTRchar[i];
	}

	float dx = x - (int)x;
	float dy = y - (int)y;

	float weight_tl = (1.0f - dx) * (1.0f - dy);
	float weight_tr = (dx)        * (1.0f - dy);
	float weight_bl = (1.0f - dx) * (dy);
	float weight_br = (dx)        * (dy);

	Vector4f subPix = weight_tl * dataTL + weight_tr * dataTR + weight_bl * dataBL + weight_br * dataBR;
	return subPix;
}
_CPU_AND_GPU_CODE_ inline float interpolateBilinearUChar(unsigned char *data,  Vector2f pos, unsigned int width)
{
    int ix = (int)pos.x;
    int iy = (int)pos.y;

    const unsigned char dataTLchar = *(data + ix + width*iy);
    const unsigned char dataTRchar = *(data + ix + 1 + width*iy);
    const unsigned char dataBLchar = *(data + ix + width*(iy + 1));
    const unsigned char dataBRchar = *(data + ix + 1 + width*(iy + 1));

    float dataTL, dataTR, dataBL, dataBR;

    dataTL = (float)dataTLchar;
    dataBR = (float)dataBRchar;
    dataBL = (float)dataBLchar;
    dataTR = (float)dataTRchar;

    float dx = pos.x - (int)pos.x;
    float dy = pos.y - (int)pos.y;

    float weight_tl = (1.0f - dx) * (1.0f - dy);
    float weight_tr = (dx)        * (1.0f - dy);
    float weight_bl = (1.0f - dx) * (dy);
    float weight_br = (dx)        * (dy);

    float subPix = weight_tl * dataTL + weight_tr * dataTR + weight_bl * dataBL + weight_br * dataBR;
    return subPix;
}

_CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinearVec4(Vector4u *data,  Vector2f pos, unsigned int width)
{
    int ix = (int)pos.x;
    int iy = (int)pos.y;

    const Vector4u dataTLchar = *(data + ix + width*iy);
    const Vector4u dataTRchar = *(data + ix + 1 + width*iy);
    const Vector4u dataBLchar = *(data + ix + width*(iy + 1));
    const Vector4u dataBRchar = *(data + ix + 1 + width*(iy + 1));

    Vector4f dataTL, dataTR, dataBL, dataBR;

    for (unsigned int i = 0; i < 4; i++)
    {
        dataTL[i] = (float)dataTLchar[i];
        dataBR[i] = (float)dataBRchar[i];
        dataBL[i] = (float)dataBLchar[i];
        dataTR[i] = (float)dataTRchar[i];
    }

    float dx = pos.x - (int)pos.x;
    float dy = pos.y - (int)pos.y;

    float weight_tl = (1.0f - dx) * (1.0f - dy);
    float weight_tr = (dx)        * (1.0f - dy);
    float weight_bl = (1.0f - dx) * (dy);
    float weight_br = (dx)        * (dy);

    Vector4f subPix = weight_tl * dataTL + weight_tr * dataTR + weight_bl * dataBL + weight_br * dataBR;
    return subPix;
}
