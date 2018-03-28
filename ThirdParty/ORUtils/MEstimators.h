// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once
#ifndef __METALC__
#include <math.h>
#endif

#include "MathUtils.h"
#include "PlatformIndependence.h"

// Tukey loss
_CPU_AND_GPU_CODE_ inline float tukey_rho(float r, float c)
{
	const float c_sq_6 = c * c / 6.f;
	if (fabs(r) <= c)
	{
		float tukey_r = r / c;
		tukey_r *= tukey_r;
		tukey_r = 1 - tukey_r;
		tukey_r = tukey_r * tukey_r * tukey_r;

		return c_sq_6 * (1.f - tukey_r);
	}
	else
	{
		return c_sq_6;
	}
}

_CPU_AND_GPU_CODE_ inline float tukey_rho_deriv(float r, float c)
{
	if (fabs(r) <= c)
	{
		float tukey_r = r / c;
		tukey_r *= tukey_r;
		tukey_r = 1 - tukey_r;
		tukey_r *= tukey_r;

		return r * tukey_r;
	}
	else
	{
		return 0.f;
	}
}

_CPU_AND_GPU_CODE_ inline float tukey_rho_in(float r, float c)
{
	const float c_sq_6 = c * c / 6.f;
	if (fabs(r) <= c)
	{
		float tukey_r = r / c;
		tukey_r *= tukey_r;
		tukey_r = 1 - tukey_r;
		tukey_r = tukey_r * tukey_r * tukey_r;

		return c_sq_6 * (1.f - tukey_r);
	}
	else
	{
		return c_sq_6;
	}
}

_CPU_AND_GPU_CODE_ inline float tukey_rho_deriv2(float r, float c)
{
	return fabs(r) < c ? 1.0f : 0.0f;
}

// Depth Tracker Norm
_CPU_AND_GPU_CODE_ inline float rho(float r, float huber_b)
{
	float tmp = fabs(r) - huber_b;
	tmp = MAX(tmp, 0.0f);
	return r*r - tmp*tmp;
}

_CPU_AND_GPU_CODE_ inline float rho_deriv(float r, float huber_b)
{
	return 2.0f * CLAMP(r, -huber_b, huber_b);
}

_CPU_AND_GPU_CODE_ inline float rho_deriv2(float r, float huber_b)
{
	return fabs(r) < huber_b ? 2.0f : 0.0f;
}

// huber norm
_CPU_AND_GPU_CODE_ inline float huber_rho(float r, float c)
{
	if (fabs(r) <= c) return 0.5f * r * r;
	return c * fabs(r) - 0.5f * c * c;
}

_CPU_AND_GPU_CODE_ inline float huber_rho_deriv(float r, float c)
{
	if (fabs(r) <= c) return r;
	return r * c / fabs(r);
}

_CPU_AND_GPU_CODE_ inline float huber_rho_deriv2(float r, float c)
{
	return fabs(r) <= c ? 1.0f : 0.0f;
}