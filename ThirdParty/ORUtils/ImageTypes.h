// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MathTypes.h"
#include "Image.h"
#include "TimeStampedImage.h"

typedef ORUtils::Image<bool> ORBoolImage;
typedef ORUtils::Image<float> ORFloatImage;
typedef ORUtils::Image<Vector2f> ORFloat2Image;
typedef ORUtils::Image<Vector3f> ORFloat3Image;
typedef ORUtils::Image<Vector4f> ORFloat4Image;
typedef ORUtils::Image<int> ORIntImage;
typedef ORUtils::Image<Vector2i> ORInt2Image;
typedef ORUtils::Image<Vector3i> ORInt3Image;
typedef ORUtils::Image<Vector4i> ORInt4Image;
typedef ORUtils::Image<short> ORShortImage;
typedef ORUtils::Image<Vector2s> ORShort2Image;
typedef ORUtils::Image<Vector3s> ORShort3Image;
typedef ORUtils::Image<Vector4s> ORShort4Image;
typedef ORUtils::Image<uchar> ORUCharImage;
typedef ORUtils::Image<Vector4u> ORUChar4Image;
typedef ORUtils::Image<uint> ORUIntImage;
typedef ORUtils::Image<ushort> ORUShortImage;

typedef ORUtils::TimeStampedImage<float> ORFloatTSImage;
typedef ORUtils::TimeStampedImage<Vector4u> ORUChar4TSImage;
typedef ORUtils::TimeStampedImage<short> ORShortTSImage;
typedef ORUtils::TimeStampedImage<uchar> ORUCharTSImage;
