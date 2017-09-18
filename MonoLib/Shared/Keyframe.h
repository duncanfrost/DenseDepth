#include "DenseMonoMath.h"
#include <ORUtils/TimeStampedImage.h>
#include <sophus/se3.hpp>

struct KeyFrame
{
    Sophus::SE3f pose;
    ORUtils::TimeStampedImage<Vector4u> *frame;
    float meanGrey;
};
