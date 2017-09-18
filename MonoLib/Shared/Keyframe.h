#include "DenseMonoMath.h"
#include <ORUtils/TimeStampedImage.h>

struct KeyFrame
{
    DenseMono::SE3f pose;
    ORUtils::TimeStampedImage<Vector4u> *frame;
    float meanGrey;
};
