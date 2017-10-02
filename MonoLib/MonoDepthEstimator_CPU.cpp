#include "MonoDepthEstimator_CPU.h"
#include "Shared/MonoDepthEstimator_Shared.h"

using namespace MonoLib;


inline void updatePhotoError2dCPU(Matrix3f R, Vector3f T,
                                  Intrinsics K,
                                  Vector2i imgSize,
                                  float *photo_error,
                                  Vector4u *currImageData,
                                  Vector4u *refImageData, unsigned int depthSamples,
                                  float minDepth, float depthIncrement,
                                  int nUpdate)
{


    std::cout << "Rotation" << std::endl;
    std::cout << R << std::endl << std::endl;
    std::cout << T << std::endl;

    // for (int y = 0; y < imgSize.y; y++)
    //     for (int x = 0; x < imgSize.x; x++)
    std::cout << "I'm here" << std::endl;
    int x = 10;
    int y = 10;
        {
            float u = x;
            float v = y;

            Vector3f pointRefUnscaled;
            pointRefUnscaled.x = u * K.fxInv + K.cxInv;
            pointRefUnscaled.y = v * K.fyInv + K.cyInv;
            pointRefUnscaled.z = 1;

            for (unsigned int z = 0; z < depthSamples; z++)
            {
                int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;

                float idepth = minDepth + z*depthIncrement;
                float depth = 1.0f/idepth;

                Vector3f pointTrack = R * (pointRefUnscaled * depth) + T;
                Vector2f pointTrackImage;
                pointTrackImage.x = K.fx * pointTrack.x / pointTrack.z + K.cx;
                pointTrackImage.y = K.fy * pointTrack.y / pointTrack.z + K.cy;

                //if the voxel is projected in the current image then update photometric error and counter
                if(pointTrackImage[0]>0 && pointTrackImage[0]<imgSize.x-1 &&
                   pointTrackImage[1]>0 && pointTrackImage[1]<imgSize.y-1 && pointTrack[2] > 0)
                {
                    //L1 norm of the photometric error
                    Vector4f photo_current_OR = interpolateBilinearVec4(currImageData,pointTrackImage.x,
                                                                        pointTrackImage.y,imgSize.x);
                    Vector4u photo_ref = refImageData[x + imgSize.x*y];

                    float diffr = photo_current_OR.x-(float)photo_ref.x;	diffr = diffr < 0 ? -diffr : diffr;
                    float diffg = photo_current_OR.y-(float)photo_ref.y;	diffg = diffg < 0 ? -diffg : diffg;
                    float diffb = photo_current_OR.z-(float)photo_ref.z;	diffb = diffb < 0 ? -diffb : diffb;

                    float normL1=(diffr+diffg+diffb)/(3.*255.);//normL1 in [0 1]

                    float oldError = photo_error[offset];
                    float obsError = normL1;
                    float newError = (nUpdate * oldError + obsError) / (nUpdate + 1);

                    std::cout << "Z = " << z << " " << oldError << " ----> " << newError << " "
                              <<  photo_current_OR << std::endl;
                    std::cout << "_____" << pointTrackImage << std::endl;

                    photo_error[offset] = newError;
                }
                else
                {
                    float normL1=1.0f;//normL1 in [0 1]
                    float oldError = photo_error[offset];
                    float obsError = normL1;
                    float newError = (nUpdate * oldError + obsError) / (nUpdate + 1);

                    photo_error[offset] = newError;
                }
            }
        }
}


MonoDepthEstimator_CPU::MonoDepthEstimator_CPU(Vector2i imgSize, Vector4f intrinsics_raw)
    :MonoDepthEstimator(imgSize, intrinsics_raw)
{
}

MonoDepthEstimator_CPU::~MonoDepthEstimator_CPU()
{
}

void MonoDepthEstimator_CPU::SetRefImage(ORUChar4TSImage *frame)
{
    currDepthFrame->Init();
    currDepthFrame->colorImageData->SetFrom(frame, MEMCPYDIR_CPU_TO_CPU);

    MonoDepthEstimator::SetRefImage(frame);
}

float MonoDepthEstimator_CPU::MeasureErrorGT(float *mu, float *sigma2,
                                             float *GTDepths,
                                             unsigned int width, unsigned int height)
{
    float error = 0;
    unsigned int count = 0;


    unsigned int nTotalPix = width*height;


    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            unsigned int index = x + width*y;

            if (sigma2[index] > 1e-3)
                continue;

            float estDepth = 1 / mu[index];

            if (estDepth > 4)
                continue;

            float gtDepth = GTDepths[index];

            if (gtDepth <= 0)
                continue;

            float diff = estDepth - gtDepth;
            error += diff*diff;
            count++;
        }

    error /= count;
    error = sqrt(error);

    float nGoodPercentage = (float)count * 100 / (float)nTotalPix;

    std::cout << "GT Error " << error << "   " << nGoodPercentage << std::endl;
    return error;
}



void MonoDepthEstimator_CPU::SetBuffer(float *data, Vector2i imgSize, float value)
{

    for (int y = 0, locId = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++, locId++) {
            data[locId] = value;
        }
}


void MonoDepthEstimator_CPU::UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                                              ORUtils::TimeStampedImage<Vector4u> *frame)
{
    float depthIncrement = (optimPyramid->maxIDepth - optimPyramid->minIDepth) /
        (float)optimPyramid->depthSamples;

    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    Vector2i imgSize = monoLevel->depth->noDims;

    std::cout << "Depth increment" << depthIncrement << std::endl;
    std::cout << "Image size" << imgSize << std::endl;

    std::cout << refToTracker << std::endl;

    updatePhotoError2dCPU(refToTracker.GetR(),
                          refToTracker.GetT(),monoLevel->intrinsics,
                          imgSize,
                          optimPyramid->photoErrors->GetData(MEMORYDEVICE_CPU),
                          frame->GetData(MEMORYDEVICE_CPU),
                          currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU),
                          optimPyramid->depthSamples,
                          optimPyramid->minIDepth,
                          depthIncrement,monoLevel->nUpdate);

    monoLevel->nUpdate++;
}


void MonoDepthEstimator_CPU::DisplayPhotoVolume(int x, int y)
{
    Vector2i imgSize = optimPyramid->g->noDims;

    for (unsigned int z = 30; z < 35; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
        std::cout << optimPyramid->photoErrors->GetData(MEMORYDEVICE_CPU)[offset] << std::endl;
    }
}
