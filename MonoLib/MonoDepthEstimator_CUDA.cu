#include "MonoDepthEstimator_CUDA.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
// #include "Shared/L0minimization.h"

using namespace MonoLib;

inline dim3 getBlocksFor2DProcess(int _width,int _height)
{
    return dim3((_width + 16 - 1) / 16, (_height + 16 - 1) / 16);
}

inline dim3 getThreadsFor2DProcess(int _width,int _height)
{
    return dim3(16,16);
}


inline float SumError(float *data, Vector2i imgSize)
{
    float error = 0;
    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            unsigned int index = x + imgSize.x*y;
            error += data[index];
        }

    return error;
}


template <typename T>
__device__ T clamp(T in, T max)
{
    T out = in;
    if (in > max - 1)
        out = max - 1;
    if (in < 0)
        out = 0;

    return out;
}

template <typename T>
__device__ T HuberNorm(T x, T eps)
{
    T absX = abs(x); 

    if (absX <= eps)
        return absX*absX / (2 * eps);
    else
        return absX - eps/2;
}

__global__ void CopyBuffer(float *source, float *dest, Vector2i imgSize)
{
    int id_x = blockIdx.x*blockDim.x+threadIdx.x;
    int id_y = blockIdx.y*blockDim.y+threadIdx.y;
    if (id_x > imgSize.x - 1 || id_y > imgSize.y - 1) return;
    int offset = id_x + id_y * imgSize.x;

    dest[offset] = source[offset];
}


__global__ void InitVolumeValues_device(float *photo_error,
                                        int *nUpdates,
                                        Vector2i imgSize, int depthSamples)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;

    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    for (int z = 0; z < depthSamples; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
        photo_error[offset] = 0;
        nUpdates[offset] = 0;
    }
}
__global__ void Init2DValues_device(float *g_data, Vector4u *imageData,
                                    Vector2i imgSize, float alpha, float beta)
{

    int id_x = blockIdx.x*blockDim.x+threadIdx.x;
    int id_y = blockIdx.y*blockDim.y+threadIdx.y;
    if (id_x > imgSize.x - 1 || id_y > imgSize.y - 1) return;
    int offset = id_x + id_y * imgSize.x;

    int x_plus = clamp(id_x+1, imgSize.x);
    int x_mid = clamp(id_x, imgSize.x);

    int y_plus = clamp(id_y+1, imgSize.y);
    int y_mid = clamp(id_y, imgSize.y);

    float pixMid = colourToIntensity(imageData[x_mid + imgSize.x * y_mid]);
    float pixXPlus = colourToIntensity(imageData[ x_plus + imgSize.x* y_mid]);
    float pixYPlus = colourToIntensity(imageData[ x_mid + imgSize.x* y_plus]);

    float dIx = pixXPlus - pixMid;
    float dIy = pixYPlus - pixMid;

    float norm = dIx*dIx + dIy*dIy;

    float normb = powf(norm,beta);

    g_data[offset]=expf(-alpha*normb);
    // g_data[offset]=1;
}

__global__ void ComputeGradient(float *image_data, Vector2i imgSize,
                                float *gradx_data, float *grady_data)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset = x + y * imgSize.x;

    int x_plus = clamp(x + 1, imgSize.x);
    int y_plus = clamp(y + 1, imgSize.y);

    //Gradient from matrix A
    gradx_data[image_offset]=image_data[x_plus + imgSize.x * y] - image_data[x + imgSize.x * y];
    grady_data[image_offset]=image_data[x + imgSize.x * y_plus] - image_data[x + imgSize.x * y];
}


__global__ void UpdateD(float *d_data, float *divQ_data,
                        float *a_data, float *g_data,
                        float sigma_d, float lambda,Vector2i imgSize)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset = x + y * imgSize.x;


    float divQ = divQ_data[image_offset];
    float d_in = d_data[image_offset];
    float g = g_data[image_offset];
    float a = a_data[image_offset];

    float d_out = d_in + sigma_d*(g*divQ + lambda*a);
    d_out /= (1 + sigma_d*lambda);

    float norm = max_agnostic(1.0f, abs_agnostic(d_out));

    d_data[image_offset] = d_out / norm;

}

__global__ void UpdateDL1(float *d_data, float *divQ_data,
                          float *a_data, float *p_data,
                          float sigma, float tau, float lambda,
                          Vector2i imgSize)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset = x + y * imgSize.x;


    float divQ = divQ_data[image_offset];
    float d_in = d_data[image_offset];
    float p = p_data[image_offset];

    float d_out = d_in + tau*divQ - lambda*(tau*p);
    d_data[image_offset] = d_out;
}


__global__ void UpdateQ(float *qx_data,float *qy_data,
                        float *dx_data,float *dy_data,
                        float *g_data, 
                        Vector2i imgSize, float sigma_q, float epsilon)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset = x + y * imgSize.x;

    float qx = qx_data[image_offset];
    float qy = qy_data[image_offset];

    float dx = dx_data[image_offset];
    float dy = dy_data[image_offset];

    float g = g_data[image_offset];

    float qx_out = qx + sigma_q * g * dx;
    float qy_out = qy + sigma_q * g * dy;

    qx_out /= (1 + sigma_q * epsilon);
    qy_out /= (1 + sigma_q * epsilon);

    float norm_q = sqrt(qx_out*qx_out + qy_out*qy_out);
    float norm = max_agnostic(1.0f, norm_q);
    qx_out /= norm;
    qy_out /= norm;


    qx_data[image_offset] = qx_out;
    qy_data[image_offset] = qy_out;
}

__global__ void UpdateQL1(float *qx_data,float *qy_data,
                          float *dx_data,float *dy_data,
                          Vector2i imgSize, float sigma_q)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;

    int image_offset = x + y * imgSize.x;

    float qx = qx_data[image_offset];
    float qy = qy_data[image_offset];

    float dx = dx_data[image_offset];
    float dy = dy_data[image_offset];

    float qx_out = qx + sigma_q * dx;
    float qy_out = qy + sigma_q * dy;

    float norm_q = sqrt(qx_out*qx_out + qy_out*qy_out);
    float norm = max_agnostic(1.0f, norm_q);
    qx_out /= norm;
    qy_out /= norm;


    qx_data[image_offset] = qx_out;
    qy_data[image_offset] = qy_out;
}

__global__ void UpdatePL1(float *p_data,
                          float *d_data,float *a_data,
                          Vector2i imgSize, float sigma,
                          float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;

    int image_offset = x + y * imgSize.x;

    float p = p_data[image_offset];
    float d = d_data[image_offset];
    float a = a_data[image_offset];

    float p_out = p + sigma*lambda*(d - a);

    float norm_p = abs(p_out);
    float norm = max_agnostic(1.0f, norm_p);
    p_out /= norm;

    p_data[image_offset] = p_out;
}


__global__ void ComputeDivQ(float *qx_data,float *qy_data,
                            float *divQ_data, Vector2i imgSize)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset = x + y * imgSize.x;


    float qx_left;
    if (x == 0)
        qx_left = 0;
    else
    {
        int x_minus = x-1;
        int offset_x_minus = x_minus + y * imgSize.x;
        qx_left = qx_data[offset_x_minus];
    }

    float qy_up;
    if (y == 0)
        qy_up = 0;
    else
    {
        int y_minus = y-1;
        int offset_y_minus = x + y_minus * imgSize.x;
        qy_up = qy_data[offset_y_minus];
    }

    float qx_mid = qx_data[image_offset];
    float qy_mid = qy_data[image_offset];

    float divQ = qy_mid - qy_up;
    divQ += qx_mid - qx_left ;

    divQ_data[image_offset] = divQ;
}

__global__ void OptimToDepth_device(float *optim_data, float *depth_data,
                                    float minIDepth, float maxIDepth,
                                    Vector2i imgSize)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset = x + y * imgSize.x;


    float iDepthDiff = maxIDepth - minIDepth;
    float inverseDepth = minIDepth + iDepthDiff * optim_data[image_offset];
    depth_data[image_offset] = 1 / inverseDepth;
}

__global__ void InitSmoothingParameters(float *a_data,float *d_data,float *qx_data,
                                        float *gy_data, Vector2i imgSize)
{
    int id_x = blockIdx.x*blockDim.x+threadIdx.x;
    int id_y = blockIdx.y*blockDim.y+threadIdx.y;
    if (id_x < imgSize.x && id_y < imgSize.y)
    {
        int offset = id_x + id_y * imgSize.x;
        d_data[offset]=a_data[offset];
        qx_data[offset]=0;
        gy_data[offset]=0;
    }
}

__global__ void updatePhotoErrorPatch(Matrix3f R, Vector3f T,
                                      Intrinsics K,
                                      Vector2i imgSize,
                                      float *photo_error,
                                      int *nUpdates,
                                      Vector4u *currImageData,
                                      Vector4u *refImageData, unsigned int depthSamples,
                                      float minIDepth, float depthIncrement)
{

    short int xBlock = blockIdx.x*blockDim.x+threadIdx.x;
    short int yBlock = blockIdx.y*blockDim.y+threadIdx.y;
    if (xBlock > imgSize.x - 1 || yBlock > imgSize.y - 1) return;


    for (unsigned int z = 0; z < depthSamples; z++)
    {
        int offset = xBlock + yBlock * imgSize.x+ z* imgSize.x*imgSize.y;
        int inCount = 0;
        float error = 0;
        // int xOffset = 0;
        // int yOffset = 0;

        for (int xOffset = -1; xOffset <= 1; xOffset++)
            for (int yOffset = -1; yOffset <= 1; yOffset++)
            {
                int x = xBlock + xOffset;
                int y = yBlock + yOffset;

                if (x > imgSize.x - 1 || y > imgSize.y - 1 || x < 0 || y < 0) continue;

                float u = x;
                float v = y;

                Vector3f pointRefUnscaled;
                pointRefUnscaled.x = u * K.fxInv + K.cxInv;
                pointRefUnscaled.y = v * K.fyInv + K.cyInv;
                pointRefUnscaled.z = 1;



                float idepth = minIDepth + z*depthIncrement;
                float depth = 1.0f/idepth;

                Vector3f pointTrack = R * (pointRefUnscaled * depth) + T;
                Vector2f pointTrackImage;
                pointTrackImage.x = K.fx * pointTrack.x / pointTrack.z + K.cx;
                pointTrackImage.y = K.fy * pointTrack.y / pointTrack.z + K.cy;



                //if the voxel is projected in the current image then update photometric error and counter
                if (PointInImage(pointTrackImage, imgSize) && pointTrack[2] > 0)
                {
                    //L1 norm of the photometric error

                    int x_ref_plus = clamp(x+1, imgSize.x);
                    int y_ref_plus = clamp(y+1, imgSize.y);

                    float x_curr_plus = clamp(pointTrackImage.x + 1, (float)imgSize.x);
                    float y_curr_plus = clamp(pointTrackImage.y + 1, (float)imgSize.y);

                    Vector4f photo_current_OR =
                        interpolateBilinearVec4(currImageData,pointTrackImage.x,
                                                pointTrackImage.y, imgSize.x);
                    Vector4u photo_ref = refImageData[x + imgSize.x*y];

                    float normL1 = PhotoErrorL1(photo_current_OR,photo_ref);
                    error += normL1;
                    inCount++;
                }
            } 

        if (inCount > 0)
        {
            float normL1 =  error / (float)inCount;
            float oldError = photo_error[offset];
            float obsError = normL1;
            int nUpdate = nUpdates[offset];

            float newError = (nUpdate * oldError + obsError) / (nUpdate + 1);

            photo_error[offset] = newError;
            nUpdates[offset] = nUpdate + 1;
        }
    }
}


__global__ void updatePhotoError2d(Matrix3f R, Vector3f T,
                                   Intrinsics K,
                                   Vector2i imgSize,
                                   float *photo_error,
                                   int *nUpdates,
                                   Vector4u *currImageData,
                                   Vector4u *refImageData, unsigned int depthSamples,
                                   float minIDepth, float depthIncrement)
{

    short int x = blockIdx.x*blockDim.x+threadIdx.x;
    short int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;


    float u = x;
    float v = y;

    Vector3f pointRefUnscaled;
    pointRefUnscaled.x = u * K.fxInv + K.cxInv;
    pointRefUnscaled.y = v * K.fyInv + K.cyInv;
    pointRefUnscaled.z = 1;



    for (unsigned int z = 0; z < depthSamples; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;

        float idepth = minIDepth + z*depthIncrement;
        float depth = 1.0f/idepth;

        Vector3f pointTrack = R * (pointRefUnscaled * depth) + T;
        Vector2f pointTrackImage;
        pointTrackImage.x = K.fx * pointTrack.x / pointTrack.z + K.cx;
        pointTrackImage.y = K.fy * pointTrack.y / pointTrack.z + K.cy;


        //if the voxel is projected in the current image then update photometric error and counter
        if (PointInImage(pointTrackImage, imgSize) && pointTrack[2] > 0)
        {
            //L1 norm of the photometric error

            int x_ref_plus = clamp(x+1, imgSize.x);
            int y_ref_plus = clamp(y+1, imgSize.y);

            float x_curr_plus = clamp(pointTrackImage.x + 1, (float)imgSize.x);
            float y_curr_plus = clamp(pointTrackImage.y + 1, (float)imgSize.y);

            Vector4f photo_current_OR =
                interpolateBilinearVec4(currImageData,pointTrackImage.x,
                                        pointTrackImage.y, imgSize.x);
            Vector4u photo_ref = refImageData[x + imgSize.x*y];


            //Compute gradient for ref
            float pixMid = colourToIntensity(photo_ref);
            float pixXPlus = colourToIntensity(refImageData[ x_ref_plus + imgSize.x* y]);
            float pixYPlus = colourToIntensity(refImageData[ x + imgSize.x* y_ref_plus]);
            float dIx_ref = pixXPlus - pixMid;
            float dIy_ref = pixYPlus - pixMid;


            //Compute gradient for current
            pixMid = colourToIntensity(photo_current_OR);
            pixXPlus = colourToIntensity(
                interpolateBilinearVec4(currImageData,x_curr_plus,
                                        pointTrackImage.y, imgSize.x));
            pixYPlus = colourToIntensity(
                interpolateBilinearVec4(currImageData,pointTrackImage.x,
                                        y_curr_plus, imgSize.x));
            float dIx_curr = pixXPlus - pixMid;
            float dIy_curr = pixYPlus - pixMid;

            float normL1 = PhotoErrorL1(photo_current_OR,photo_ref);
            // float normL1 = PhotoErrorL1Grad(photo_current_OR,photo_ref,
            //                             dIx_ref, dIy_ref,
            // dIx_curr, dIy_curr);

            float oldError = photo_error[offset];
            float obsError = normL1;
            int nUpdate = nUpdates[offset];

            float newError = (nUpdate * oldError + obsError) / (nUpdate + 1);

            photo_error[offset] = newError;
            nUpdates[offset] = nUpdate + 1;
        }
    }
}

__global__ void updatePhotoErrorFeatures(Matrix3f R, Vector3f T,
                                         Intrinsics K,
                                         Vector2i imgSize,
                                         float *photo_error,
                                         int *nUpdates,
                                         float *currImageData,
                                         float *refImageData,
                                         unsigned int depthSamples,
                                         float minIDepth,
                                         float depthIncrement)
{

    short int x = blockIdx.x*blockDim.x+threadIdx.x;
    short int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;


    int featureChannels = 64;

    float u = x;
    float v = y;

    Vector3f pointRefUnscaled;
    pointRefUnscaled.x = u * K.fxInv + K.cxInv;
    pointRefUnscaled.y = v * K.fyInv + K.cyInv;
    pointRefUnscaled.z = 1;

    for (unsigned int z = 0; z < depthSamples; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;

        float idepth = minIDepth + z*depthIncrement;
        float depth = 1.0f/idepth;

        Vector3f pointTrack = R * (pointRefUnscaled * depth) + T;
        Vector2f pointTrackImage;
        pointTrackImage.x = K.fx * pointTrack.x / pointTrack.z + K.cx;
        pointTrackImage.y = K.fy * pointTrack.y / pointTrack.z + K.cy;



        float interpData[64];
        //if the voxel is projected in the current image then update photometric error and counter
        if (PointInImage(pointTrackImage, imgSize) && pointTrack[2] > 0)
        {

            InterpolateFeature(currImageData, interpData, pointTrackImage,
                               imgSize.x, featureChannels);

            float total = 0;
            for (unsigned int c = 0; c < featureChannels; c++)
            {
                int index = featureChannels*(x + imgSize.x*y) + c;
                float featureRef = refImageData[index];
                float featureCurr = interpData[c];

                float diff = abs(featureRef - featureCurr);
                total += diff*diff;
            }
            float normL1 = total;

            float oldError = photo_error[offset];
            float obsError = normL1;
            int nUpdate = nUpdates[offset];

            float newError = (nUpdate * oldError + obsError) / (nUpdate + 1);

            photo_error[offset] = newError;
            nUpdates[offset] = nUpdate + 1;
        }
    }
}



__global__ void MinPhotoErrorInit(float *photo_error,float *d_data,
                                  float *a_data, int *minIdx_data,
                                  Vector2i imgSize, int depthSamples)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    float minError = 9999.0f;
    int minIdx = 0;

    float increment = 1.0f / (float)(depthSamples - 1);

    for (unsigned int z = 0; z < depthSamples; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
        float error= photo_error[offset];

        if (error < minError)
        {
            minIdx = z;
            minError = error;
        }
    }

    a_data[image_offset] = increment*(float)minIdx;
    d_data[image_offset] = increment*(float)minIdx;
    minIdx_data[image_offset] = minIdx;
}


__global__ void ComputeCertainty(float *photo_error,
                                 int *minIdx_data,
                                 float *certainty_data,
                                 Vector2i imgSize,
                                 int depthSamples)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;


    float c = 0;
    for (unsigned int z = 0; z < depthSamples-2; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
        float error= photo_error[offset];

        int offset_front = x + y * imgSize.x+ (z+1)* imgSize.x*imgSize.y;
        float error_front = photo_error[offset_front];


        int offset_front2 = x + y * imgSize.x+ (z+2)* imgSize.x*imgSize.y;
        float error_front2 = photo_error[offset_front2];

        float grad = error - error_front;
        float grad_front = error_front2 - error_front;

        float grad2 = grad_front - grad;
        c += grad2;
    }

    float val = expf(-c/3);
    if (val > 1) val = 1;

    certainty_data[image_offset] = val;
}






__global__ void MinPhotoError2d_device(float *photo_error,float *d_data,
                                       float *a_data, int *minIdx_data,
                                       Vector2i imgSize,
                                       int depthSamples, float theta,
                                       float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    float minError = 9999.0f;
    int minIdx = 0;

    float increment = 1.0f / (float)(depthSamples - 1);
    float d_value = d_data[image_offset];

    for (unsigned int z = 0; z < depthSamples; z++)
    {
        float error = GetCombinedError(photo_error, d_value,
                                       x,y,z,theta,lambda,increment,imgSize);

        if (error < minError)
        {
            minIdx = z;
            minError = error;
        }
    }

    a_data[image_offset] = increment*(float)minIdx;
    minIdx_data[image_offset] = minIdx;
}


__global__ void MinErrorNaiveFit_device(float *photo_error,float *d_data,
                                        float *a_data, int *minIdx_data,
                                        Vector2i imgSize,
                                        int depthSamples, float theta,
                                        float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    float minError = 9999.0f;
    int minIdx = 0;

    float increment = 1.0f / (float)(depthSamples - 1);
    float d_value = d_data[image_offset];

    for (unsigned int z = 0; z < depthSamples; z++)
    {
        float error = GetCombinedError(photo_error, d_value,
                                       x,y,z,theta,lambda,increment,imgSize);

        if (error < minError)
        {
            minIdx = z;
            minError = error;
        }
    }

    a_data[image_offset] = increment*(float)minIdx;
    minIdx_data[image_offset] = minIdx;

    if (minIdx == 0 || minIdx == depthSamples-1)
        return;

    float errorPlus = GetCombinedError(photo_error, d_value,
                                       x,y,minIdx+1,theta,lambda,increment,imgSize);
    float errorMinus = GetCombinedError(photo_error, d_value,
                                        x,y,minIdx-1,theta,lambda,increment,imgSize);


    Vector3f errors = Vector3f(errorMinus, minError, errorPlus);
    Vector3f param = GetQuadFit(errors);
    float a = param[0];
    float b = param[1];
    float t_best = -b / (2*a);

    a_data[image_offset] += t_best*increment;
}

__global__ void MinErrorTrueFit_device(float *photo_error,float *d_data,
                                       float *a_data, int *minIdx_data,
                                       float *error_data,
                                       Vector2i imgSize,
                                       int depthSamples, float theta,
                                       float eps, float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    float minError = 9999.0f;
    int minIdx = 0;

    float increment = 1.0f / (float)(depthSamples - 1);
    float d_value = d_data[image_offset];

    for (unsigned int z = 0; z < depthSamples; z++)
    {
        float error = GetCombinedError(photo_error, d_value,
                                       x,y,z,theta,lambda,increment,imgSize);

        if (error < minError)
        {
            minIdx = z;
            minError = error;
        }
    }

    a_data[image_offset] = increment*(float)minIdx;
    minIdx_data[image_offset] = minIdx;

    if (minIdx == 0 || minIdx == depthSamples-1)
        return;

    //These are photo errors, not combined errors
    float errorPlus = GetPhotoError(photo_error,x,y,minIdx+1,imgSize);
    float errorMid = GetPhotoError(photo_error,x,y,minIdx,imgSize);
    float errorMinus = GetPhotoError(photo_error,x,y,minIdx-1,imgSize);
    Vector3f param = GetQuadFit(Vector3f(errorMinus, errorMid, errorPlus));
    float a = param[0];
    float b = param[1];
    float c = param[2];

    float a_depth = a_data[image_offset];
    float u = d_value;
    float m = depthSamples - 1;
    float theta_m_m = (1 / (theta*m*m));
    float nom = (u - a_depth)/(theta*m) - lambda*b;
    float denom = 2*lambda*a + theta_m_m;
    float t_best = nom / denom;

    a_depth = a_depth + (t_best/m);
    a_data[image_offset] = a_depth;

    __syncthreads();

    int x_plus = clamp(x+1, imgSize.x);
    int y_plus = clamp(y+1, imgSize.y);

    float grad_a_x=a_data[x_plus + imgSize.x * y] - a_data[x + imgSize.x * y];
    float grad_a_y=a_data[x + imgSize.x * y_plus] - a_data[x + imgSize.x * y];

    float grad_norm = (grad_a_x*grad_a_x + grad_a_y*grad_a_y);
    float absNorm=sqrtf(grad_norm);
    float TV_Huber = HuberNorm(absNorm, eps);

    float photoError = a*t_best*t_best + b*t_best + c;
    float totalError = TV_Huber + lambda * photoError;
    error_data[image_offset] = totalError;
}


__global__ void MinErrorQuant_device(float *photo_error,float *d_data,
                                     float *a_data, int *minIdx_data,
                                     float *error_data,
                                     Vector2i imgSize,
                                     int depthSamples, float theta,
                                     float eps, float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    float minError = 9999.0f;
    int minIdx = 0;

    float increment = 1.0f / (float)(depthSamples - 1);
    float d_value = d_data[image_offset];

    for (unsigned int z = 0; z < depthSamples; z++)
    {
        float error = GetCombinedError(photo_error, d_value,
                                       x,y,z,theta,lambda,increment,imgSize);

        if (error < minError)
        {
            minIdx = z;
            minError = error;
        }
    }

    a_data[image_offset] = increment*(float)minIdx;
    minIdx_data[image_offset] = minIdx;
}




__global__ void ComputeFullError_device(float *d_data, float *error_data,
                                        float *photo_error, int *minIdx_data,
                                        float minIDepth, float maxIDepth,
                                        int depthSamples,
                                        Vector2i imgSize, float eps, float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    int x_plus = clamp(x+1, imgSize.x);
    int y_plus = clamp(y+1, imgSize.y);
    // //Gradient from matrix A

    float grad_d_x=d_data[x_plus + imgSize.x * y] - d_data[x + imgSize.x * y];
    float grad_d_y=d_data[x + imgSize.x * y_plus] - d_data[x + imgSize.x * y];

    float grad_norm = (grad_d_x*grad_d_x + grad_d_y*grad_d_y);
    float absNorm=sqrtf(grad_norm);
    float TV_Huber = HuberNorm(absNorm, eps);


    float dData = d_data[image_offset];
    float z = dData * (depthSamples - 1);
    int z1 = floor(z);
    int z2 = ceil(z);

    float dz = z - z1;
    float weight1 = (1 - dz);
    float weight2 = dz;

    int offset1 = x + y * imgSize.x+ z1* imgSize.x*imgSize.y;
    int offset2 = x + y * imgSize.x+ z2* imgSize.x*imgSize.y;
        
    // float photoErrorPix1 = photo_error[offset1];
    // float photoErrorPix2 = photo_error[offset2];

    // float photoErrorPix = photoErrorPix1 * weight1 +
    //     photoErrorPix2 * weight2;


    // int offset = x + y * imgSize.x+ (int)z* imgSize.x*imgSize.y;

    int zi = minIdx_data[image_offset];
    int offset = x + y * imgSize.x+ zi* imgSize.x*imgSize.y;

    float photoErrorPix = photo_error[offset];

    float totalError = TV_Huber + lambda * photoErrorPix;

    error_data[image_offset] = totalError;
}

__global__ void ComputeL1Error(float *d_data,
                               float *a_data,
                               float *error_data,
                               Vector2i imgSize,
                               float lambda)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    int x_plus = clamp(x+1, imgSize.x);
    int y_plus = clamp(y+1, imgSize.y);
    // //Gradient from matrix A

    float grad_d_x=d_data[x_plus + imgSize.x * y] - d_data[x + imgSize.x * y];
    float grad_d_y=d_data[x + imgSize.x * y_plus] - d_data[x + imgSize.x * y];

    float grad_norm = (grad_d_x*grad_d_x + grad_d_y*grad_d_y);
    float absNorm=sqrtf(grad_norm);

    float d = d_data[image_offset];
    float a = a_data[image_offset];

    float diff = fabs(d - a);

    float totalError = absNorm + lambda * diff;
    error_data[image_offset] = totalError;
}





MonoDepthEstimator_CUDA::MonoDepthEstimator_CUDA(Vector2i imgSize, Vector4f intrinsics_raw)
    :MonoDepthEstimator(imgSize, intrinsics_raw)
{
}

float MonoDepthEstimator_CUDA::EvaluateGT()
{
    MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;

    size_t totalSize = monoLevel->gtDepth->dataSize;

    monoLevel->depth->UpdateHostFromDevice();
    monoLevel->gtDepth->UpdateHostFromDevice();

    float *obtainedDepth = monoLevel->depth->GetData(MEMORYDEVICE_CPU);
    float *gtDepths = monoLevel->gtDepth->GetData(MEMORYDEVICE_CPU);

    float totalMeasured = 0;
    float diffTotal = 0.0f;
    for (size_t locId = 0; locId < totalSize; locId++)
    {
        float gtDepth = gtDepths[locId];
        if (gtDepth > 0.0f || gtDepth > 3.0f)
        {

            float methodDepth = obtainedDepth[locId];
            if (methodDepth > 3.0f) continue;

            diffTotal += fabs(gtDepths[locId] - methodDepth);

            totalMeasured++;
        }
    }

    return diffTotal / (float)totalMeasured;
}

void MonoDepthEstimator_CUDA::SetRefImage(ORUChar4TSImage *frame)
{
    currDepthFrame->Init();
    currDepthFrame->colorImageData->SetFrom(frame, MEMCPYDIR_CUDA_TO_CPU);

    MonoDepthEstimator::SetRefImage(frame);
}

void MonoDepthEstimator_CUDA::SetRefAndFeatureImage(ORUChar4TSImage *frame,
                                                    ORUtils::MemoryBlock<float> *featureImage)
{
    currDepthFrame->Init();
    currDepthFrame->colorImageData->SetFrom(frame, MEMCPYDIR_CUDA_TO_CPU);
    currDepthFrame->featureImageData->SetFrom(featureImage, MEMCPYDIR_CUDA_TO_CPU);

    MonoDepthEstimator::SetRefImage(frame);
}


void MonoDepthEstimator_CUDA::ReinitOptim()
{
    float *photoErrors = optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA);
    int *nUpdates = optimPyramid->nUpdates->GetData(MEMORYDEVICE_CUDA);
    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    Vector2i imgSize = currDepthFrame->colorImageData->noDims;

    dim3 blocks2=getBlocksFor2DProcess(imgSize.x ,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x ,imgSize.y);

    Init2DValues_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                                      currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CUDA),
                                                      imgSize, tvSettings.edgeAlpha, tvSettings.edgeBeta);

    InitVolumeValues_device<<<blocks2,threadsPerBlock2>>>(photoErrors,nUpdates,
                                                          imgSize, optimPyramid->depthSamples);

    monoLevel->nUpdate= 0;

}

void MonoDepthEstimator_CUDA::UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                                               ORUtils::TimeStampedImage<Vector4u> *frame)
{
    float depthIncrement = (optimPyramid->maxIDepth - optimPyramid->minIDepth) /
        ((float)optimPyramid->depthSamples -1);

    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    Vector2i imgSize = monoLevel->depth->noDims;

    dim3 blocks2=getBlocksFor2DProcess(imgSize.x ,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x ,imgSize.y);

    updatePhotoError2d<<<blocks2,threadsPerBlock2>>>(refToTracker.GetR(),
                                                     refToTracker.GetT(),
                                                     monoLevel->intrinsics,
                                                     imgSize,
                                                     optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                     optimPyramid->nUpdates->GetData(MEMORYDEVICE_CUDA),
                                                     frame->GetData(MEMORYDEVICE_CUDA),
                                                     currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CUDA),
                                                     optimPyramid->depthSamples,
                                                     optimPyramid->minIDepth,
                                                     depthIncrement);
    monoLevel->nUpdate++;

    cudaThreadSynchronize();
}

void MonoDepthEstimator_CUDA::RunTVOptimisation()
{
    InitOptim();
    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;

    Vector2i imgSize = monoLevel->depth->noDims;
    dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x, imgSize.y);


    float thetaEnd = 1e-4;
    float outerError = 0;
    float beta = 0.002;
    float theta = 0.2;
    
    while (theta > thetaEnd)
    {

        theta = theta*(1-beta);



        float sigma_q = tvSettings.sigma_q;
        float sigma_d = tvSettings.sigma_d;

        ComputeFullError_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->depthSamples,
                                                              optimPyramid->minIDepth, optimPyramid->maxIDepth,
                                                              imgSize, tvSettings.epsilon, tvSettings.lambda); 


        optimPyramid->error->UpdateHostFromDevice();
        float lastError = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);

        float invTheta = 1 / theta;

        for (unsigned int j = 0; j < 1; j++)
        {

            ComputeGradient<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                          imgSize, 
                                                          optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->dy->GetData(MEMORYDEVICE_CUDA));



            UpdateQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->dy->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                                  imgSize, sigma_q, tvSettings.epsilon);


            ComputeDivQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                      imgSize);



            UpdateD<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                                  sigma_d, invTheta, imgSize);

            ComputeFullError_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                                  optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                                  optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                                  optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                                  optimPyramid->minIDepth, optimPyramid->maxIDepth,
                                                                  optimPyramid->depthSamples,
                                                                  imgSize, tvSettings.epsilon, tvSettings.lambda); 


            optimPyramid->error->UpdateHostFromDevice();
            float error = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);


            std::cout << "Theta " << theta
                      << " OuterError " << outerError
                      << " Error: " << error
                      << " Sigma d: " << sigma_d 
                      << " Sigma q: " << sigma_q
                      << std::endl;
        }



        // MinErrorTrueFit_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
        MinErrorQuant_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                             imgSize,
                                                             optimPyramid->depthSamples, theta, tvSettings.epsilon,
                                                             tvSettings.lambda);

        optimPyramid->error->UpdateHostFromDevice();
        outerError = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
    }

    OptimToDepth(false);
}


void MonoDepthEstimator_CUDA::InitOptim()
{
    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;

    Vector2i imgSize = monoLevel->depth->noDims;
    dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x ,imgSize.y);

    MinPhotoErrorInit<<<blocks2,threadsPerBlock2>>>(optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                    imgSize, optimPyramid->depthSamples);

    InitSmoothingParameters<<<blocks2,threadsPerBlock2>>>(optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                          imgSize);


    OptimToDepth(false);
}


void MonoDepthEstimator_CUDA::SmoothDTAM()
{
    // MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    // Vector2i imgSize = monoLevel->depth->noDims;
    //update dual variable q
    // dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    // dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x, imgSize.y);

    // for (unsigned int i = 0; i < 1; i++)
    // {
    // UpdateDQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
    //                                        optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
    //                                        optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
    //                                        optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
    //                                        optimPyramid->a->GetData(MEMORYDEVICE_CUDA), imgSize,
    //                                        optimPyramid->minIDepth,optimPyramid->maxIDepth,
    //                                        tvSettings.epsilon, tvSettings.sigma_q,tvSettings.sigma_d,
    //                                        tvSettings.theta);
    // }
}

void MonoDepthEstimator_CUDA::DisplayPhotoVolume(int x, int y)
{
    optimPyramid->photoErrors->UpdateHostFromDevice();
    Vector2i imgSize = optimPyramid->g->noDims;

    for (unsigned int z = 30; z < 35; z++)
    {
        int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
        std::cout << optimPyramid->photoErrors->GetData(MEMORYDEVICE_CPU)[offset] << std::endl;
    }
}
    

__global__ void ComputeTVError_device(float *d_data, float *error_data, Vector2i
                                      imgSize, float eps)
{
    int x = blockIdx.x*blockDim.x+threadIdx.x;
    int y = blockIdx.y*blockDim.y+threadIdx.y;
    if (x > imgSize.x - 1 || y > imgSize.y - 1) return;
    int image_offset=x + y * imgSize.x;

    int x_plus = clamp(x+1, imgSize.x);
    int y_plus = clamp(y+1, imgSize.y);
    // //Gradient from matrix A

    float grad_d_x=d_data[x_plus + imgSize.x * y] - d_data[x + imgSize.x * y];
    float grad_d_y=d_data[x + imgSize.x * y_plus] - d_data[x + imgSize.x * y];

    float grad_norm = (grad_d_x*grad_d_x + grad_d_y*grad_d_y);
    float absNorm=sqrtf(grad_norm);
    float TV_Huber = HuberNorm(absNorm, eps);

    error_data[image_offset] = TV_Huber;
}


void MonoDepthEstimator_CUDA::OptimToDepth(bool useRawDepth)
{
    float *data;
    if (useRawDepth)
        data = optimPyramid->a->GetData(MEMORYDEVICE_CUDA);
    else
        data = optimPyramid->d->GetData(MEMORYDEVICE_CUDA);

    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    Vector2i imgSize = monoLevel->depth->noDims;

    dim3 blocks2=getBlocksFor2DProcess(imgSize.x ,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x ,imgSize.y);
    OptimToDepth_device<<<blocks2,threadsPerBlock2>>>(data, monoLevel->depth->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->minIDepth,optimPyramid->maxIDepth, imgSize);
}



void MonoDepthEstimator_CUDA::SmoothL1()
{

    //D is the same as u
    //A is the same as f_in
    std::cout << "Here" << std::endl;

    Vector2i imgSize = optimPyramid->d->noDims;
    dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x, imgSize.y);

    float L2=1.0;
    float tau=0.00051;
    float sigma=1.0/(L2*tau);
    float lambda = 0.8;

    for (unsigned int j = 0; j < 1200; j++)
    {

        ComputeGradient<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                      imgSize, 
                                                      optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->dy->GetData(MEMORYDEVICE_CUDA));

        UpdateQL1<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->dy->GetData(MEMORYDEVICE_CUDA),
                                                imgSize, sigma);

        UpdatePL1<<<blocks2,threadsPerBlock2>>>(optimPyramid->p->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                imgSize, sigma, lambda);

        ComputeDivQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                  imgSize);

        UpdateDL1<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                optimPyramid->p->GetData(MEMORYDEVICE_CUDA),
                                                sigma, tau, lambda, imgSize);

        ComputeL1Error<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                     optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                     optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                     imgSize, lambda);


        optimPyramid->error->UpdateHostFromDevice();
        float error = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
        std::cout << "Error: " << error << std::endl;

    }
    
    std::cout << "Here2" << std::endl;

}


void MonoDepthEstimator_CUDA::SmoothHuber()
{
    
    // InitOptim();

    //D is the same as u
    //A is the same as f_in
    std::cout << "Here" << std::endl;

    Vector2i imgSize = optimPyramid->d->noDims;
    dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x, imgSize.y);

    float lambda = 5.0f;

    float sigma_d = 0.001;
    float sigma_q = 10;
    float eps = 1e-4f;

    for (unsigned int j = 0; j < 1200; j++)
    {

        ComputeGradient<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                      imgSize, 
                                                      optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->dy->GetData(MEMORYDEVICE_CUDA));



        UpdateQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->dy->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                              imgSize, sigma_q, eps);

        // cv::Mat imOut2 = cv::Mat(imgSize.y, imgSize.x, CV_32FC1);
        // optimPyramid->dy->UpdateHostFromDevice();
        // for (int y = 0; y < imgSize.y; y++)
        //     for (int x = 0; x < imgSize.x; x++)
        //     {
        //         unsigned int index = x + imgSize.x * y;
        //         float val = optimPyramid->dy->GetData(MEMORYDEVICE_CPU)[index];
        //         imOut2.at<float>(y,x) = val;
        //     }
        // std::cout << imOut2 << std::endl;
        // exit(1);


        ComputeDivQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                  imgSize);



        UpdateD<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                              sigma_d, lambda, imgSize);


        ComputeL1Error<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                     optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                     optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                     imgSize, lambda);


        // cv::Mat imOut = cv::Mat(imgSize.y, imgSize.x, CV_8UC1);
        // optimPyramid->qx->UpdateHostFromDevice();
        // for (int y = 0; y < imgSize.y; y++)
        //     for (int x = 0; x < imgSize.x; x++)
        //     {
        //         unsigned int index = x + imgSize.x * y;
        //         float val = optimPyramid->qx->GetData(MEMORYDEVICE_CPU)[index];
        //         unsigned char pix = val * 256;
        //         imOut.at<unsigned char>(y,x) = pix;
        //     }

        // std::cout << imOut << std::endl;

        // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
        // cv::imshow( "Display window", imOut );                   // Show our image inside it.
        // cv::waitKey(0.1f);                                          // Wait for a keystroke in the window

        optimPyramid->error->UpdateHostFromDevice();
        float error = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
        std::cout << "Error: " << error << std::endl;

    }
    
    std::cout << "Here2" << std::endl;

}

void MonoDepthEstimator_CUDA::RunTVL1Optimisation()
{
    // Same as smmoth L1 except 
    InitOptim();
    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;

    Vector2i imgSize = monoLevel->depth->noDims;
    dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x, imgSize.y);

    float thetaEnd = 1;
    float outerError = 0;
    float beta = 0.002;

    optimPyramid->photoErrors->UpdateHostFromDevice();

    double totalError = 0;
    long count = 0;

    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
            for (int z = 0; z < optimPyramid->depthSamples; z++)
            {
                int offset = x + y * imgSize.x+ z* imgSize.x*imgSize.y;
                float error = optimPyramid->photoErrors->GetData(MEMORYDEVICE_CPU)[offset];
                totalError += error;

                count += 1;
            }


    totalError /= (double)count;

    float theta = 0.2;

    float L2=1.0;
    float tau=0.000051;
    float sigma=1.0/(L2*tau);
    
    while (theta > thetaEnd)
    {
        theta = theta*(1-beta);

        float invTheta = 1.0f / theta;


        ComputeFullError_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->minIDepth, optimPyramid->maxIDepth,
                                                              optimPyramid->depthSamples,
                                                              imgSize, tvSettings.epsilon, tvSettings.lambda); 


        optimPyramid->error->UpdateHostFromDevice();
        float lastError = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);

        for (unsigned int j = 0; j < 20; j++)
        {
            ComputeGradient<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                          imgSize, 
                                                          optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->dy->GetData(MEMORYDEVICE_CUDA));

            UpdateQL1<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->dy->GetData(MEMORYDEVICE_CUDA),
                                                    imgSize, sigma);

            UpdatePL1<<<blocks2,threadsPerBlock2>>>(optimPyramid->p->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                    imgSize, sigma, tvSettings.lambda);

            ComputeDivQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                      imgSize);

            UpdateDL1<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                    optimPyramid->p->GetData(MEMORYDEVICE_CUDA),
                                                    sigma, tau, tvSettings.lambda, imgSize);

            ComputeL1Error<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                         optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                         optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                         imgSize, tvSettings.lambda);


            optimPyramid->error->UpdateHostFromDevice();
            float error = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
            std::cout << "Error: " << error << "   Theta: " << theta << " / " << thetaEnd << "\n";
        }


        cv::Mat imOut = cv::Mat(imgSize.y, imgSize.x, CV_8UC1);
        optimPyramid->d->UpdateHostFromDevice();
        for (int y = 0; y < imgSize.y; y++)
            for (int x = 0; x < imgSize.x; x++)
            {
                unsigned int index = x + imgSize.x * y;
                float val = optimPyramid->d->GetData(MEMORYDEVICE_CPU)[index];
                unsigned char pix = val * 256;
                imOut.at<unsigned char>(y,x) = pix;
            }

        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "Display window", imOut );                   // Show our image inside it.
        cv::waitKey(0.1f);                                          // Wait for a keystroke in the window


        MinErrorTrueFit_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                             imgSize,
                                                             optimPyramid->depthSamples, theta, tvSettings.epsilon,
                                                             tvSettings.lambda);

        optimPyramid->error->UpdateHostFromDevice();
        outerError = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
    }

    OptimToDepth(false);
}

float MonoDepthEstimator_CUDA::MeasureError()
{
    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    Vector2i imgSize = monoLevel->depth->noDims;

    dim3 blocks2=getBlocksFor2DProcess(imgSize.x ,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x ,imgSize.y);

    ComputeFullError_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->minIDepth, optimPyramid->maxIDepth,
                                                          optimPyramid->depthSamples,
                                                          imgSize, tvSettings.epsilon, tvSettings.lambda); 

    optimPyramid->error->UpdateHostFromDevice();
    float error = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
    return error;
}

void MonoDepthEstimator_CUDA::UpdatePhotoErrorWithFeatures(ORUtils::SE3Pose refToTracker,
                                                           ORUtils::TimeStampedImage<Vector4u> *frame,
                                                           ORUtils::MemoryBlock<float> *featureImage)
{


    
    float depthIncrement = (optimPyramid->maxIDepth - optimPyramid->minIDepth) /
        ((float)optimPyramid->depthSamples -1);

    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;
    Vector2i imgSize = monoLevel->depth->noDims;

    dim3 blocks2=getBlocksFor2DProcess(imgSize.x ,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x ,imgSize.y);

    std::cout << "Updating here" << std::endl;

    updatePhotoErrorFeatures<<<blocks2,threadsPerBlock2>>>(refToTracker.GetR(),
                                                           refToTracker.GetT(),
                                                           monoLevel->intrinsics,
                                                           imgSize,
                                                           optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                           optimPyramid->nUpdates->GetData(MEMORYDEVICE_CUDA),
                                                           featureImage->GetData(MEMORYDEVICE_CUDA),
                                                           currDepthFrame->featureImageData->GetData(MEMORYDEVICE_CUDA),
                                                           optimPyramid->depthSamples,
                                                           optimPyramid->minIDepth,
                                                           depthIncrement);
    monoLevel->nUpdate++;

    cudaThreadSynchronize();
}


void MonoDepthEstimator_CUDA::RunTVOptimisationActive(float theta)
{
    MonoLib::MonoPyramidLevel *monoLevel = currDepthFrame->dataImage;

    Vector2i imgSize = monoLevel->depth->noDims;
    dim3 blocks2=getBlocksFor2DProcess(imgSize.x,imgSize.y);
    dim3 threadsPerBlock2=getThreadsFor2DProcess(imgSize.x, imgSize.y);


    float outerError = 0;
    float sigma_q = tvSettings.sigma_q;
    float sigma_d = tvSettings.sigma_d;

    ComputeFullError_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                          optimPyramid->depthSamples,
                                                          optimPyramid->minIDepth, optimPyramid->maxIDepth,
                                                          imgSize, tvSettings.epsilon, tvSettings.lambda); 


    optimPyramid->error->UpdateHostFromDevice();
    float lastError = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);

    float invTheta = 1 / theta;

    for (unsigned int j = 0; j < 1; j++)
    {

        ComputeGradient<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                      imgSize, 
                                                      optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                                      optimPyramid->dy->GetData(MEMORYDEVICE_CUDA));



        UpdateQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->dx->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->dy->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                              imgSize, sigma_q, tvSettings.epsilon);


        ComputeDivQ<<<blocks2,threadsPerBlock2>>>(optimPyramid->qx->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->qy->GetData(MEMORYDEVICE_CUDA),
                                                  optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                                  imgSize);



        UpdateD<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->divQ->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                              optimPyramid->g->GetData(MEMORYDEVICE_CUDA),
                                              sigma_d, invTheta, imgSize);

        ComputeFullError_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                              optimPyramid->minIDepth, optimPyramid->maxIDepth,
                                                              optimPyramid->depthSamples,
                                                              imgSize, tvSettings.epsilon, tvSettings.lambda); 


        optimPyramid->error->UpdateHostFromDevice();
        float error = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);


        std::cout << "Theta " << theta
                  << " OuterError " << outerError
                  << " Error: " << error
                  << " Sigma d: " << sigma_d 
                  << " Sigma q: " << sigma_q
                  << std::endl;

        MinErrorTrueFit_device<<<blocks2,threadsPerBlock2>>>(optimPyramid->photoErrors->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->d->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->a->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->minIndices->GetData(MEMORYDEVICE_CUDA),
                                                             optimPyramid->error->GetData(MEMORYDEVICE_CUDA),
                                                             imgSize,
                                                             optimPyramid->depthSamples, theta, tvSettings.epsilon,
                                                             tvSettings.lambda);

        optimPyramid->error->UpdateHostFromDevice();
        outerError = SumError(optimPyramid->error->GetData(MEMORYDEVICE_CPU), imgSize);
    }

    OptimToDepth(false);
}