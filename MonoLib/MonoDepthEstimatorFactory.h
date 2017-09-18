// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MonoDepthEstimator.h"
#include "MonoDepthEstimator_CPU.h"
#ifdef COMPILE_WITH_CUDA
#include "MonoDepthEstimator_CUDA.h"
#endif

#include <string>

namespace MonoLib
{
    /**
     * \brief This struct provides functions that can be used to construct view builders.
     */
    struct MonoDepthEstimatorFactory
    {
        //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

        /**
         * \brief Makes a MonoDepthEstimator.
         *
         * \param imgSize     The size of the input rgb image.
         * \param intrinsics  The intrinsic parameters of the RGB camera.
         * \param deviceType  The device on which the view builder should operate.
         */
        static MonoDepthEstimator *MakeMonoDepthEstimator(Vector2i imgSize,
                                                          const Vector4f& intrinsics,
                                                          std::string deviceType)
        {
            MonoDepthEstimator *monoDepthEstimator = NULL;

            // if (deviceType == "CPU") monoDepthEstimator =
                                         // new MonoDepthEstimator_CPU(imgSize, intrinsics);
#ifdef COMPILE_WITH_CUDA
            if (deviceType == "CUDA") monoDepthEstimator =
                                          new MonoDepthEstimator_CUDA(imgSize, intrinsics);
#endif
#ifdef COMPILE_WITH_METAL
            if (deviceType == "Metal") monoDepthEstimator =
                                           new MonoDepthEstimator_CPU(imgSize, intrinsics);
#endif

            return monoDepthEstimator;
        }
    };
}
