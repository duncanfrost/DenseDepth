// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Image.h"
#include "Vector.h"

#ifndef __METALC__

namespace ORUtils
{
	/** \brief
	Represents images, templated on the pixel type
	*/
	template <typename T>
	class TimeStampedImage : public Image < T >
	{
	private:
		/** Image Timestamp */
		long long timestamp;

	public:
		
		long long GetTimeStamp() { return timestamp; }
		void SetTimeStamp(long long timestamp) { this->timestamp = timestamp; }

		/** Initialize an empty image of the given size, either
		on CPU only or on both CPU and GPU.
		*/
		TimeStampedImage(Vector2<int> noDims, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: Image<T>(noDims, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->timestamp = 0;
		}

		TimeStampedImage(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: Image<T>(Vector2i(0, 0), allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->timestamp = 0;
		}

		TimeStampedImage(Vector2<int> noDims, MemoryDeviceType memoryType)
			: Image<T>(noDims, memoryType)
		{
			this->timestamp = 0;
		}

		void SetFrom(const TimeStampedImage<T> *source, MemoryCopyDirection memoryCopyDirection)
		{
			this->timestamp = source->timestamp;
			MemoryBlock<T>::SetFrom(source, memoryCopyDirection);
		}

		void SetFrom(const Image<T> *source, MemoryCopyDirection memoryCopyDirection)
		{
			this->timestamp = 0;
			MemoryBlock<T>::SetFrom(source, memoryCopyDirection);
		}

		void Clear(unsigned char defaultValue = 0)
		{
			this->timestamp = 0;
			Image<T>::Clear(defaultValue);
		}

		// Suppress the default copy constructor and assignment operator
		TimeStampedImage(const TimeStampedImage&);
		TimeStampedImage& operator=(const TimeStampedImage&);
	};
}

#endif
