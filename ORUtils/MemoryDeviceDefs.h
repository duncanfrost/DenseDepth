// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

/**
 * \brief The values of this enumeration denote the different types of memory device on which code may be running.
 */
enum MemoryDeviceType
{
  MEMORYDEVICE_CPU,
  MEMORYDEVICE_CUDA
};

/**
* \brief The values of this enumeration denote the different types of memory copy directions.
*/
enum MemoryCopyDirection 
{
	MEMCPYDIR_CPU_TO_CPU, 
	MEMCPYDIR_CPU_TO_CUDA,
	MEMCPYDIR_CUDA_TO_CPU,
	MEMCPYDIR_CUDA_TO_CUDA
};