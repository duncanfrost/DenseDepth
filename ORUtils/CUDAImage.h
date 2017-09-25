#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>

class CUDAImage
{
public:

    CUDAImage(int rows, int columns)
    {
        nElements = rows*columns;
        cudaMallocHost((void**)&cpuData, nElements * sizeof(unsigned char));
        cudaMalloc((void**)&cudaData, nElements * sizeof(unsigned char));
    }

    void UpdateDeviceFromHost()
    {
        cudaMemcpy(cudaData, cpuData, nElements * sizeof(unsigned char), cudaMemcpyHostToDevice);
    }

    void UpdateHostFromDevice()
    {
        cudaMemcpy(cudaData, cpuData, nElements * sizeof(unsigned char), cudaMemcpyHostToDevice);
        cudaMemcpy(cpuData, cudaData, nElements * sizeof(unsigned char), cudaMemcpyDeviceToHost);
    }


private:

    unsigned int nElements;

    unsigned char *cpuData;
    unsigned char *cudaData;

};
