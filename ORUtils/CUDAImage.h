#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>

template <typename T>
class CUDAImage
{
public:
    CUDAImage(int rows, int columns)
        rows(rows), columns(columns)
    {
        nElements = rows*columns;
        cudaMallocHost((void**)&cpuData, nElements * sizeof(T));
        cudaMalloc((void**)&cudaData, nElements * sizeof(T));
    }

    void UpdateDeviceFromHost()
    {
        cudaMemcpy(cudaData, cpuData, nElements * sizeof(T), cudaMemcpyHostToDevice);
    }

    void UpdateHostFromDevice()
    {
        cudaMemcpy(cpuData, cudaData, nElements * sizeof(T), cudaMemcpyDeviceToHost);
    }

    unsigned char *GetCPUData() const
    {
        return cpuData;
    }

    unsigned char *GetCUDAData() const
    {
        return cpuData;
    }

    void Initialise(T val)
    {
        for (unsigned int y = 0; y < rows; y++)
            for (unsigned int x = 0; x < columns; x++)
            {
                int index = x + columns*y;
                cpuData[index] = val;
            }
    }

private:
    unsigned int nElements;
    unsigned int rows, columns;

    T *cpuData;
    T *cudaData;
};
