#include "intel_gpu_matrix.hpp"

#include <csignal>

void signalHandler(int)
{
    IntelGpuMatrix::stop();
}

int main()
{
    signal(SIGINT, signalHandler);

    IntelGpuMatrix gpu;

    gpu.runBenchmark(1024);

    return 0;
}