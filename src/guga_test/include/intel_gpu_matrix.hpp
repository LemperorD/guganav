#pragma once

#include <CL/cl.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

class IntelGpuMatrix
{
public:
    IntelGpuMatrix()
    {
        initOpenCL();
    }

    ~IntelGpuMatrix()
    {
        if(kernel_) clReleaseKernel(kernel_);
        if(program_) clReleaseProgram(program_);
        if(queue_) clReleaseCommandQueue(queue_);
        if(context_) clReleaseContext(context_);
    }

    void runBenchmark(int N = 1024)
    {
        const size_t bytes = N * N * sizeof(float);

        std::vector<float> A(N * N, 1.0f);
        std::vector<float> B(N * N, 2.0f);
        std::vector<float> C(N * N, 0.0f);

        cl_int err;

        cl_mem dA =
            clCreateBuffer(context_,
                           CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                           bytes,
                           A.data(),
                           &err);

        cl_mem dB =
            clCreateBuffer(context_,
                           CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                           bytes,
                           B.data(),
                           &err);

        cl_mem dC =
            clCreateBuffer(context_,
                           CL_MEM_WRITE_ONLY,
                           bytes,
                           nullptr,
                           &err);

        clSetKernelArg(kernel_, 0, sizeof(cl_mem), &dA);
        clSetKernelArg(kernel_, 1, sizeof(cl_mem), &dB);
        clSetKernelArg(kernel_, 2, sizeof(cl_mem), &dC);
        clSetKernelArg(kernel_, 3, sizeof(int), &N);

        size_t global[2] =
        {
            static_cast<size_t>(N),
            static_cast<size_t>(N)
        };

        uint64_t count = 0;

        auto t0 = std::chrono::steady_clock::now();

        while(running_)
        {
            err = clEnqueueNDRangeKernel(
                queue_,
                kernel_,
                2,
                nullptr,
                global,
                nullptr,
                0,
                nullptr,
                nullptr);

            clFinish(queue_);

            count++;

            auto t1 = std::chrono::steady_clock::now();

            if(std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count() >= 1)
            {
                std::cout
                    << "Iterations: "
                    << count
                    << std::endl;

                t0 = t1;
            }
        }

        clEnqueueReadBuffer(
            queue_,
            dC,
            CL_TRUE,
            0,
            bytes,
            C.data(),
            0,
            nullptr,
            nullptr);

        std::cout
            << "Result C[0] = "
            << C[0]
            << std::endl;

        clReleaseMemObject(dA);
        clReleaseMemObject(dB);
        clReleaseMemObject(dC);
    }

    static void stop()
    {
        running_ = false;
    }

private:

    void initOpenCL()
    {
        cl_platform_id platform;
        cl_uint num_platforms;

        clGetPlatformIDs(
            1,
            &platform,
            &num_platforms);

        clGetDeviceIDs(
            platform,
            CL_DEVICE_TYPE_GPU,
            1,
            &device_,
            nullptr);

        char device_name[256];

        clGetDeviceInfo(
            device_,
            CL_DEVICE_NAME,
            sizeof(device_name),
            device_name,
            nullptr);

        std::cout
            << "Using GPU: "
            << device_name
            << std::endl;

        context_ =
            clCreateContext(
                nullptr,
                1,
                &device_,
                nullptr,
                nullptr,
                nullptr);

        queue_ =
            clCreateCommandQueue(
                context_,
                device_,
                0,
                nullptr);

        const char* kernel_source = R"(
        __kernel void matmul(
            __global const float* A,
            __global const float* B,
            __global float* C,
            int N)
        {
            int row = get_global_id(0);
            int col = get_global_id(1);

            float sum = 0.0f;

            for(int k=0;k<N;k++)
            {
                sum += A[row*N+k] * B[k*N+col];
            }

            C[row*N+col] = sum;
        }
        )";

        cl_int err;

        program_ =
            clCreateProgramWithSource(
                context_,
                1,
                &kernel_source,
                nullptr,
                &err);

        err =
            clBuildProgram(
                program_,
                1,
                &device_,
                nullptr,
                nullptr,
                nullptr);

        if(err != CL_SUCCESS)
        {
            size_t log_size;

            clGetProgramBuildInfo(
                program_,
                device_,
                CL_PROGRAM_BUILD_LOG,
                0,
                nullptr,
                &log_size);

            std::string log(log_size, '\0');

            clGetProgramBuildInfo(
                program_,
                device_,
                CL_PROGRAM_BUILD_LOG,
                log_size,
                log.data(),
                nullptr);

            std::cerr << log << std::endl;

            throw std::runtime_error("OpenCL build failed");
        }

        kernel_ =
            clCreateKernel(
                program_,
                "matmul",
                &err);
    }

private:

    cl_device_id device_{};
    cl_context context_{};
    cl_command_queue queue_{};
    cl_program program_{};
    cl_kernel kernel_{};

    inline static std::atomic<bool> running_{true};
};