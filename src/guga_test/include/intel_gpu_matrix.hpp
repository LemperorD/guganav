#pragma once

#include <CL/cl.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Intel GPU 矩阵乘法基准测试类
 *
 * 使用 OpenCL 框架在 Intel 集成显卡上执行大规模矩阵乘法 (N×N)，
 * 并持续运行以评估 GPU 的计算吞吐量。
 *
 * 使用方式：
 * @code
 *   IntelGpuMatrix bench;
 *   bench.runBenchmark(1024);  // 1024×1024 矩阵乘法
 * @endcode
 *
 * 通过静态方法 stop() 可在其他线程中优雅终止基准测试循环。
 *
 * @note 仅支持 Intel GPU (CL_DEVICE_TYPE_GPU)，若平台无 Intel GPU
 *       OpenCL 设备则初始化阶段会失败。
 * @note 构造函数自动调用 initOpenCL()，若设备不可用会抛异常。
 */
class IntelGpuMatrix
{
public:
    /**
     * @brief 构造函数，自动初始化 OpenCL 运行环境
     *
     * 执行流程：
     * 1. 发现系统中的 Intel GPU 平台
     * 2. 获取 GPU 设备并创建 OpenCL 上下文
     * 3. 创建命令队列
     * 4. 编译内嵌的矩阵乘法 Kernel 源码
     *
     * @throws std::runtime_error 当 OpenCL Kernel 编译失败时抛出
     */
    IntelGpuMatrix()
    {
        initOpenCL();
    }

    /**
     * @brief 析构函数，释放所有 OpenCL 资源
     *
     * 按创建的反序释放：Kernel → Program → CommandQueue → Context，
     * 避免资源泄漏。每个资源在释放前检查非空。
     */
    ~IntelGpuMatrix()
    {
        if (kernel_) clReleaseKernel(kernel_);
        if (program_) clReleaseProgram(program_);
        if (queue_) clReleaseCommandQueue(queue_);
        if (context_) clReleaseContext(context_);
    }

    /**
     * @brief 运行矩阵乘法基准测试
     *
     * 在 GPU 上持续执行 N×N 大小的矩阵乘法 C = A × B，
     * 每秒输出一次完成的迭代次数，直到 running_ 标志被置为 false。
     *
     * 测试使用的数据：
     * - 矩阵 A: 所有元素为 1.0f (N×N)
     * - 矩阵 B: 所有元素为 2.0f (N×N)
     * - 矩阵 C: 输出结果，初始化为 0.0f
     *
     * @param N 矩阵的维度 (N×N)，默认 1024
     *
     * @note 主循环受 running_ 原子变量控制，可通过 stop() 从其他线程终止。
     * @note 循环结束后会回读结果到主机端，打印 C[0] 用于正确性验证。
     * @note 理论上 C[0] 应为 2.0 × N (每个元素 1×2 累加 N 次)。
     */
    void runBenchmark(int N = 1024)
    {
        // 计算矩阵数据的总字节数 (N*N 个 float)
        const size_t bytes = N * N * sizeof(float);

        // 初始化主机端矩阵数据
        // A: 全 1 矩阵, B: 全 2 矩阵, C: 全 0 矩阵 (待填充结果)
        std::vector<float> A(N * N, 1.0f);
        std::vector<float> B(N * N, 2.0f);
        std::vector<float> C(N * N, 0.0f);

        cl_int err;

        // ============================================================
        // 步骤 1: 创建 GPU 端缓冲区 (Device Memory)
        // ============================================================

        // dA: 只读缓冲区，创建时将主机数据拷贝到设备端
        cl_mem dA =
            clCreateBuffer(context_,
                           CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                           bytes,
                           A.data(),
                           &err);

        // dB: 只读缓冲区，创建时将主机数据拷贝到设备端
        cl_mem dB =
            clCreateBuffer(context_,
                           CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                           bytes,
                           B.data(),
                           &err);

        // dC: 只写缓冲区，无需初始化 (结果将由 Kernel 写入)
        cl_mem dC =
            clCreateBuffer(context_,
                           CL_MEM_WRITE_ONLY,
                           bytes,
                           nullptr,
                           &err);

        // ============================================================
        // 步骤 2: 设置 Kernel 参数
        // ============================================================
        // 参数绑定顺序必须与 Kernel 函数签名一致:
        // matmul(__global const float* A, __global const float* B,
        //        __global float* C, int N)
        clSetKernelArg(kernel_, 0, sizeof(cl_mem), &dA);  // 参数0: 矩阵 A
        clSetKernelArg(kernel_, 1, sizeof(cl_mem), &dB);  // 参数1: 矩阵 B
        clSetKernelArg(kernel_, 2, sizeof(cl_mem), &dC);  // 参数2: 结果矩阵 C
        clSetKernelArg(kernel_, 3, sizeof(int), &N);       // 参数3: 矩阵维度 N

        // ============================================================
        // 步骤 3: 定义全局工作项数量
        // ============================================================
        // 二维工作空间: 每个工作项计算 C 的一个元素
        // global[0] = N 行, global[1] = N 列 → 共 N×N 个工作项
        size_t global[2] =
        {
            static_cast<size_t>(N),  // 行方向工作项数
            static_cast<size_t>(N)   // 列方向工作项数
        };

        uint64_t count = 0;  // 迭代计数器

        auto t0 = std::chrono::steady_clock::now();  // 计时起点

        // ============================================================
        // 步骤 4: 主基准测试循环
        // ============================================================
        // 持续向 GPU 提交 Kernel 执行，每秒打印一次迭代次数
        // running_ 是原子变量，其他线程可通过 stop() 将其置 false
        while (running_)
        {
            // 将 Kernel 入队到命令队列，在 GPU 上异步执行
            err = clEnqueueNDRangeKernel(
                queue_,      // 命令队列
                kernel_,     // 要执行的 Kernel
                2,           // 工作维度 = 2 (二维 NDRange)
                nullptr,     // 全局偏移 (从0开始)
                global,      // 全局工作项数量 {N, N}
                nullptr,     // 局部工作组大小 (由 OpenCL 自动决定)
                0,           // 事件等待列表长度
                nullptr,     // 事件等待列表
                nullptr);    // 输出事件

            // 阻塞等待队列中所有命令执行完毕，确保本次 Kernel 完成
            clFinish(queue_);

            count++;  // 成功完成一次迭代

            // 每秒输出一次吞吐量统计
            auto t1 = std::chrono::steady_clock::now();

            if (std::chrono::duration_cast<std::chrono::seconds>(t1 - t0)
                    .count() >= 1)
            {
                std::cout
                    << "Iterations: "  // 每秒迭代次数 ≈ GPU 吞吐量
                    << count
                    << std::endl;

                t0 = t1;  // 重置计时起点
            }
        }

        // ============================================================
        // 步骤 5: 回读计算结果 (正确性验证)
        // ============================================================
        // 将 GPU 端的计算结果 dC 拷贝回主机端 C
        clEnqueueReadBuffer(
            queue_,       // 命令队列
            dC,           // 设备端源缓冲区
            CL_TRUE,      // 阻塞读取 (等待传输完成)
            0,            // 偏移量
            bytes,        // 读取字节数
            C.data(),     // 主机端目标缓冲区
            0,            // 事件等待列表长度
            nullptr,      // 事件等待列表
            nullptr);     // 输出事件

        // 验证: C[0] 理论值 = N × (1.0 × 2.0) = 2N
        std::cout
            << "Result C[0] = "
            << C[0]
            << std::endl;

        // ============================================================
        // 步骤 6: 释放 GPU 缓冲区资源
        // ============================================================
        clReleaseMemObject(dA);
        clReleaseMemObject(dB);
        clReleaseMemObject(dC);
    }

    /**
     * @brief 停止正在运行的基准测试
     *
     * 设置原子标志 running_ 为 false，使 runBenchmark() 中的
     * while 循环在下一次迭代时退出。
     *
     * 该方法为静态方法，可在任意线程中调用，线程安全。
     *
     * @note 典型用法是在信号处理函数中调用:
     * @code
     *   signal(SIGINT, [](int){ IntelGpuMatrix::stop(); });
     * @endcode
     */
    static void stop()
    {
        running_ = false;
    }

private:
    /**
     * @brief 初始化 OpenCL 运行环境
     *
     * 按以下顺序完成 OpenCL 初始化:
     * 1. 获取系统中的 OpenCL 平台 (Intel GPU)
     * 2. 获取 GPU 设备并打印设备名称
     * 3. 创建 OpenCL 上下文
     * 4. 创建命令队列 (用于向 GPU 提交命令)
     * 5. 编译内嵌的矩阵乘法 Kernel 源码
     *
     * @throws std::runtime_error 当 Kernel 编译失败时抛出，
     *         错误日志会打印到 stderr
     *
     * @note Kernel 源码使用 OpenCL C 语言编写，内嵌在 C++ 原始字符串中。
     *       Kernel 函数 matmul 采用朴素矩阵乘法算法，
     *       每个工作项计算输出矩阵的一个元素。
     */
    void initOpenCL()
    {
        cl_platform_id platform;
        cl_uint num_platforms;

        // ----------------------------------------------------------
        // 步骤 1: 获取 OpenCL 平台
        // ----------------------------------------------------------
        // 获取第一个可用的 OpenCL 平台 (通常为 Intel GPU 平台)
        clGetPlatformIDs(
            1,               // 期望获取的平台数量
            &platform,       // 输出: 平台句柄
            &num_platforms); // 输出: 实际可用平台数量

        // ----------------------------------------------------------
        // 步骤 2: 获取 GPU 设备
        // ----------------------------------------------------------
        // 从平台中查找 GPU 类型的设备 (集成显卡)
        clGetDeviceIDs(
            platform,                // 所属平台
            CL_DEVICE_TYPE_GPU,      // 设备类型: GPU
            1,                       // 期望获取的设备数量
            &device_,                // 输出: 设备句柄 (存入成员变量)
            nullptr);                // 实际设备数量 (不需要)

        // 打印 GPU 设备名称 (如 "Intel(R) UHD Graphics 630")
        char device_name[256];

        clGetDeviceInfo(
            device_,              // 设备句柄
            CL_DEVICE_NAME,       // 查询属性: 设备名称
            sizeof(device_name),  // 缓冲区大小
            device_name,          // 输出缓冲区
            nullptr);             // 实际返回大小 (不需要)

        std::cout
            << "Using GPU: "
            << device_name
            << std::endl;

        // ----------------------------------------------------------
        // 步骤 3: 创建 OpenCL 上下文
        // ----------------------------------------------------------
        // 上下文是 OpenCL 的核心对象，管理设备、内存、程序等资源
        context_ =
            clCreateContext(
                nullptr,   // 上下文属性 (使用默认)
                1,         // 设备数量
                &device_,  // 设备列表
                nullptr,   // 错误回调函数
                nullptr,   // 回调用户数据
                nullptr);  // 错误码 (不需要)

        // ----------------------------------------------------------
        // 步骤 4: 创建命令队列
        // ----------------------------------------------------------
        // 命令队列用于向 GPU 提交 Kernel 执行、内存传输等命令
        queue_ =
            clCreateCommandQueue(
                context_,   // 所属上下文
                device_,    // 目标设备
                0,          // 队列属性 (0 = 默认, 按序执行)
                nullptr);   // 错误码 (不需要)

        // ----------------------------------------------------------
        // 步骤 5: 创建并编译 Kernel 程序
        // ----------------------------------------------------------
        // OpenCL C Kernel 源码: 朴素矩阵乘法
        // 每个工作项 (global_id[0], global_id[1]) 计算 C 的一个元素
        // C[row][col] = Σ(A[row][k] × B[k][col])  for k=0..N-1
        const char* kernel_source = R"(
        __kernel void matmul(
            __global const float* A,  // 输入矩阵 A (N×N, 行主序)
            __global const float* B,  // 输入矩阵 B (N×N, 行主序)
            __global float* C,        // 输出矩阵 C (N×N, 行主序)
            int N)                    // 矩阵维度
        {
            // 获取当前工作项的全局索引
            int row = get_global_id(0);  // 行号 (第0维)
            int col = get_global_id(1);  // 列号 (第1维)

            float sum = 0.0f;

            // 内积计算: A 的第 row 行 × B 的第 col 列
            for(int k=0;k<N;k++)
            {
                sum += A[row*N+k] * B[k*N+col];
            }

            // 将结果写入输出矩阵对应位置
            C[row*N+col] = sum;
        }
        )";

        cl_int err;

        // 从源码创建 Program 对象
        program_ =
            clCreateProgramWithSource(
                context_,        // 所属上下文
                1,               // 源码字符串数量
                &kernel_source,  // 源码字符串数组
                nullptr,         // 各字符串长度 (nullptr = 自动计算)
                &err);           // 错误码

        // 编译 Program (针对目标 GPU 设备)
        err =
            clBuildProgram(
                program_,   // 待编译的程序
                1,          // 设备数量
                &device_,   // 设备列表
                nullptr,    // 编译选项 (如 "-cl-fast-relaxed-math")
                nullptr,    // 编译完成回调
                nullptr);   // 回调用户数据

        // 编译失败时获取并打印错误日志，抛出异常
        if (err != CL_SUCCESS)
        {
            size_t log_size;

            // 先查询日志长度
            clGetProgramBuildInfo(
                program_,
                device_,
                CL_PROGRAM_BUILD_LOG,  // 查询编译日志
                0,
                nullptr,
                &log_size);

            std::string log(log_size, '\0');

            // 获取编译日志内容
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

        // 从编译好的 Program 中提取 Kernel 入口函数
        kernel_ =
            clCreateKernel(
                program_,    // 已编译的程序
                "matmul",    // Kernel 函数名 (与源码中保持一致)
                &err);       // 错误码
    }

private:
    // ================================================================
    // OpenCL 核心对象 (成员变量)
    // ================================================================

    cl_device_id device_{};         ///< OpenCL GPU 设备句柄
    cl_context context_{};          ///< OpenCL 上下文 (管理设备、内存等)
    cl_command_queue queue_{};      ///< OpenCL 命令队列 (提交 GPU 任务)
    cl_program program_{};          ///< OpenCL 程序对象 (编译后的 Kernel)
    cl_kernel kernel_{};            ///< OpenCL Kernel 句柄 (可执行的 GPU 函数)

    // ================================================================
    // 静态控制变量
    // ================================================================

    /**
     * @brief 基准测试运行控制标志
     *
     * inline static 使其在类内定义即可 (C++17)，
     * std::atomic<bool> 保证跨线程的读写安全性。
     *
     * 初始值为 true (运行状态)，通过 stop() 设置为 false (停止)。
     */
    inline static std::atomic<bool> running_{true};
};
