#include <hidden_code.h>

#include <oneapi/tbb.h>

#include <iostream>

/**
 * @brief 用于在流水线中传递的数据
 *
 * 因为 pipeline 每一级只能传递一个对象，
 * 所以这里把获取的数据封装成一个结构体。
 */
struct PipelineData
{
    GotData_t got_data;                    // 原始数据
    ProcessFuncOutput_t processed_result;  // 处理结果
};

int main()
{
    // fetch环节
    std::cout << "Hardware concurrency: " << std::thread::hardware_concurrency() << std::endl;
    std::cout << "Default concurrency: " << oneapi::tbb::info::default_concurrency() << std::endl;

    // 初始化 TBB
    oneapi::tbb::global_control control(
        oneapi::tbb::global_control::max_allowed_parallelism,
        std::thread::hardware_concurrency());

    /**
     * 最大允许同时存在多少个任务(Token)
     *
     * token越大，同时运行的processFunc越多。
     *
     * 一般可以设置：
     *      CPU核心数
     * 或者：
     *      CPU核心数 × 2
     */
    const size_t max_token = 10;

    /**
     * 建立流水线
     */
    oneapi::tbb::parallel_pipeline(
        max_token,

        /**********************************************************
         * 第一阶段：获取数据（串行）
         **********************************************************/
        oneapi::tbb::make_filter<void, PipelineData*>(

            // serial_in_order 表示严格按照顺序执行
            oneapi::tbb::filter_mode::serial_in_order,

            [](tbb::flow_control& fc) -> PipelineData*
            {
                // 从隐藏代码获取数据
                GotData_t got_data = HiddenCode::getData();

                // 创建一个流水线对象
                PipelineData* data = new PipelineData;

                data->got_data = got_data;

                return data;
            })

        &

        /**********************************************************
         * 第二阶段：并行处理（最耗时）
         **********************************************************/
        oneapi::tbb::make_filter<PipelineData*, PipelineData*>(

            // parallel表示多个线程同时执行
            oneapi::tbb::filter_mode::parallel,

            [](PipelineData* data) -> PipelineData*
            {
                // 调用真正耗时的算法
                data->processed_result =
                    HiddenCode::processFunc(
                        data->got_data.data);

                return data;
            })

        &

        /**********************************************************
         * 第三阶段：提交数据（串行）
         **********************************************************/
        oneapi::tbb::make_filter<PipelineData*, void>(

            oneapi::tbb::filter_mode::serial_in_order,

            [](PipelineData* data)
            {
                ToCommitData_t commit_data;

                // 必须保持 index 对应
                commit_data.index = data->got_data.index;

                commit_data.data = data->processed_result;

                // 提交结果
                HiddenCode::commitData(commit_data);

                // 释放内存
                delete data;
            })
    );
}