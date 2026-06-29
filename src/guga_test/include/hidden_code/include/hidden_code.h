#include <cstdint>

struct ProcessFuncInput_t {
    uint8_t data[4];
};

struct ProcessFuncOutput_t {
    uint8_t data[4];
};

struct GotData_t {
    int index;
    ProcessFuncInput_t data;
};

struct ToCommitData_t {
    int index;
    ProcessFuncOutput_t data;
};

namespace HiddenCode {

GotData_t getData();

ProcessFuncOutput_t processFunc(ProcessFuncInput_t process_func_input);

void commitData(ToCommitData_t committed_data);

}
