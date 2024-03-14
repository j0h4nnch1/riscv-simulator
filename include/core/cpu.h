#ifndef __CPU_H
#define __CPU_H

#include <cstdint>
#include "register_file.h"
#include "fetch.h"
#include "decoder.h"
#include "encoding.h"
#include "memory/mem.h"
#include "register_file.h"
#include <unordered_map>
#include <memory>
#include <vector>

typedef uint64_t pc_t;
class cpu_t{
public:
    uint32_t fetch(uint32_t vaddr);
    void decode_run(riscv32_cpu_state state, instr data);
    uint32_t execute_one(uint32_t data);
    pc_t fetch_decode_exec(mmu_t& iv_mem, riscv32_cpu_state cpu);
    void register_func(decoder decode);
    void init(riscv32_cpu_state& cpu);
private:
//string, handle
    std::vector<decoder> all_inst;

    std::unordered_map<std::string, decoder> all_func;

};

#endif