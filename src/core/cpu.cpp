#include "core/cpu.h"
#include "memory/mem.h"
#include "core/decoder.h"

void cpu_t::decode_run(riscv32_cpu_state state, instr data){
    printf("%s, data:%lx\n", __FUNCTION__, data.val);
    for(auto it : all_inst){
        if((data.val&it._mask)==it._match){
            it.func(state, data);
        }
    }
}

void cpu_t::register_func(decoder decode){
    all_inst.push_back(decode);
}

void cpu_t:: init(riscv32_cpu_state& cpu){
    //zero reg x0, and set pc
    
    cpu.gpr[0].val = 0;
    cpu.pc = RESET_VECTOR;

#define DECLARE_INSN(name, match, mask) \
    uint32_t name##_match = (match); \
    uint32_t name##_mask = (mask); \
    extern uint32_t func_##name(riscv32_cpu_state, instr); \

#include "core/encoding.h"
#undef DECLARE_INSN

#define DECLARE_FUNC(name) \
    register_func((decoder){ \
        name##_mask, \
        name##_match, \
        func_##name \
    });
#include "core/instruction_list.h"

#undef DECLARE_FUNC
}
pc_t cpu_t::fetch_decode_exec(mmu_t& iv_mem, riscv32_cpu_state cpu){
    //run once
    printf(">%s \n", __FUNCTION__);
    instr isn;
    isn.val = iv_mem.fetch(cpu.pc);
    decode_run(cpu, isn);
    cpu.pc += 4;
    return cpu.pc;
}