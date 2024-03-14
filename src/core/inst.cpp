#include "core/instruction_list.h"

uint32_t func_addi(riscv32_cpu_state cpu, instr inst){
    printf(">%s\n", __FUNCTION__);
    cpu.gpr[inst.i.rd].val = cpu.gpr[inst.i.rs1].val + inst.i.simm11_0;
    return 0;
}