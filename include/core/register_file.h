#ifndef __REG_H
#define __REG_H

#include <cstdint>
struct reg_t{
    uint32_t val;
};

typedef struct {
  struct {
    uint32_t val;
  } gpr[32];

  uint32_t pc;
} riscv32_cpu_state;
#endif