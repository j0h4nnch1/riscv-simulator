#ifndef __SIM_H
#define __SIM_H

#include "../memory/mem.h"
// #include "common/option_parser.h"
#include <string>
#include <map>
#include <assert.h>
#include <vector>

class option_parser_t;

class sim_t{
private:
    uint32_t entry_point;
    std::map<std::string, uint32_t> symbols;
    // std::vector<uint32_t> img;
    const uint32_t img[5] = {
        0x00000297,  // auipc t0,0
        0x0002b823,  // sd  zero,16(t0)
        0x0102b503,  // ld  a0,16(t0)
        0x0000006b,  // nemu_trap
        0xdeadbeef,  // some data
    };
    void load_payload(const std::string &payload);
    
public:
    sim_t(){}
    char* file = "dummy-riscv32-nemu.elf";

    void run();
    void load_img();
    std::map<std::string, uint32_t> load_elf(MMU& iv_mem, const char* file);
    char* get_filename();
    friend class option_paser_t;
};

#endif