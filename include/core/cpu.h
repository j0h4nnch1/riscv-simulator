#ifndef __CPU_H
#define __CPU_H

#include <cstdint>
#include "register_file.h"

class cpu{
public:
    cpu(uint32_t pc):pc(pc){}
    uint32_t fetch(uint32_t vaddr);
    uint32_t decode(uint32_t data);
    uint32_t execute_one(uint32_t data);
private:
    uint32_t pc;
    reg_t reg;
};


#endif