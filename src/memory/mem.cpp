#include "memory/mem.h"
#include <stdio.h>


uint32_t mmu_t::host2guest(uint8_t* addr){
    return (addr - this->phymem + PADDR_BASE);
}

uint8_t* mmu_t::guest2host(uint32_t addr){
    return (uint8_t*)(this->phymem + addr - PADDR_BASE);
}

uint32_t mmu_t::read_host(void* addr, uint32_t len){
    switch (len){
    case 1:
        return *(uint8_t*)addr;
    case 2:
        return *(uint16_t*)addr;
    case 4:
        return *(uint32_t*)addr;
    default:
        printf("not support len\n");
        break;
    }
    return 0;
}

template <typename T>
void mmu_t::write_host(void* addr, const T& data){
    uint32_t size = sizeof(data);
    switch (size){
        case 1:
            (uint8_t*)addr = data;
            break;
        case 2:
            (uint16_t*)addr = data;
            break;
        case 4:
            (uint32_t*)addr = data;
            break;
        default:
            break;
            printf("not support len\n");
    }
}

void mmu_t::init(){}

void mmu_t::dump_memory(uint32_t start, uint32_t size){
    // iv_mem.read_host((void*)iv_mem.guest2host(RESET_VECTOR + 4), 4);
    printf("----dump memory start----\n");
    for(int i = 0; i < size; ++i){
        uint32_t temp = read_host((void*)guest2host(RESET_VECTOR + 4 * i), 4);
        printf("%08x\n", temp);
    }
    printf("----dump memory end----\n");
}