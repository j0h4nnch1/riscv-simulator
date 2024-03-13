#ifndef __MEM_H
#define __MEM_H

#include <cstdint>

#define VADDR_BASE 0x80000000
#define PADDR_BASE 0x80000000
#define RESET_VECTOR VADDR_BASE
#define MEM_SIZE 0x8000000

class mmu_t{
public:
    mmu_t(){
        this->phymem = new uint8_t[MEM_SIZE];
    }
    uint32_t host2guest(uint8_t* addr);
    uint8_t* guest2host(uint32_t addr);
    /**
     * addr: host addr
     * len : read len 1/2/4 byte
    */
    uint32_t read_host(void* addr, uint32_t len);
    /**
     * addr: host addr
     * 
    */
    template <typename T>
    void write_host(void* addr, const T& data);
    void init();
    void dump_memory(uint32_t start, uint32_t size);
    ~mmu_t(){
        delete phymem;
    }
private:
    uint8_t* phymem;
};

#endif