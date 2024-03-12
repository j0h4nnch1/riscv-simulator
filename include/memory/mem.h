#ifndef __MEM_H
#define __MEM_H

#include <cstdint>

#define VADDR_BASE 0x80000000
#define PADDR_BASE 0x80000000
#define RESET_VECTOR VADDR_BASE
class MMU{
public:
    uint32_t vaddr2paddr(uint32_t vaddr, uint32_t paddr){
        paddr = vaddr;
        return paddr;
    }
    uint32_t paddr2vaddr(uint32_t vaddr, uint32_t paddr){
        vaddr = paddr;
        return vaddr;
    }
    void* write(void* dest, const void* src, uint32_t size){
        char* dest_char = (char*)dest;
        const char* src_char = (const char*)src;
        for(uint32_t i = 0; i < size; i++){
            dest_char[i] = src_char[i];
        }
        return dest;
    }
    void* read(){

    }
    void init(){
        //todo MMU func
    };
    
private:

};

#endif