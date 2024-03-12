#include "sim/sim.h"
#include "memory/mem.h"
#include <cstring>
#include <sys/mman.h> //mmap
#include <fcntl.h>    //file
#include <unistd.h>
#include "common/elf.h"

void sim_t::load_payload(const std::string& payload){
    memcpy((void*)RESET_VECTOR, (void*)payload.c_str(), sizeof(payload));
}

char* sim_t::get_filename(){
    return this->file;
}

void sim_t::run(){
    MMU iv_mem;
    iv_mem.init();// init MMU, nothing for now
    printf("before load_elf:%s\n", this->get_filename());
    this->load_elf(iv_mem, this->file);
    // load_payload(); //load program to mem
    // cpu.run(); //fetch->decode->run->updatepc
}

void sim_t::load_img(){
    memcpy((void*)RESET_VECTOR, (void*)(this->img), sizeof(this->img));
}

std::map<std::string, uint32_t> sim_t::load_elf(MMU& iv_mem, const char* file){
    
    if(file == nullptr){
        load_img();// using default
        std::map<std::string, uint32_t> symbols;
        return symbols;
    }
    int fd = open(file, O_RDONLY);
    printf("open success\n");
    assert(fd >= 0);
    off_t size = lseek(fd, 0, SEEK_END);
    const char* buffer = (const char*)mmap(NULL, size, PROT_READ, MAP_PRIVATE, fd, 0);
    assert(buffer != MAP_FAILED);
    printf("buffer:%s\n", buffer);
    close(fd);
    //check elf file only elf32 now
    const header32* elf_header = (const header32*)buffer;
    bool iself = (elf_header->e_ident[0]==0x7f)&&(elf_header->e_ident[1]=='E') \
                &&(elf_header->e_ident[2]=='L')&&(elf_header->e_ident[3]=='F');
    bool iself32 = elf_header->e_ident[4] == 1;
    printf("iself32\n");
    assert(iself && iself32);
    //get programm section
    printf("e_phoff:%d\n", elf_header->e_phoff);
    program_hdr32* program_header = (program_hdr32*)(buffer + elf_header->e_phoff);
    entry_point = elf_header->e_entry;
    printf("e_entry:%lx\n", elf_header->e_entry);
    printf("e_phnum:%d\n", elf_header->e_phnum);
    //copy program to mem
    for(int i = 0; i < elf_header->e_phnum; i++){
        if((program_header[i].p_type == 1) && (program_header[i].p_memsz != 0) \
            && program_header[i].p_filesz!=0){
            printf("poffset:%d\n", program_header[i].p_offset);
            printf("before write, paddr: %p, src_addr: %p, size:%d\n", (void*)program_header[i].p_paddr, (void*)((uint8_t*)buffer + program_header[i].p_offset), program_header[i].p_filesz);
            // iv_mem.write((void*)program_header[i].p_paddr, (void*)((uint8_t*)buffer + program_header[i].p_offset), program_header[i].p_filesz);
            memcpy((void*)program_header[i].p_paddr, (void*)((uint8_t*)buffer + program_header[i].p_offset), program_header[i].p_filesz);
            printf("paddr:%d, vaddr:%d\n",program_header[i].p_paddr, program_header[i].p_vaddr);
        }
    }
    //construct symbol
    std::map<std::string, uint32_t> symbols;
    munmap((void*)buffer, size);
    return symbols;
}