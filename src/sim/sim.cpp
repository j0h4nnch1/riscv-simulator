#include "sim/sim.h"
#include "memory/mem.h"
#include <cstring>
#include <sys/mman.h> //mmap
#include <fcntl.h>    //file
#include <unistd.h>
#include "common/elf.h"
#include <readline/readline.h> //for input

void sim_t::load_payload(const std::string& payload, mmu_t& iv_mem){
    FILE* f = fopen(payload.c_str(), "rb");
    fseek(f, 0, SEEK_END);
    long size =  ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* ram_image = (uint8_t* )malloc(size);
    if( fread( ram_image, size, 1, f ) != 1){
        printf("Error: Could not load image.\n" );
    }
    fclose(f);
    printf("payload :%s, size: %d\n", payload.c_str(), size);
    memcpy((void*)iv_mem.guest2host(RESET_VECTOR), (void*)ram_image, size);
    printf("load dtb\n");
    free(ram_image);
    uint32_t size_dtb = sizeof(default64mbdtb);
    uint32_t pos_dtb = MEM_RANGE - size_dtb - 192; 
    memcpy(((void*)iv_mem.guest2host(pos_dtb)), default64mbdtb, size_dtb);
    printf("load payload success\n");
    iv_mem.dump_memory(RESET_VECTOR, 16);   
}

char* sim_t::get_filename(){
    return this->file;
}

void sim_t::reset(){
    printf("reset\n");
}

void sim_t::win_run(cpu_t& cpu, mmu_t& iv_mem, riscv32_cpu_state& state){
    cpu.init(state);
    printf("init success, pc:%lx\n", state.pc);
    char* str = nullptr;
    for( ; str = readline("(sim) > "); ){
        if(str==nullptr) continue;
        switch (*str){
            case 'r':
                printf("run a command\n");
                //todo run cpu
                cpu.fetch_decode_exec(iv_mem, state);
                break;
            case 'd':
                printf("dump reg\n");
                break;
            case 'h':
                printf("input r/d/h \n");
                break;
            case 'q':
                printf("exit emu\n");
                return ;
                break;
            default:
                printf("not support\n");
                break;
        }
    }
}

void sim_t::run(cpu_t& cpu, mmu_t& iv_mem){
    riscv32_cpu_state state = {};
    iv_mem.init();// init MMU, nothing for now
    printf("before load_elf:%s\n", this->get_filename());
    this->load_elf(iv_mem, this->file);
    this->win_run(cpu, iv_mem, state);
    // this->reset();
    // cpu.run(); //fetch->decode->run->updatepc
}

void sim_t::load_img(mmu_t& iv_mem){
    memcpy((void*)iv_mem.guest2host(RESET_VECTOR), (void*)(this->img), sizeof(this->img));
    uint32_t temp = iv_mem.read_host((void*)iv_mem.guest2host(RESET_VECTOR ), 4);
    printf("data in mem:%lx\n", temp);
}

std::map<std::string, uint32_t> sim_t::load_elf(mmu_t& iv_mem, const char* file){
    
    if(file == nullptr){
        printf("load default img\n");
        load_img(iv_mem);// using default
        std::map<std::string, uint32_t> symbols;
        return symbols;
    }
    if(strcmp(file, "Image") == 0){
        load_payload(file, iv_mem);
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
    program_hdr32* phdr = (program_hdr32*)(buffer + elf_header->e_phoff);
    entry_point = elf_header->e_entry;
    printf("e_entry:%lx\n", elf_header->e_entry);
    printf("e_phnum:%d\n", elf_header->e_phnum);
    //copy program to mem
    for(int i = 0; i < elf_header->e_phnum; i++){
        if((phdr[i].p_type == 1) && (phdr[i].p_memsz != 0) \
            && phdr[i].p_filesz!=0){
            printf("poffset:%d\n", phdr[i].p_offset);
            printf("before write, paddr: %p, src_addr: %p, size:%d\n", (void*)phdr[i].p_paddr, (void*)((uint8_t*)buffer + phdr[i].p_offset), phdr[i].p_filesz);
            // iv_mem.write_host((void*)iv_mem.guest2host(phdr[i].p_paddr), (uint32_t)(*((uint8_t*)buffer + phdr[i].p_offset)));
            memcpy((void*)iv_mem.guest2host(phdr[i].p_paddr), (void*)((uint8_t*)buffer + phdr[i].p_offset), phdr[i].p_filesz);
            // uint32_t temp = iv_mem.read_host((void*)iv_mem.guest2host(RESET_VECTOR), 4);
            // printf("data in mem:%lx\n", temp);
            // printf("start addr:%lx", reinterpret_cast<uint32_t*>(iv_mem.guest2host(RESET_VECTOR)));
            iv_mem.dump_memory(RESET_VECTOR, 14);
            printf("paddr:%lx, vaddr:%lx\n",phdr[i].p_paddr, phdr[i].p_vaddr);
        }
    }
    //construct symbol
    std::map<std::string, uint32_t> symbols;
    munmap((void*)buffer, size);
    return symbols;
}