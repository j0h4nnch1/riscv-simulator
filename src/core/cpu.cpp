#include "core/cpu.h"
#include "memory/mem.h"
#include "core/decoder.h"
bool first_time = false;
uint32_t cpu_t::decode_run(riscv32_cpu_state& state, instr data, mmu_t& iv_mem, uint32_t time_sed){
    // printf("pc : %08x, data : [0x%08x]\n", state.pc, data.val);
#if 0
    for(auto it : all_inst){
        if((data.val&it._mask)==it._match){
            it.func(state, data);
        }
    }
#endif

	uint32_t new_timer = CSR( timerl ) + time_sed;
	if( new_timer < CSR( timerl ) ) CSR( timerh )++;
	CSR( timerl ) = new_timer;

	// Handle Timer interrupt.
	if( ( CSR( timerh ) > CSR( timermatchh ) || ( CSR( timerh ) == CSR( timermatchh ) && CSR( timerl ) > CSR( timermatchl ) ) ) && ( CSR( timermatchh ) || CSR( timermatchl ) ) )
	{
		CSR( extraflags ) &= ~4; // Clear WFI
		CSR( mip ) |= 1<<7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
	}
	else
		CSR( mip ) &= ~(1<<7);
    
    if(state.extraflags & 4 == 1){
        printf("WFI!\n");
        return 1;
        //WFI cpu should not run when wait for interpert
    }

    uint32_t ir = data.val;
    uint32_t trap = 0, pre_rd = 0, cycle = 0;
    uint32_t rd_idx = (ir >> 7) & 0x1f;

	if( ( CSR( mip ) & (1<<7) ) && ( CSR( mie ) & (1<<7) /*mtie*/ ) && ( CSR( mstatus ) & 0x8 /*mie*/) )
	{
		// Timer interrupt.
		trap = 0x80000007;
		state.pc -= 4;
	}
    else{

    switch( ir & 0x7f )
    {
        case 0x37: // LUI (0b0110111)
        {
            // printf("lui\n");
            pre_rd = ( ir & 0xfffff000 );
            break;
        }
        case 0x17: // AUIPC (0b0010111)
        {
            // printf("auipc\n");
            pre_rd = state.pc + ( ir & 0xfffff000 );
            break;
        }
        case 0x6F: // JAL (0b1101111)
        {
            // printf("jal\n");
            int32_t reladdy = ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000));
            if( reladdy & 0x00100000 ) reladdy |= 0xffe00000; // Sign extension.
            pre_rd = state.pc + 4;
            state.pc = state.pc + reladdy - 4;
            break;
        }
        case 0x67: // JALR (0b1100111)
        {
            // printf("jalr\n");
            uint32_t imm = ir >> 20;
            int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
            pre_rd = state.pc + 4;
            state.pc = ( (state.gpr[( (ir >> 15) & 0x1f )].val + imm_se) & ~1) - 4;
            break;
        }
        case 0x63: // Branch (0b1100011)
        {
            // printf("branch\n");
            uint32_t immm4 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
            if( immm4 & 0x1000 ) immm4 |= 0xffffe000;
            int32_t rs1 = state.gpr[(ir >> 15) & 0x1f].val;
            int32_t rs2 = state.gpr[((ir >> 20) & 0x1f)].val;
            immm4 = state.pc + immm4 - 4;
            rd_idx = 0;
            switch( ( ir >> 12 ) & 0x7 )
            {
                // BEQ, BNE, BLT, BGE, BLTU, BGEU
                case 0: if( rs1 == rs2 ) state.pc = immm4; break;
                case 1: if( rs1 != rs2 ) state.pc = immm4; break;
                case 4: if( rs1 < rs2 ) state.pc = immm4; break;
                case 5: if( rs1 >= rs2 ) state.pc = immm4; break; //BGE
                case 6: if( (uint32_t)rs1 < (uint32_t)rs2 ) state.pc = immm4; break;   //BLTU
                case 7: if( (uint32_t)rs1 >= (uint32_t)rs2 ) state.pc = immm4; break;  //BGEU
                default: trap = (2+1);
            }
            break;
        }
        case 0x03: // Load (0b0000011)
        {
            
            uint32_t rs1 = state.gpr[((ir >> 15) & 0x1f)].val;
            uint32_t imm = ir >> 20;
            int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
            uint32_t rsval = rs1 + imm_se;
#define support_mmio 1
#ifdef support_mmio
            // printf("load rsval:%08x, rs1:%08x, func:%d\n", rsval,rs1,( ir >> 12 ) & 0x7);
            if(first_time){
                printf("pc : %08x, data : [0x%08x]\n", state.pc, data.val);
                printf("load rsval:%08x, rs1:%08x, func:%d\n", rsval,rs1,( ir >> 12 ) & 0x7);
                first_time = false;
            }

            // rsval += VADDR_BASE;
            
            // printf("load rsval:%08x, rs1:%08x, func:%d\n", rsval,rs1,( ir >> 12 ) & 0x7);
            if( rsval >= MEM_RANGE-3 || rsval < VADDR_BASE)
            {
                
                // printf("rsval is out of MEM_SIZE : %08x", rsval);
                if( rsval >= 0x10000000 && rsval < 0x12000000 )  // UART, CLNT
                {
                    if( rsval == 0x1100bffc ) // https://chromitem-soc.readthedocs.io/en/latest/clint.html
                        pre_rd = CSR( timerh );
                    else if( rsval == 0x1100bff8 )
                        pre_rd = CSR( timerl );
                    else
                        // pre_rd = Handle_Load(rsval)
                        MINIRV32_HANDLE_MEM_LOAD_CONTROL( rsval, pre_rd );
                }
                else
                {
                    trap = (5+1);
                    pre_rd = rsval;
                }
            }
            else
            {
#endif
                switch( ( ir >> 12 ) & 0x7 )
                {
                    //LB, LH, LW, LBU, LHU
                    case 0: pre_rd = iv_mem.load(rsval, 1, true); break;
                    case 1: pre_rd = iv_mem.load(rsval, 2, true); break;
                    case 2: 
                        pre_rd = iv_mem.load(rsval, 4, false);
                        // if(rd_idx==0x10)
                            // printf("pre_rd:%08x\n", pre_rd);
                        break;
                    case 4: pre_rd = iv_mem.load(rsval, 1, false); break;
                    case 5: pre_rd = iv_mem.load(rsval, 2, false); break;
                    default: trap = (2+1);
                }
#ifdef support_mmio
            }
#endif
            break;
        }
        case 0x23: // Store 0b0100011
        {
            // printf("store\n");
            uint32_t rs1 = state.gpr[((ir >> 15) & 0x1f)].val;
            uint32_t rs2 = state.gpr[((ir >> 20) & 0x1f)].val;
            uint32_t addy = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
            if( addy & 0x800 ) addy |= 0xfffff000;
#ifdef support_mmio
            addy += rs1;
            rd_idx = 0;
            if(first_time){
                printf("pc : %08x, data : [0x%08x]\n", state.pc, data.val);
                printf("rs1_idx : %08x, rs2_idx : %08x\n", ((ir >> 15) & 0x1f), ((ir >> 20) & 0x1f));
                printf("store addy:%08x, rs1:%08x, rs2:%08x,func:%d\n", addy,rs1,rs2,( ir >> 12 ) & 0x7);
                dump_state(state);
                first_time = false;
            }
            // printf("addy:%08x, ir:%03x, rs2:%08x\n", addy, ( ir >> 12 ) & 0x7, rs2);
            if( addy >= MEM_RANGE-3 || addy < VADDR_BASE)
            {
                // printf("addy:%08x, ir:%03x, rs2:%08x\n", addy, ( ir >> 12 ) & 0x7, rs2);
                // printf("add op\n");
                // printf("addy:%08x, ir:%03x, rs2:%08x\n", addy+VADDR_BASE, ( ir >> 12 ) & 0x7, rs2);
                if( addy >= 0x10000000 && addy < 0x12000000 )
                {
                    // Should be stuff like SYSCON, 8250, CLNT
                    if( addy == 0x11004004 ) //CLNT
                        CSR( timermatchh ) = rs2;
                    else if( addy == 0x11004000 ) //CLNT
                        CSR( timermatchl ) = rs2;
                    else if( addy == 0x11100000 ) //SYSCON (reboot, poweroff, etc.)
                    {
                        state.pc += 4;
                        printf("here rs2: %d", rs2);
                        return rs2; // NOTE: PC will be PC of Syscon.
                    }
                    else
                        MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, rs2 );
                }
                else
                {
                    trap = (7+1); // Store access fault.
                    pre_rd = addy;
                }
            }
            else
            {
#endif
                switch( ( ir >> 12 ) & 0x7 )
                {
                    //SB, SH, SW
                    case 0: iv_mem.store( addy, rs2, 1); break;
                    case 1: iv_mem.store( addy, rs2, 2); break;
                    case 2: iv_mem.store( addy, rs2, 4); break;
                    default: trap = (2+1);
                }
#ifdef support_mmio
            }
#endif
            break;
        }
        case 0x13: // Op-immediate 0b0010011
        case 0x33: // Op           0b0110011
        {
            // printf("op\n");
            uint32_t imm = ir >> 20;
            imm = imm | (( imm & 0x800 )?0xfffff000:0);
            uint32_t rs1 = state.gpr[((ir >> 15) & 0x1f)].val;
            uint32_t is_reg = !!( ir & 0x20 );
            uint32_t rs2 = is_reg ? state.gpr[(imm & 0x1f)].val : imm;

            if( is_reg && ( ir & 0x02000000 ) )
            {
                switch( (ir>>12)&7 ) //0x02000000 = RV32M
                {
                    case 0: pre_rd = rs1 * rs2; break; // MUL
#ifndef CUSTOM_MULH // If compiling on a system that doesn't natively, or via libgcc support 64-bit math.
                    case 1: pre_rd = ((int64_t)((int32_t)rs1) * (int64_t)((int32_t)rs2)) >> 32; break; // MULH
                    case 2: pre_rd = ((int64_t)((int32_t)rs1) * (uint64_t)rs2) >> 32; break; // MULHSU
                    case 3: pre_rd = ((uint64_t)rs1 * (uint64_t)rs2) >> 32; break; // MULHU
#else
                    CUSTOM_MULH
#endif
                    case 4: if( rs2 == 0 ) pre_rd = -1; else pre_rd = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? rs1 : ((int32_t)rs1 / (int32_t)rs2); break; // DIV
                    case 5: if( rs2 == 0 ) pre_rd = 0xffffffff; else pre_rd = rs1 / rs2; break; // DIVU
                    case 6: if( rs2 == 0 ) pre_rd = rs1; else pre_rd = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? 0 : ((uint32_t)((int32_t)rs1 % (int32_t)rs2)); break; // REM
                    case 7: if( rs2 == 0 ) pre_rd = rs1; else pre_rd = rs1 % rs2; break; // REMU
                }
            }
            else
            {
                switch( (ir>>12)&7 ) // These could be either op-immediate or op commands.  Be careful.
                {
                    case 0: pre_rd = (is_reg && (ir & 0x40000000) ) ? ( rs1 - rs2 ) : ( rs1 + rs2 ); break; 
                    case 1: pre_rd = rs1 << (rs2 & 0x1F); break;
                    case 2: pre_rd = (int32_t)rs1 < (int32_t)rs2; break;
                    case 3: pre_rd = rs1 < rs2; break;
                    case 4: pre_rd = rs1 ^ rs2; break;
                    case 5: pre_rd = (ir & 0x40000000 ) ? ( ((int32_t)rs1) >> (rs2 & 0x1F) ) : ( rs1 >> (rs2 & 0x1F) ); break;
                    case 6: pre_rd = rs1 | rs2; break;
                    case 7: pre_rd = rs1 & rs2; break;
                }
            }
            break;
        }
        case 0x0f: // 0b0001111
        {
            // printf("0x0f\n");
            rd_idx = 0;   // fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
            break;
        }
#define support_super 1
#ifdef support_super
        case 0x73: // Zifencei+Zicsr  (0b1110011)
        {
            // printf("machine\n");
            uint32_t csrno = ir >> 20;//[31:20] is machine trap num
            uint32_t microop = ( ir >> 12 ) & 0x7;//which aotic operation
            if( (microop & 3) ) // It's a Zicsr function.
            {
                // printf("csr func, csrno:%03x\n",csrno);
                int rs1imm = (ir >> 15) & 0x1f;
                uint32_t rs1 = state.gpr[(rs1imm)].val;
                uint32_t writeval = rs1;

                // https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
                // Generally, support for Zicsr
                switch( csrno )
                {
                    case 0x340: pre_rd = CSR( mscratch ); break;
                    case 0x305: pre_rd = CSR( mtvec ); break;
                    case 0x304: pre_rd = CSR( mie ); break;
                    // case 0xC00: pre_rd = cycle; break;
                    case 0x344: pre_rd = CSR( mip ); break;
                    case 0x341: pre_rd = CSR( mepc ); break;
                    case 0x300: pre_rd = CSR( mstatus ); break; //mstatus
                    case 0x342: pre_rd = CSR( mcause ); break;
                    case 0x343: pre_rd = CSR( mtval ); break;
                    case 0xf11: pre_rd = 0xff0ff0ff; break; //mvendorid
                    case 0x301: pre_rd = 0x40401101; break; //misa (XLEN=32, IMA+X)
                    //case 0x3B0: pre_rd = 0; break; //pmpaddr0
                    //case 0x3a0: pre_rd = 0; break; //pmpcfg0
                    //case 0xf12: pre_rd = 0x00000000; break; //marchid
                    //case 0xf13: pre_rd = 0x00000000; break; //mimpid
                    //case 0xf14: pre_rd = 0x00000000; break; //mhartid
                    default:
                    {
                        // printf("other csr read\n");
                        MINIRV32_OTHERCSR_READ( csrno, pre_rd );
                        break;
                    }   
                }
                // printf("microop:%d\n", microop);
                switch( microop )
                {
                    
                    case 1: writeval = rs1; break;  			//CSRRW
                    case 2: writeval = pre_rd | rs1; break;		//CSRRS
                    case 3: writeval = pre_rd & ~rs1; break;		//CSRRC
                    case 5: writeval = rs1imm; break;			//CSRRWI
                    case 6: writeval = pre_rd | rs1imm; break;	//CSRRSI
                    case 7: writeval = pre_rd & ~rs1imm; break;	//CSRRCI
                }

                switch( csrno )
                {
                    // printf("write csr no:%d\n", csrno);
                    case 0x340: SETCSR( mscratch, writeval ); break;
                    case 0x305: SETCSR( mtvec, writeval ); break;
                    case 0x304: SETCSR( mie, writeval ); break;
                    case 0x344: SETCSR( mip, writeval ); break;
                    case 0x341: SETCSR( mepc, writeval ); break;
                    case 0x300: SETCSR( mstatus, writeval ); break; //mstatus
                    case 0x342: SETCSR( mcause, writeval ); break;
                    case 0x343: SETCSR( mtval, writeval ); break;
                    //case 0x3a0: break; //pmpcfg0
                    //case 0x3B0: break; //pmpaddr0
                    //case 0xf11: break; //mvendorid
                    //case 0xf12: break; //marchid
                    //case 0xf13: break; //mimpid
                    //case 0xf14: break; //mhartid
                    //case 0x301: break; //misa
                    default:
                    {
                        // printf("write csr\n");
                        MINIRV32_OTHERCSR_WRITE( csrno, writeval );
                        break;
                    }

                }
            }
            else if( microop == 0x0 ) // "SYSTEM" 0b000 ECALL/EBREAK
            {
                printf("SYSTEM\n");
                rd_idx = 0;
                if( csrno == 0x105 ) //WFI (Wait for interrupts)
                {
                    CSR( mstatus ) |= 8;    //Enable interrupts
                    CSR( extraflags ) |= 4; //Infor environment we want to go to sleep.
                    // SETCSR( pc, (pc + 4) );
                    state.pc += 4;
                    return 1;
                }
                else if( ( ( csrno & 0xff ) == 0x02 ) )  // MRET
                {
                    //https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
                    //Table 7.6. MRET then in mstatus/mstatush sets MPV=0, MPP=0, MIE=MPIE, and MPIE=1. La
                    // Should also update mstatus to reflect correct mode.
                    uint32_t startmstatus = CSR( mstatus );
                    uint32_t startextraflags = CSR( extraflags );
                    SETCSR( mstatus , (( startmstatus & 0x80) >> 4) | ((startextraflags&3) << 11) | 0x80 );
                    SETCSR( extraflags, (startextraflags & ~3) | ((startmstatus >> 11) & 3) );
                    state.pc = CSR( mepc ) -4;
                }
                else
                {
                    switch( csrno )
                    {
                    case 0: trap = ( CSR( extraflags ) & 3) ? (11+1) : (8+1); break; // ECALL; 8 = "Environment call from U-mode"; 11 = "Environment call from M-mode"
                    case 1:	trap = (3+1); break; // EBREAK 3 = "Breakpoint"
                    default: trap = (2+1); break; // Illegal opcode.
                    }
                }
            }
            else
                trap = (2+1); 				// Note micrrop 0b100 == undefined.
            break;
        }
#endif
#define support_rv32a 1
#ifdef support_rv32a
        case 0x2f: // RV32A (0b00101111)
        {
            // printf("rv32a\n");
            uint32_t rs1 = state.gpr[((ir >> 15) & 0x1f)].val;
            uint32_t rs2 = state.gpr[((ir >> 20) & 0x1f)].val;
            uint32_t irmid = ( ir>>27 ) & 0x1f;


            // We don't implement load/store from UART or CLNT with RV32A here.

            if( rs1 >= MEM_RANGE-3 )
            {
                trap = (7+1); //Store/AMO access fault
                pre_rd = rs1;
            }
            else
            {
                pre_rd = iv_mem.load( rs1 ,4, false);

                // Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
                uint32_t dowrite = 1;
                switch( irmid )
                {
                    case 2: //LR.W (0b00010)
                        dowrite = 0;
                        CSR( extraflags ) = (CSR( extraflags ) & 0x07) | (rs1<<3);
                        break;
                    case 3:  //SC.W (0b00011) (Make sure we have a slot, and, it's valid)
                        pre_rd = ( CSR( extraflags ) >> 3 != ( rs1 & 0x1fffffff ) );  // Validate that our reservation slot is OK.
                        dowrite = !pre_rd; // Only write if slot is valid.
                        break;
                    case 1: break; //AMOSWAP.W (0b00001)
                    case 0: rs2 += pre_rd; break; //AMOADD.W (0b00000)
                    case 4: rs2 ^= pre_rd; break; //AMOXOR.W (0b00100)
                    case 12: rs2 &= pre_rd; break; //AMOAND.W (0b01100)
                    case 8: rs2 |= pre_rd; break; //AMOOR.W (0b01000)
                    case 16: rs2 = ((int32_t)rs2<(int32_t)pre_rd)?rs2:pre_rd; break; //AMOMIN.W (0b10000)
                    case 20: rs2 = ((int32_t)rs2>(int32_t)pre_rd)?rs2:pre_rd; break; //AMOMAX.W (0b10100)
                    case 24: rs2 = (rs2<pre_rd)?rs2:pre_rd; break; //AMOMINU.W (0b11000)
                    case 28: rs2 = (rs2>pre_rd)?rs2:pre_rd; break; //AMOMAXU.W (0b11100)
                    default: trap = (2+1); dowrite = 0; break; //Not supported.
                }
                if( dowrite ) iv_mem.store( rs1, rs2 ,4);
            }
            break;
        }
#endif
        default: trap = (2+1); // Fault: Invalid opcode.
    }//switch
    
    }//else
    // If there was a trap, do NOT allow register writeback.
    if( trap ){
        // printf("trap\n");
        if(trap & 0x80000000){// a interpert
            state.mcause = trap;
            state.mtval = 0;
            state.pc += 4;
        }
        else{
            SETCSR( mcause,  trap - 1 );
			SETCSR( mtval, (trap > 5 && trap <= 8)? pre_rd : state.pc );
        }
        state.mepc = state.pc;
        // SETCSR( mepc, pc ); //TRICKY: The kernel advances mepc automatically.
		//CSR( mstatus ) & 8 = MIE, & 0x80 = MPIE
		// On an interrupt, the system moves current MIE into MPIE
		SETCSR( mstatus, (( CSR( mstatus ) & 0x08) << 4) | (( CSR( extraflags ) & 3 ) << 11) );
		state.pc = (CSR( mtvec ) - 4);

		// If trapping, always enter machine mode.
		CSR( extraflags ) |= 3;

		trap = 0;
		state.pc += 4;
        return 0;
    }

    if( rd_idx )//zero reg should not be written any value
    {
        state.gpr[rd_idx].val = pre_rd;
    }
    return 0;
}

void cpu_t::dump_state(riscv32_cpu_state& state){
    printf( "Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x ",
    state.gpr[0].val, state.gpr[1].val, state.gpr[2].val, state.gpr[3].val, state.gpr[4].val, state.gpr[5].val, state.gpr[6].val, state.gpr[7].val,
    state.gpr[8].val, state.gpr[9].val, state.gpr[10].val, state.gpr[11].val, state.gpr[12].val, state.gpr[13].val, state.gpr[14].val, state.gpr[15].val );
	printf( "a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\n",
    state.gpr[16].val, state.gpr[17].val, state.gpr[18].val, state.gpr[19].val, state.gpr[20].val, state.gpr[21].val, state.gpr[22].val, state.gpr[23].val,
    state.gpr[24].val, state.gpr[25].val, state.gpr[26].val, state.gpr[27].val, state.gpr[28].val, state.gpr[29].val, state.gpr[30].val, state.gpr[31].val );
}

uint32_t cpu_t::handle_exception(riscv32_cpu_state& state, uint32_t ir){
    //todo
    return 3;
}

void cpu_t::register_func(decoder decode){
    all_inst.push_back(decode);
}

void cpu_t:: init(riscv32_cpu_state& cpu){
    //zero reg x0, and set pc
    
    cpu.gpr[0].val = 0;
    cpu.pc = RESET_VECTOR;
#if 0
#define DECLARE_INSN(name, match, mask) \
    uint32_t name##_match = (match); \
    uint32_t name##_mask = (mask); \
    extern uint32_t func_##name(riscv32_cpu_state&, instr); \

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
#endif
}
#define MAX_CYCLE 1024
pc_t cpu_t::fetch_decode_exec(mmu_t& iv_mem, riscv32_cpu_state& cpu){
    //run once
    uint32_t max_inst = -1;
    cpu.extraflags |= 3;//machine module
    // printf(">%s \n", __FUNCTION__);
    cpu.gpr[11].val = MEM_RANGE - sizeof(default64mbdtb);
    printf("gpr11:%08x, size:%08x\n", cpu.gpr[11].val, sizeof(default64mbdtb));

    uint32_t dtb = MEM_RANGE - sizeof(default64mbdtb);
    printf("default %08x\n", default64mbdtb[0x13c/4]);
    printf("before write:%08x, dtb:%08x\n", iv_mem.read(dtb+0x13c, 4), dtb);
    if(iv_mem.read(dtb+0x13c, 4) == 0x00c0ff03){
        uint32_t validram = cpu.gpr[11].val;
        uint32_t update = (validram>>24) | ((( validram >> 16 ) & 0xff) << 8 ) | (((validram>>8) & 0xff ) << 16 ) | ( ( validram & 0xff) << 24 );
        iv_mem.write(dtb + 0x13c, update);
        printf("wirte:%08x\n", update);
        printf("read:%08x\n", iv_mem.read(dtb + 0x13c,4));
    }


    uint32_t last_time = GetTimeMicroseconds();
    for(int cycle = 0; cycle < max_inst||max_inst < 0; cycle++){
        instr isn;
        uint32_t time_sed = GetTimeMicroseconds()-last_time;
        last_time += time_sed;
        isn.val = iv_mem.fetch(cpu.pc);
        uint32_t ret = decode_run(cpu, isn, iv_mem, time_sed);
        cpu.pc += 4;
        // printf("ret is :%08x\n", ret);
        switch(ret){
            case 0: break;
            case 1:{
                printf("1 is got !, should sleep!\n");
                // sleep(1);
                MiniSleep();
                break;
                //todo: should halt here
            }
            case 2: 
                printf("return 2 fault\n");
                max_inst = 0;//stop when fault
                break;
            case 3:   
                max_inst = 0;
                break;
            default:
                printf("unknown fault!\n");
                break;
        }
    }

    return cpu.pc;
}