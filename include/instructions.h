#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <stdint.h>

// ============================================================================
// FOUNDATIONAL INSTRUCTIONS (Bootloader Essential)
// ============================================================================

// Data Movement
void instr_movs_imm8(uint16_t instr);              // MOVS Rd, #imm8 [EXISTING]
void instr_mov_reg(uint16_t instr);                // MOV Rd, Rm (high register support)

// Arithmetic Operations
void instr_adds_imm3(uint16_t instr);              // ADDS Rd, Rn, #imm3 [EXISTING]
void instr_adds_imm8(uint16_t instr);              // ADDS Rd, #imm8 [EXISTING]
void instr_add_reg_reg(uint16_t instr);            // ADDS Rd, Rn, Rm [EXISTING]
void instr_subs_imm3(uint16_t instr);              // SUBS Rd, Rn, #imm3 [EXISTING]
void instr_subs_imm8(uint16_t instr);              // SUBS Rd, #imm8
void instr_sub_reg_reg(uint16_t instr);            // SUBS Rd, Rn, Rm [EXISTING]

// Comparison
void instr_cmp_imm8(uint16_t instr);               // CMP Rn, #imm8
void instr_cmp_reg_reg(uint16_t instr);            // CMP Rn, Rm [EXISTING]

// Load/Store - Word (32-bit)
void instr_ldr_imm5(uint16_t instr);               // LDR Rd, [Rn, #imm5]
void instr_ldr_reg_offset(uint16_t instr);         // LDR Rd, [Rn, Rm] [EXISTING]
void instr_ldr_pc_imm8(uint16_t instr);            // LDR Rd, [PC, #imm8] [EXISTING]
void instr_ldr_sp_imm8(uint16_t instr);            // LDR Rd, [SP, #imm8]
void instr_str_imm5(uint16_t instr);               // STR Rd, [Rn, #imm5]
void instr_str_reg_offset(uint16_t instr);         // STR Rd, [Rn, Rm] [EXISTING]
void instr_str_sp_imm8(uint16_t instr);            // STR Rd, [SP, #imm8]

// Stack Operations
void instr_push(uint16_t instr);                   // PUSH {reglist, lr} [EXISTING]
void instr_pop(uint16_t instr);                    // POP {reglist, pc} [EXISTING]

// Branches - Unconditional
void instr_b_uncond(uint16_t instr);               // B label [EXISTING]

// Branches - Conditional (all handled by bcond)
void instr_bcond(uint16_t instr);                  // Bxx label [EXISTING]
// Note: bcond handles BEQ, BNE, BCS, BCC, BMI, BPL, BVS, BVC, BHI, BLS, BGE, BLT, BGT, BLE

// Branch and Link / Exchange
void instr_bl(uint16_t instr);                     // BL label (32-bit) [EXISTING]
void instr_bx(uint16_t instr);                     // BX Rm [EXISTING]
void instr_blx(uint16_t instr);                    // BLX Rm

// ============================================================================
// ESSENTIAL INSTRUCTIONS (Program Execution)
// ============================================================================

// Load/Store - Byte (8-bit)
void instr_ldrb_imm5(uint16_t instr);              // LDRB Rd, [Rn, #imm5]
void instr_ldrb_reg_offset(uint16_t instr);        // LDRB Rd, [Rn, Rm] [EXISTING]
void instr_ldrsb_reg_offset(uint16_t instr);       // LDRSB Rd, [Rn, Rm] (sign-extended)
void instr_strb_imm5(uint16_t instr);              // STRB Rd, [Rn, #imm5]
void instr_strb_reg_offset(uint16_t instr);        // STRB Rd, [Rn, Rm]

// Load/Store - Halfword (16-bit)
void instr_ldrh_imm5(uint16_t instr);              // LDRH Rd, [Rn, #imm5]
void instr_ldrh_reg_offset(uint16_t instr);        // LDRH Rd, [Rn, Rm]
void instr_ldrsh_reg_offset(uint16_t instr);       // LDRSH Rd, [Rn, Rm] (sign-extended)
void instr_strh_imm5(uint16_t instr);              // STRH Rd, [Rn, #imm5]
void instr_strh_reg_offset(uint16_t instr);        // STRH Rd, [Rn, Rm]

// Shift Operations - Immediate
void instr_shift_logical_left(uint16_t instr);     // LSLS Rd, Rm, #imm5 [EXISTING]
void instr_shift_logical_right(uint16_t instr);    // LSRS Rd, Rm, #imm5 [EXISTING]
void instr_shift_arithmetic_right(uint16_t instr); // ASRS Rd, Rm, #imm5 [EXISTING]

// Shift Operations - Register
void instr_lsls_reg(uint16_t instr);               // LSLS Rd, Rs
void instr_lsrs_reg(uint16_t instr);               // LSRS Rd, Rs
void instr_asrs_reg(uint16_t instr);               // ASRS Rd, Rs
void instr_rors_reg(uint16_t instr);               // RORS Rd, Rs (rotate right)

// Logical Operations
void instr_bitwise_and(uint16_t instr);            // ANDS Rd, Rm [EXISTING]
void instr_bitwise_orr(uint16_t instr);            // ORRS Rd, Rm [EXISTING]
void instr_bitwise_eor(uint16_t instr);            // EORS Rd, Rm [EXISTING]
void instr_bitwise_bic(uint16_t instr);            // BICS Rd, Rm [EXISTING]
void instr_bitwise_mvn(uint16_t instr);            // MVNS Rd, Rm [EXISTING]

// Multiplication
void instr_muls(uint16_t instr);                   // MULS Rd, Rm

// Multiple Load/Store
void instr_stmia(uint16_t instr);                  // STMIA Rn!, {reglist} [EXISTING]
void instr_ldmia(uint16_t instr);                  // LDMIA Rn!, {reglist} [EXISTING]

// ============================================================================
// IMPORTANT INSTRUCTIONS (Advanced Features)
// ============================================================================

// Special Comparison
void instr_cmn_reg(uint16_t instr);                // CMN Rn, Rm (compare negative)
void instr_tst_reg_reg(uint16_t instr);            // TST Rn, Rm [EXISTING]
void instr_teq_reg(uint16_t instr);                // TEQ Rn, Rm (test equivalence)

// Special Register Access (32-bit instructions)
void instr_msr(uint32_t instr);                    // MSR spec_reg, Rn
void instr_mrs(uint32_t instr);                    // MRS Rd, spec_reg

// System Operations
void instr_svc(uint16_t instr);                    // SVC #imm8 (supervisor call)
void instr_bkpt(uint16_t instr);                   // BKPT #imm8 [EXISTING]

// High Register Operations
void instr_add_high_reg(uint16_t instr);           // ADD Rd, Rm (high registers)
void instr_cmp_high_reg(uint16_t instr);           // CMP Rn, Rm (high registers)
void instr_mov_high_reg(uint16_t instr);           // MOV Rd, Rm (high registers)

// ============================================================================
// OPTIONAL INSTRUCTIONS (Optimization & Polish)
// ============================================================================

// Conditional Execution
void instr_it(uint16_t instr);                     // IT{x{y{z}}} cond

// Memory Barriers (32-bit instructions)
void instr_dsb(uint32_t instr);                    // DSB (data sync barrier)
void instr_dmb(uint32_t instr);                    // DMB (data memory barrier)
void instr_isb(uint32_t instr);                    // ISB (instruction sync barrier)

// Power Management
void instr_wfi(uint16_t instr);                    // WFI (wait for interrupt)
void instr_wfe(uint16_t instr);                    // WFE (wait for event)
void instr_sev(uint16_t instr);                    // SEV (send event)

// Miscellaneous
void instr_nop(uint16_t instr);                    // NOP [EXISTING]
void instr_yield(uint16_t instr);                  // YIELD (hint instruction)
void instr_sxtb(uint16_t instr);                   // SXTB Rd, Rm (sign extend byte)
void instr_sxth(uint16_t instr);                   // SXTH Rd, Rm (sign extend halfword)
void instr_uxtb(uint16_t instr);                   // UXTB Rd, Rm (zero extend byte)
void instr_uxth(uint16_t instr);                   // UXTH Rd, Rm (zero extend halfword)
void instr_rev(uint16_t instr);                    // REV Rd, Rm (reverse bytes)
void instr_rev16(uint16_t instr);                  // REV16 Rd, Rm (reverse bytes in halfwords)
void instr_revsh(uint16_t instr);                  // REVSH Rd, Rm (reverse signed halfword)

// Address Calculation
void instr_add_sp_imm7(uint16_t instr);            // ADD SP, #imm7
void instr_sub_sp_imm7(uint16_t instr);            // SUB SP, #imm7
void instr_adr(uint16_t instr);                    // ADR Rd, label (PC-relative address)
void instr_add_pc_imm8(uint16_t instr);            // ADD Rd, PC, #imm8

// Special/Debug
void instr_unimplemented(uint16_t instr);          // Handler for unimplemented [EXISTING]
void instr_udf(uint16_t instr);                    // UDF (undefined) [EXISTING]
void instr_cpsid(uint16_t instr);                  // CPSID (disable interrupts)
void instr_cpsie(uint16_t instr);                  // CPSIE (enable interrupts)

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Flag manipulation
void set_flag_n(uint32_t result);
void set_flag_z(uint32_t result);
void set_flag_c(int carry);
void set_flag_v(int overflow);
void update_nz_flags(uint32_t result);
void update_nzc_flags(uint32_t result, int carry);
void update_nzcv_flags(uint32_t result, int carry, int overflow);
void update_add_flags(uint32_t op1, uint32_t op2, uint32_t result);
void update_sub_flags(uint32_t op1, uint32_t op2, uint32_t result);

// Condition code checking
int check_condition(uint8_t cond);

// Register access
uint32_t get_register(uint8_t reg);
void set_register(uint8_t reg, uint32_t value);

// Memory access with alignment checking
uint32_t read_memory_word(uint32_t address);
uint16_t read_memory_halfword(uint32_t address);
uint8_t read_memory_byte(uint32_t address);
void write_memory_word(uint32_t address, uint32_t value);
void write_memory_halfword(uint32_t address, uint16_t value);
void write_memory_byte(uint32_t address, uint8_t value);

// Sign extension helpers
uint32_t sign_extend_8(uint8_t value);
uint32_t sign_extend_16(uint16_t value);
uint32_t sign_extend_24(uint32_t value);

// Bit manipulation helpers
uint32_t rotate_right(uint32_t value, uint8_t amount);
uint32_t arithmetic_shift_right(uint32_t value, uint8_t amount);

#endif // INSTRUCTIONS_H
