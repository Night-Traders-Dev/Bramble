#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <stdint.h>

/* Instruction handler prototypes */
void instr_movs_imm8(uint16_t instr);
void instr_ldr_pc_imm8(uint16_t instr);
void instr_str_reg_offset(uint16_t instr);
void instr_ldr_reg_offset(uint16_t instr);
void instr_add_reg_reg(uint16_t instr);
void instr_sub_reg_reg(uint16_t instr);
void instr_push(uint16_t instr);
void instr_pop(uint16_t instr);
void instr_bkpt(uint16_t instr);
void instr_cmp_reg_reg(uint16_t instr);
void instr_stmia(uint16_t instr);
void instr_ldmia(uint16_t instr);
void instr_bcond(uint16_t instr);
void instr_bl(uint16_t instr);
void instr_bx(uint16_t instr);
void instr_unimplemented(uint16_t instr);
void instr_nop(uint16_t instr);
void instr_udf(uint16_t instr);
void instr_shift_logical_left(uint16_t instr);
void instr_shift_logical_right(uint16_t instr);
void instr_shift_arithmetic_right(uint16_t instr);
void instr_bitwise_and(uint16_t instr);
void instr_bitwise_eor(uint16_t instr);
void instr_bitwise_orr(uint16_t instr);
void instr_bitwise_bic(uint16_t instr);
void instr_bitwise_mvn(uint16_t instr);

#endif
