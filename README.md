# **RISC-V 32-bit Single-Cycle CPU**  

This repository contains the **SystemVerilog implementation** of a **RISC-V 32-bit Single-Cycle CPU**, supporting a subset of the **RV32I instruction set**. The processor is designed to execute fundamental arithmetic, logical, memory, and branch instructions.  

## **Supported Instructions**  
The CPU can execute the following instructions from the **RV32I** instruction set:  

✅ **Arithmetic & Logical**: `add`, `sub`, `and`, `or`, `xor`, `slt`, `sltu`  
✅ **Immediate Operations**: `addi`, `andi`, `ori`, `xori`, `slti`, `sltiu`  
✅ **Branching**: `beq`, `bne`  
✅ **Memory Operations**: `lw`, `sw`  
✅ **Control Flow**: `jal`  

## **How to Run**  
1️⃣ Write Assembly Code

Write your RISC-V assembly program and save it in:

`RISCV.sim/sim_1/behav/xsim/instruction.hex`

2️⃣ Simulate the CPU

Run the simulation in Vivado / ModelSim

The results will be stored in:

`RISCV.sim/sim_1/behav/xsim/Register.txt`
`RISCV.sim/sim_1/behav/xsim/Register_file.hex`
`RISCV.sim/sim_1/behav/xsim/DataMem_file.hex`

## **Future Improvements**
🚀 Add pipeline stages (IF, ID, EX, MEM, WB) for performance optimization.
🚀 Implement forwarding & hazard detection.
🚀 Extend support to more RISC-V instructions (RV32M, RV32F).

## **License**
This project is open-source. Feel free to modify and contribute
