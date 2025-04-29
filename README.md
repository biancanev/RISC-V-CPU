# Custom RISC-V Verilog CPU

### 1.0 Introduction

This project is meant to build off of the CS61C CPU project at UC Berkeley. Since this is a project for fun, it does not follow strictly to the spec of the project, but most of the features of this Verilog CPU are related.

### 2.0 ALU Spec

| Input | Bit Length | Description |
| --- | --- | --- |
| a | 32 | Used as input `A` for arithmetic operations |
| b | 32 | Used as input `B` for arithmetic operations |
| alu_control | 4 | Controls the ALU operation |

| Output | Bit Length | Description |
| --- | --- | --- |
| result | 32 | Output of arithmetic operation |
| zero | 1 | Zero flag

| alu_control | Instruction |
| --- | --- |
| 0 | add: `RESULT = A + B` |
| 1 | sub: `RESULT = A - B` |
| 2 | and: `RESULT = A & B` |
| 3 | or: `RESULT = A \| B` |
| 4 | xor: `RESULT = A ^ B` |
| 5 | sll: `RESULT = A << B[4:0]` |
| 6 | srl: `RESULT = A >> B[4:0]` |
| 7 | sra: `RESULT = signed(A) >>> B[4:0]`|
| 8 | slt: `RESULT = (signed(A) < signed(B)) ? 1 : 0` |
| 8 | sltu: `RESULT = (A < B) ? 1 : 0` |

Any undefined signal to the ALU returns binary 0.