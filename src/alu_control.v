module alu_control(
    input [1:0] alu_op,          // ALU operation type from main control unit
    input [2:0] funct3,          // Instruction funct3 field
    input [6:0] funct7,          // Instruction funct7 field
    output reg [3:0] alu_control // ALU control signals
);

    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001;
    localparam ALU_AND = 4'b0010;
    localparam ALU_OR  = 4'b0011;
    localparam ALU_XOR = 4'b0100;
    localparam ALU_SLL = 4'b0101; // Shift Left Logical
    localparam ALU_SRL = 4'b0110; // Shift Right Logical
    localparam ALU_SRA = 4'b0111; // Shift Right Arithmetic
    localparam ALU_SLT = 4'b1000; // Set Less Than (signed)
    localparam ALU_SLTU = 4'b1001; // Set Less Than (unsigned)
    localparam ALU_OP_LOAD_STORE = 2'b00; // Load/Store instructions
    localparam ALU_OP_BRANCH = 2'b01;     // Branch instructions
    localparam ALU_OP_R_TYPE = 2'b10;     // R-type instructions
    localparam ALU_OP_I_TYPE = 2'b11;     // I-type ALU instructions

     always @(*) begin
        case(alu_op)
            ALU_OP_LOAD_STORE: begin
                // Load/Store instructions use addition
                alu_control = ALU_ADD;
            end
            
            ALU_OP_BRANCH: begin
                // Branch instructions use subtraction for comparison
                alu_control = ALU_SUB;
            end
            
            ALU_OP_R_TYPE: begin
                // R-type instructions: decode based on funct3 and funct7
                case(funct3)
                    3'b000: begin
                        // ADD or SUB based on funct7 bit 5
                        if (funct7[5]) 
                            alu_control = ALU_SUB;
                        else
                            alu_control = ALU_ADD;
                    end
                    3'b001: alu_control = ALU_SLL;  // SLL
                    3'b010: alu_control = ALU_SLT;  // SLT
                    3'b011: alu_control = ALU_SLTU; // SLTU
                    3'b100: alu_control = ALU_XOR;  // XOR
                    3'b101: begin
                        // SRL or SRA based on funct7 bit 5
                        if (funct7[5]) 
                            alu_control = ALU_SRA;
                        else
                            alu_control = ALU_SRL;
                    end
                    3'b110: alu_control = ALU_OR;   // OR
                    3'b111: alu_control = ALU_AND;  // AND
                    default: alu_control = ALU_ADD; // Default to ADD
                endcase
            end
            
            ALU_OP_I_TYPE: begin
                // I-type ALU instructions: decode based on funct3
                case(funct3)
                    3'b000: alu_control = ALU_ADD;  // ADDI
                    3'b001: alu_control = ALU_SLL;  // SLLI
                    3'b010: alu_control = ALU_SLT;  // SLTI
                    3'b011: alu_control = ALU_SLTU; // SLTIU
                    3'b100: alu_control = ALU_XOR;  // XORI
                    3'b101: begin
                        // SRLI or SRAI based on funct7 bit 5
                        if (funct7[5]) 
                            alu_control = ALU_SRA;
                        else
                            alu_control = ALU_SRL;
                    end
                    3'b110: alu_control = ALU_OR;   // ORI
                    3'b111: alu_control = ALU_AND;  // ANDI
                    default: alu_control = ALU_ADD; // Default to ADD
                endcase
            end
            
            default: alu_control = ALU_ADD; // Default to ADD for unrecognized alu_op
        endcase
    end

endmodule