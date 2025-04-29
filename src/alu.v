/*
    ALU Module for a simple RISC-V processor
    Given inputs 32-bit inputs a and b and a 4-bit ALU control signal, output a 32 bit result
*/

module alu(
    input [31:0] a,
    input [31:0] b,
    input [3:0] alu_control,
    output reg [31:0] result,
    output wire zero
);

    // ALU Control Signals
    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001;
    localparam ALU_AND = 4'b0010;
    localparam ALU_OR  = 4'b0011;
    localparam ALU_XOR = 4'b0100;
    localparam ALU_SLL = 4'b0101;
    localparam ALU_SRL = 4'b0110;
    localparam ALU_SRA = 4'b0111;
    localparam ALU_SLT = 4'b1000;
    localparam ALU_SLTU = 4'b1001;

    // Set zero flag if result is zero
    assign zero = (result == 32'b0);

    always @(*) begin
        case(alu_control)
            ALU_ADD: result = a + b; // Addition
            ALU_SUB: result = a - b; // Subtraction
            ALU_AND: result = a & b; // Bitwise AND
            ALU_OR:  result = a | b; // Bitwise OR
            ALU_XOR: result = a ^ b; // Bitwise XOR
            ALU_SLL: result = a << b[4:0]; // Logical Shift Left
            ALU_SRL: result = a >> b[4:0]; // Logical Shift Right
            ALU_SRA: result = $signed(a) >>> b[4:0]; // Arithmetic Shift Right
            ALU_SLT: result = ($signed(a) < $signed(b)) ? 32'b1 : 32'b0; // Set Less Than (signed)
            ALU_SLTU: result = (a < b) ? 32'b1 : 32'b0; // Set Less Than (unsigned)
            default: result = 32'b0; // Default case
        endcase
end


endmodule