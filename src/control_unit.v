/*
    Control Unit for a simple CPU architecture
    This module supports pipelining
*/
module pipelined_control_unit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    input stall,
    input flush,
    output reg [1:0] alu_op,
    output reg alu_src,
    output reg reg_write,
    output reg mem_to_reg,
    output reg mem_write,
    output reg branch,
    output reg jump,
    output reg pc_write
);
    localparam R_TYPE = 7'b0110011;
    localparam I_TYPE_ALU = 7'b0010011;
    localparam I_TYPE_LD = 7'b0000011;
    localparam S_TYPE = 7'b0100011;
    localparam B_TYPE = 7'b1100011;
    localparam J_TYPE_JAL = 7'b1101111;
    localparam J_TYPE_JALR = 7'b1100111;

    reg [1:0] alu_op_temp;
    reg alu_src_temp;
    reg mem_to_reg_temp;
    reg reg_write_temp;
    reg mem_read_temp;
    reg mem_write_temp;
    reg branch_temp;
    reg jump_temp;

    always @(*) begin
        // Default values
        alu_op_temp = 2'b00;
        alu_src_temp = 1'b0;
        mem_to_reg_temp = 1'b0;
        reg_write_temp = 1'b0;
        mem_read_temp = 1'b0;
        mem_write_temp = 1'b0;
        branch_temp = 1'b0;
        jump_temp = 1'b0;

        case(opcode)
            R_TYPE: begin
                alu_op_temp = 2'b10;     // R-type ALU operation
                alu_src_temp = 1'b0;     // Use register value for ALU
                mem_to_reg_temp = 1'b0;  // Write ALU result to register
                reg_write_temp = 1'b1;   // Enable register write
                mem_read_temp = 1'b0;    // No memory read
                mem_write_temp = 1'b0;   // No memory write
                branch_temp = 1'b0;      // Not a branch
                jump_temp = 1'b0;        // Not a jump
            end

            I_TYPE_ALU: begin
                alu_op_temp = 2'b11;     // I-type ALU operation
                alu_src_temp = 1'b1;     // Use immediate value for ALU
                mem_to_reg_temp = 1'b0;  // Write ALU result to register
                reg_write_temp = 1'b1;   // Enable register write
                mem_read_temp = 1'b0;    // No memory read
                mem_write_temp = 1'b0;   // No memory write
                branch_temp = 1'b0;      // Not a branch
                jump_temp = 1'b0;        // Not a jump
            end

            I_TYPE_LD: begin
                alu_op_temp = 2'b00;     // Addition for address calculation
                alu_src_temp = 1'b1;     // Use immediate for address calculation
                mem_to_reg_temp = 1'b1;  // Write memory data to register
                reg_write_temp = 1'b1;   // Enable register write
                mem_read_temp = 1'b1;    // Enable memory read
                mem_write_temp = 1'b0;   // No memory write
                branch_temp = 1'b0;      // Not a branch
                jump_temp = 1'b0;        // Not a jump
            end

            S_TYPE: begin
                alu_op_temp = 2'b00;     // Addition for address calculation
                alu_src_temp = 1'b1;     // Use immediate for address calculation
                mem_to_reg_temp = 1'bx;  // Don't care (no register write)
                reg_write_temp = 1'b0;   // No register write
                mem_read_temp = 1'b0;    // No memory read
                mem_write_temp = 1'b1;   // Enable memory write
                branch_temp = 1'b0;      // Not a branch
                jump_temp = 1'b0;        // Not a jump
            end

            B_TYPE: begin
                alu_op_temp = 2'b01;     // Subtraction for comparison
                alu_src_temp = 1'b0;     // Use register value for comparison
                mem_to_reg_temp = 1'bx;  // Don't care (no register write)
                reg_write_temp = 1'b0;   // No register write
                mem_read_temp = 1'b0;    // No memory read
                mem_write_temp = 1'b0;   // No memory write
                branch_temp = 1'b1;      // Is a branch
                jump_temp = 1'b0;        // Not a jump
            end

            J_TYPE_JAL: begin
                alu_op_temp = 2'bxx;     // ALU not used for operation
                alu_src_temp = 1'bx;     // Don't care
                mem_to_reg_temp = 1'b0;  // Write PC+4 to register
                reg_write_temp = 1'b1;   // Enable register write
                mem_read_temp = 1'b0;    // No memory read
                mem_write_temp = 1'b0;   // No memory write
                branch_temp = 1'b0;      // Not a branch
                jump_temp = 1'b1;        // Is a jump
            end

            I_TYPE_JALR: begin
                alu_op_temp = 2'b00;     // Addition for address calculation
                alu_src_temp = 1'b1;     // Use immediate
                mem_to_reg_temp = 1'b0;  // Write PC+4 to register
                reg_write_temp = 1'b1;   // Enable register write
                mem_read_temp = 1'b0;    // No memory read
                mem_write_temp = 1'b0;   // No memory write
                branch_temp = 1'b0;      // Not a branch
                jump_temp = 1'b1;        // Is a jump
            end

             default: begin
                alu_op_temp = 2'b00;
                alu_src_temp = 1'b0;
                mem_to_reg_temp = 1'b0;
                reg_write_temp = 1'b0;
                mem_read_temp = 1'b0;
                mem_write_temp = 1'b0;
                branch_temp = 1'b0;
                jump_temp = 1'b0;
            end
        endcase
    end
     always @(*) begin
        // Flush: reset control signals to default values
        if (flush) begin
            alu_op = 2'b00;
            alu_src = 1'b0;
            mem_to_reg = 1'b0;
            reg_write = 1'b0;
            mem_read = 1'b0;
            mem_write = 1'b0;
            branch = 1'b0;
            jump = 1'b0;
            pc_write = 1'b1;
        end
        // Stall holds control signals
        else if (stall) begin
            alu_op = alu_op_temp;
            alu_src = alu_src_temp;
            mem_to_reg = mem_to_reg_temp;
            reg_write = 1'b0;       // Disable register write
            mem_read = 1'b0;        // Disable memory read
            mem_write = 1'b0;       // Disable memory write
            branch = branch_temp;
            jump = jump_temp;
            pc_write = 1'b0;        // Disable PC update to stall fetch
        end
        // Continue with normal operation
        else begin
            alu_op = alu_op_temp;
            alu_src = alu_src_temp;
            mem_to_reg = mem_to_reg_temp;
            reg_write = reg_write_temp;
            mem_read = mem_read_temp;
            mem_write = mem_write_temp;
            branch = branch_temp;
            jump = jump_temp;
            pc_write = 1'b1;        // Enable PC update
        end
    end
endmodule