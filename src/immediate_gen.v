module immediate_gen(
    input [31:0] instruction,        // Full instruction
    output reg [31:0] immediate      // Generated immediate value
);

    wire [6:0] opcode;
    assign opcode = instruction[6:0];

    always @(*) begin
        case(opcode)
            7'b0000011: begin // I-type (load)
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            end
            
            7'b0010011: begin // I-type (ALU)
                // Special case for shifts with immediate
                if (instruction[14:12] == 3'b001 || instruction[14:12] == 3'b101) begin
                    immediate = {27'b0, instruction[24:20]}; // shamt field
                end else begin
                    immediate = {{20{instruction[31]}}, instruction[31:20]};
                end
            end
            
            7'b0100011: begin // S-type (store)
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            end
            
            7'b1100011: begin // B-type (branch)
                immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], 
                             instruction[11:8], 1'b0};
            end
            
            7'b0110111: begin // U-type (lui)
                immediate = {instruction[31:12], 12'b0};
            end
            
            7'b0010111: begin // U-type (auipc)
                immediate = {instruction[31:12], 12'b0};
            end
            
            7'b1101111: begin // J-type (jal)
                immediate = {{12{instruction[31]}}, instruction[19:12], instruction[20], 
                             instruction[30:21], 1'b0};
            end
            
            7'b1100111: begin // I-type (jalr)
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            end
            
            default: begin
                immediate = 32'b0; // Default value
            end
        endcase
    end

endmodule