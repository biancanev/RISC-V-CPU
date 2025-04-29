/*
    Define 32 general-purpose registers
    Each register is 32 bits wide
*/

module register_file(
    input clk,                    // Clock
    input reset,                  // Reset signal
    input [4:0] rs1_addr,         // Source register 1 address
    input [4:0] rs2_addr,         // Source register 2 address
    input [4:0] rd_addr,          // Destination register address
    input [31:0] write_data,      // Data to write
    input reg_write,              // Register write enable
    output [31:0] rs1_data,       // Source register 1 data
    output [31:0] rs2_data        // Source register 2 data
);

    // List of all 32, 32-bit registers
    reg [31:0] registers [0:31];
        
    // Initialize all registers
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'b0;
        end
    end

    // Define reset behavior
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end
    end

    // Synchronous write behavior (avoid address x0)
    always @(posedge clk) begin
        if (reg_write && rd_addr != 5'b0) begin
            registers[rd_addr] <= write_data;
        end
    end

    //Asynchronous read behavior
    assign rs1_data = (rs1_addr == 5'b0) ? 32'b0 : registers[rs1_addr];
    assign rs2_data = (rs2_addr == 5'b0) ? 32'b0 : registers[rs2_addr];

endmodule