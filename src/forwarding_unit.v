module forwarding_unit(
    input [4:0] id_ex_rs1_addr,  // Register source 1 address in EX stage
    input [4:0] id_ex_rs2_addr,  // Register source 2 address in EX stage
    input [4:0] ex_mem_rd_addr,  // Destination register address in MEM stage
    input [4:0] mem_wb_rd_addr,  // Destination register address in WB stage

    input ex_mem_reg_write,      // Register write enable in MEM stage
    input mem_wb_reg_write,      // Register write enable in WB stage

    output reg [1:0] forward_a,  // Forwarding control for ALU input A
    output reg [1:0] forward_b,  // Forwarding control for ALU input B

    input [4:0] mem_mem_rs2_addr, // Register source 2 address in MEM stage (for store instructions)
    input mem_mem_write,          // Memory write signal in MEM stage
    output reg forward_mem        // Forwarding control for memory data
);

    localparam NO_FORWARD = 2'b00;    // No forwarding needed
    localparam FORWARD_FROM_WB = 2'b01; // Forward from WB stage
    localparam FORWARD_FROM_MEM = 2'b10; // Forward from MEM stage

    always @(*) begin
        forward_a = NO_FORWARD;
        forward_b = NO_FORWARD;

        if (ex_mem_reg_write && ex_mem_rd_addr != 5'b0 && 
            ex_mem_rd_addr == id_ex_rs1_addr) begin
            forward_a = FORWARD_FROM_MEM;
        end

        if (ex_mem_reg_write && ex_mem_rd_addr != 5'b0 && 
            ex_mem_rd_addr == id_ex_rs2_addr) begin
            forward_b = FORWARD_FROM_MEM;
        end

        if (mem_wb_reg_write && mem_wb_rd_addr != 5'b0 && 
            mem_wb_rd_addr == id_ex_rs1_addr && 
            !(ex_mem_reg_write && ex_mem_rd_addr != 5'b0 && ex_mem_rd_addr == id_ex_rs1_addr)) begin
            forward_a = FORWARD_FROM_WB;
        end

        if (mem_wb_reg_write && mem_wb_rd_addr != 5'b0 && 
            mem_wb_rd_addr == id_ex_rs2_addr && 
            !(ex_mem_reg_write && ex_mem_rd_addr != 5'b0 && ex_mem_rd_addr == id_ex_rs2_addr)) begin
            forward_b = FORWARD_FROM_WB;
        end
    end

    always @(*) begin
        forward_mem = 1'b0; // Default no forwarding for memory writes
        if (mem_mem_write && mem_wb_reg_write && mem_wb_rd_addr != 5'b0 && 
            mem_wb_rd_addr == mem_mem_rs2_addr) begin
            forward_mem = 1'b1;
        end
    end
endmodule