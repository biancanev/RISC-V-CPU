`timescale 1ns/1ps

module register_file_tb;
    // Inputs
    reg clk;
    reg reset;
    reg [4:0] rs1_addr;
    reg [4:0] rs2_addr;
    reg [4:0] rd_addr;
    reg [31:0] write_data;
    reg reg_write;
    
    // Outputs
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    
    // Instantiate the register file
    register_file uut (
        .clk(clk),
        .reset(reset),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rd_addr(rd_addr),
        .write_data(write_data),
        .reg_write(reg_write),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );
    
    // Clock generation
    always begin
        #5 clk = ~clk; // 10ns clock period
    end
    
    // Waveform generation
    initial begin
        $dumpfile("register_file_test.vcd");
        $dumpvars(0, register_file_tb);
    end
    
    // Test vector generator
    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        rs1_addr = 0;
        rs2_addr = 0;
        rd_addr = 0;
        write_data = 0;
        reg_write = 0;
        
        // Apply reset
        #20;
        reset = 0;
        
        // Test 1: Write to register x1
        rd_addr = 5'd1;
        write_data = 32'hABCD1234;
        reg_write = 1;
        #10; // Wait for one clock cycle
        
        // Test 2: Read from register x1
        rs1_addr = 5'd1;
        reg_write = 0;
        #10;
        if (rs1_data !== 32'hABCD1234) 
            $display("Test 2 failed: Expected 0xABCD1234, got %h", rs1_data);
        else 
            $display("Test 2 passed");
        
        // Test 3: Write to register x2
        rd_addr = 5'd2;
        write_data = 32'h87654321;
        reg_write = 1;
        #10;
        
        // Test 4: Read from registers x1 and x2
        rs1_addr = 5'd1;
        rs2_addr = 5'd2;
        reg_write = 0;
        #10;
        if (rs1_data !== 32'hABCD1234 || rs2_data !== 32'h87654321)
            $display("Test 4 failed");
        else
            $display("Test 4 passed");
        
        // Test 5: Attempt to write to x0 (should remain 0)
        rd_addr = 5'd0;
        write_data = 32'h12345678;
        reg_write = 1;
        #10;
        
        // Verify x0 is still 0
        rs1_addr = 5'd0;
        reg_write = 0;
        #10;
        if (rs1_data !== 32'b0)
            $display("Test 5 failed: x0 should be hardwired to zero");
        else
            $display("Test 5 passed: x0 is hardwired to zero");
        
        // End simulation
        #10;
        $finish;
    end
    
    // Monitor changes
    initial begin
        $monitor("Time=%0t reset=%b rs1_addr=%d rs2_addr=%d rd_addr=%d write_data=%h reg_write=%b rs1_data=%h rs2_data=%h", 
                $time, reset, rs1_addr, rs2_addr, rd_addr, write_data, reg_write, rs1_data, rs2_data);
    end
endmodule