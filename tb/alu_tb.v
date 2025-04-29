`timescale 1ns/1ps

module alu_tb;
    // Define input signals
    reg [31:0] a;
    reg [31:0] b;
    reg [3:0] alu_control;

    // Define output signals
    wire [31:0] result;
    wire zero;

    // Instantiate the ALU
    alu uut (
        .a(a),
        .b(b),
        .alu_control(alu_control),
        .result(result),
        .zero(zero)
    );

    initial begin
        // Initialize test case
        a = 0; b = 0; alu_control = 0;
        #10; // Wait 10ns
        
        // Test addition
        a = 32'd5; b = 32'd7; alu_control = 4'b0000;
        #10;
        if (result !== 32'd12) $display("Addition test failed");
        else $display("Addition test passed");
        
        // Test subtraction
        a = 32'd10; b = 32'd3; alu_control = 4'b0001;
        #10;
        if (result !== 32'd7) $display("Subtraction test failed");
        else $display("Subtraction test passed");
        
        // Test AND
        a = 32'hF0F0; b = 32'hFF00; alu_control = 4'b0010;
        #10;
        if (result !== 32'hF000) $display("AND test failed");
        else $display("AND test passed");
        
        // Test OR
        a = 32'hF0F0; b = 32'h0F0F; alu_control = 4'b0011;
        #10;
        if (result !== 32'hFFFF) $display("OR test failed");
        else $display("OR test passed");
        
        // Test zero flag
        a = 32'd5; b = 32'd5; alu_control = 4'b0001; // SUB should give 0
        #10;
        if (zero !== 1'b1) $display("Zero flag test failed");
        else $display("Zero flag test passed");
        
        // End simulation
        #10;
        $finish;
    end
    
    // Monitor changes
    initial begin
        $monitor("Time=%0t a=%0d b=%0d alu_control=%0b result=%0d zero=%0b", 
                 $time, a, b, alu_control, result, zero);
    end

endmodule