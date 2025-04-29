/*
    Controller for pipeline cpu
*/

module pipeline_control(
    input clk,
    input reset,
    input branch_taken,         // Actual branch outcome
    input branch_prediction,    // Predicted branch outcome
    input load_use_hazard,      // Load-use hazard detected
    output reg stall_if,        // Stall instruction fetch stage
    output reg stall_id,        // Stall instruction decode stage
    output reg flush_if,        // Flush instruction fetch stage
    output reg flush_id,        // Flush instruction decode stage
    output reg flush_ex         // Flush execute stage
);

    always @(*) begin
        // Default: no stalls or flushes
        stall_if = 1'b0;
        stall_id = 1'b0;
        flush_if = 1'b0;
        flush_id = 1'b0;
        flush_ex = 1'b0;
        
        // Handle load-use hazards
        if (load_use_hazard) begin
            stall_if = 1'b1;
            stall_id = 1'b1;
        end
        
        // Handle branch mispredictions
        if (branch_taken != branch_prediction) begin
            flush_if = 1'b1;
            flush_id = 1'b1;
            flush_ex = 1'b1;
        end
    end

endmodule