module branch_predictor(
    input clk,
    input reset,
    input [31:0] branch_pc,     // PC of the branch instruction
    input branch_outcome,       // Actual branch outcome (1: taken, 0: not taken)
    input branch_resolved,      // Signal indicating branch outcome is known
    output reg branch_prediction // Branch prediction (1: taken, 0: not taken)
);

    // For now we will use a simple static predictor
    always @(*) begin
        branch_prediction = (branch_pc[2] == 1'b0) ? 1'b0 : 1'b1;
    end
endmodule