/*
    This is a register that is used for pipelining for our RISC-V CPU architecture
*/

module pipeline_reg #(
    parameter WIDTH = 32 //32 bit by default
)(
    input clk,
    input reset,
    input [WIDTH-1:0] data_in,
    output reg [WIDTH-1:0] data_out
);
    always @(posedge clk) begin
        if (reset)
            data_out <= {WIDTH{1'b0}};
        else
            data_out <= data_in;
    end

endmodule