module hazard_detection_unit(
    input [4:0] if_id_rs1_addr,      // RS1 address in ID stage
    input [4:0] if_id_rs2_addr,      // RS2 address in ID stage
    input [4:0] id_ex_rd_addr,       // Destination register in EX stage
    input id_ex_mem_read,            // Memory read signal in EX stage
    input branch_taken,              // Branch taken signal
    input jump,                      // Jump instruction signal

    output reg stall_pipeline,       // Stall signal for pipeline
    output reg flush_pipeline        // Flush signal for pipeline
);

    always @(*) begin
        stall_pipeline = 1'b0;
        flush_pipeline = 1'b0;

        if (id_ex_mem_read && 
            ((id_ex_rd_addr == if_id_rs1_addr && if_id_rs1_addr != 5'b0) || 
             (id_ex_rd_addr == if_id_rs2_addr && if_id_rs2_addr != 5'b0))) begin
            stall_pipeline = 1'b1;   // Stall the pipeline
        end

        if (branch_taken || jump) begin
            flush_pipeline = 1'b1;   // Flush the pipeline
        end
    end
endmodule