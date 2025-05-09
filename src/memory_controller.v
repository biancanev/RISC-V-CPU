module memory_controller (
    input clk,
    input reset,
    
    // CPU instruction interface
    input [31:0] instr_addr,
    output reg [31:0] instruction,
    input instr_req,
    output reg instr_ready,
    
    // CPU data interface
    input [31:0] data_addr,
    input [31:0] data_write,
    output reg [31:0] data_read,
    input data_req,
    input data_write_en,
    output reg data_ready,
    
    // Memory interface
    output reg [31:0] mem_addr,
    output reg [31:0] mem_write_data,
    input [31:0] mem_read_data,
    output reg mem_read_en,
    output reg mem_write_en,
    input mem_ready
);

    // States for memory controller FSM
    localparam IDLE = 2'b00;
    localparam INSTR_ACCESS = 2'b01;
    localparam DATA_ACCESS = 2'b10;
    
    reg [1:0] state, next_state;
    
    // FSM state update
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end
    
    // FSM next state logic
    always @(*) begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (instr_req)
                    next_state = INSTR_ACCESS;
                else if (data_req)
                    next_state = DATA_ACCESS;
            end
            
            INSTR_ACCESS: begin
                if (mem_ready)
                    next_state = IDLE;
            end
            
            DATA_ACCESS: begin
                if (mem_ready)
                    next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Memory controller outputs
    always @(*) begin
        // Default values
        mem_addr = 32'h0;
        mem_write_data = 32'h0;
        mem_read_en = 1'b0;
        mem_write_en = 1'b0;
        instruction = 32'h0;
        instr_ready = 1'b0;
        data_read = 32'h0;
        data_ready = 1'b0;
        
        case (state)
            IDLE: begin
                // No memory access
            end
            
            INSTR_ACCESS: begin
                mem_addr = instr_addr;
                mem_read_en = 1'b1;
                mem_write_en = 1'b0;
                instruction = mem_read_data;
                instr_ready = mem_ready;
            end
            
            DATA_ACCESS: begin
                mem_addr = data_addr;
                mem_read_en = ~data_write_en;
                mem_write_en = data_write_en;
                mem_write_data = data_write;
                data_read = mem_read_data;
                data_ready = mem_ready;
            end
        endcase
    end

endmodule