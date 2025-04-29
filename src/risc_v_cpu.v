module risc_v_pipeline_cpu(
    input clk,                  // Clock input
    input reset,                // Reset signal
    // Memory interfaces
    output [31:0] instr_addr,   // Instruction memory address
    input [31:0] instruction,   // Instruction fetched from memory
    output [31:0] data_addr,    // Data memory address
    output [31:0] data_write,   // Data to write to memory
    input [31:0] data_read,     // Data read from memory
    output mem_write_en,        // Memory write enable
    output mem_read_en          // Memory read enable
);

    reg [31:0] pc;
    wire [31:0] pc_next;
    wire [31:0] pc_plus4;
    wire [31:0] branch_target;
    wire pc_write;

    wire [31:0] if_instruction;

    reg [31:0] if_id_pc;
    reg [31:0] if_id_instruction;
    wire if_id_write;           // From hazard detection unit
    wire if_id_flush;           // From hazard detection unit or branch/jump

    wire [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr;
    wire [31:0] id_rs1_data, id_rs2_data;
    wire [31:0] id_immediate;
    wire [2:0] id_funct3;
    wire [6:0] id_funct7;

    wire [1:0] id_alu_op;
    wire id_alu_src;
    wire id_mem_to_reg;
    wire id_reg_write;
    wire id_mem_read;
    wire id_mem_write;
    wire id_branch;
    wire id_jump;

    wire stall_pipeline;
    wire flush_pipeline;

    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_immediate;
    reg [4:0] id_ex_rs1_addr;
    reg [4:0] id_ex_rs2_addr;
    reg [4:0] id_ex_rd_addr;
    reg [2:0] id_ex_funct3;
    reg [6:0] id_ex_funct7;

    reg [1:0] id_ex_alu_op;
    reg id_ex_alu_src;
    reg id_ex_mem_to_reg;
    reg id_ex_reg_write;
    reg id_ex_mem_read;
    reg id_ex_mem_write;
    reg id_ex_branch;
    reg id_ex_jump;

    wire [3:0] ex_alu_control;
    wire [31:0] ex_alu_result;
    wire ex_zero_flag;
    wire ex_branch_taken;

    wire [1:0] forward_a, forward_b;
    wire [31:0] ex_alu_input_a, ex_alu_input_b;
    wire [31:0] ex_alu_input_b_mux_out;

    reg [31:0] ex_mem_pc_plus4;
    reg [31:0] ex_mem_branch_target;
    reg ex_mem_zero_flag;
    reg ex_mem_branch_taken;
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_rs2_data;
    reg [4:0] ex_mem_rd_addr;
    reg [2:0] ex_mem_funct3;

    reg ex_mem_mem_to_reg;
    reg ex_mem_reg_write;
    reg ex_mem_mem_read;
    reg ex_mem_mem_write;
    reg ex_mem_branch;
    reg ex_mem_jump;

    wire mem_pc_src;
    wire [31:0] mem_read_data;

    reg [31:0] mem_wb_alu_result;
    reg [31:0] mem_wb_read_data;
    reg [4:0] mem_wb_rd_addr;

    reg mem_wb_mem_to_reg;
    reg mem_wb_reg_write;

    wire [31:0] wb_write_data;

    wire branch_prediction;
    wire branch_resolved;
    wire actual_branch_outcome;

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 32'h00000000;
        else if (pc_write)
            pc <= pc_next;
    end

    assign pc_plus4 = pc + 32'd4;

    assign pc_next = mem_pc_src ? ex_mem_branch_target : pc_plus4;

    assign instr_addr = pc;
    assign if_instruction = instruction;

    branch_predictor branch_pred(
        .clk(clk),
        .reset(reset),
        .branch_pc(id_ex_pc),
        .branch_outcome(actual_branch_outcome),
        .branch_resolved(branch_resolved),
        .branch_prediction(branch_prediction)
    );

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            if_id_pc <= 32'h00000000;
            if_id_instruction <= 32'h00000000;
        end
        else if (if_id_flush) begin
            if_id_pc <= 32'h00000000;
            if_id_instruction <= 32'h00000000;
        end
        else if (if_id_write) begin
            if_id_pc <= pc;
            if_id_instruction <= if_instruction;
        end
    end

    assign id_rs1_addr = if_id_instruction[19:15];
    assign id_rs2_addr = if_id_instruction[24:20];
    assign id_rd_addr = if_id_instruction[11:7];
    assign id_funct3 = if_id_instruction[14:12];
    assign id_funct7 = if_id_instruction[31:25];

    register_file registers(
        .clk(clk),
        .reset(reset),
        .rs1_addr(id_rs1_addr),
        .rs2_addr(id_rs2_addr),
        .rd_addr(mem_wb_rd_addr),
        .write_data(wb_write_data),
        .reg_write(mem_wb_reg_write),
        .rs1_data(id_rs1_data),
        .rs2_data(id_rs2_data)
    );

    immediate_gen imm_gen(
        .instruction(if_id_instruction),
        .immediate(id_immediate)
    );

    pipelined_control_unit control_unit(
        .opcode(if_id_instruction[6:0]),
        .funct3(id_funct3),
        .funct7(id_funct7),
        .stall(stall_pipeline),
        .flush(flush_pipeline),
        .alu_op(id_alu_op),
        .alu_src(id_alu_src),
        .mem_to_reg(id_mem_to_reg),
        .reg_write(id_reg_write),
        .mem_read(id_mem_read),
        .mem_write(id_mem_write),
        .branch(id_branch),
        .jump(id_jump),
        .pc_write(pc_write)
    );

    hazard_detection_unit hazard_detect(
        .if_id_rs1_addr(id_rs1_addr),
        .if_id_rs2_addr(id_rs2_addr),
        .id_ex_rd_addr(id_ex_rd_addr),
        .id_ex_mem_read(id_ex_mem_read),
        .branch_taken(ex_mem_branch_taken),
        .jump(id_ex_jump),
        .stall_pipeline(stall_pipeline),
        .flush_pipeline(flush_pipeline)
    );

    assign if_id_write = ~stall_pipeline;
    assign if_id_flush = flush_pipeline;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all ID/EX pipeline registers
            id_ex_pc <= 32'h00000000;
            id_ex_rs1_data <= 32'h00000000;
            id_ex_rs2_data <= 32'h00000000;
            id_ex_immediate <= 32'h00000000;
            id_ex_rs1_addr <= 5'b00000;
            id_ex_rs2_addr <= 5'b00000;
            id_ex_rd_addr <= 5'b00000;
            id_ex_funct3 <= 3'b000;
            id_ex_funct7 <= 7'b0000000;
            
            // Reset control signals
            id_ex_alu_op <= 2'b00;
            id_ex_alu_src <= 1'b0;
            id_ex_mem_to_reg <= 1'b0;
            id_ex_reg_write <= 1'b0;
            id_ex_mem_read <= 1'b0;
            id_ex_mem_write <= 1'b0;
            id_ex_branch <= 1'b0;
            id_ex_jump <= 1'b0;
        end
        else if (flush_pipeline) begin
            // Flush ID/EX pipeline registers on hazard
            id_ex_pc <= 32'h00000000;
            id_ex_rs1_data <= 32'h00000000;
            id_ex_rs2_data <= 32'h00000000;
            id_ex_immediate <= 32'h00000000;
            id_ex_rs1_addr <= 5'b00000;
            id_ex_rs2_addr <= 5'b00000;
            id_ex_rd_addr <= 5'b00000;
            id_ex_funct3 <= 3'b000;
            id_ex_funct7 <= 7'b0000000;
            
            // Reset control signals
            id_ex_alu_op <= 2'b00;
            id_ex_alu_src <= 1'b0;
            id_ex_mem_to_reg <= 1'b0;
            id_ex_reg_write <= 1'b0;
            id_ex_mem_read <= 1'b0;
            id_ex_mem_write <= 1'b0;
            id_ex_branch <= 1'b0;
            id_ex_jump <= 1'b0;
        end
        else begin
            // Normal pipeline register update
            id_ex_pc <= if_id_pc;
            id_ex_rs1_data <= id_rs1_data;
            id_ex_rs2_data <= id_rs2_data;
            id_ex_immediate <= id_immediate;
            id_ex_rs1_addr <= id_rs1_addr;
            id_ex_rs2_addr <= id_rs2_addr;
            id_ex_rd_addr <= id_rd_addr;
            id_ex_funct3 <= id_funct3;
            id_ex_funct7 <= id_funct7;
            
            // Update control signals
            id_ex_alu_op <= id_alu_op;
            id_ex_alu_src <= id_alu_src;
            id_ex_mem_to_reg <= id_mem_to_reg;
            id_ex_reg_write <= id_reg_write;
            id_ex_mem_read <= id_mem_read;
            id_ex_mem_write <= id_mem_write;
            id_ex_branch <= id_branch;
            id_ex_jump <= id_jump;
        end
    end

    alu_control alu_ctrl(
        .alu_op(id_ex_alu_op),
        .funct3(id_ex_funct3),
        .funct7(id_ex_funct7),
        .alu_control(ex_alu_control)
    );

    forwarding_unit forwarding(
        .id_ex_rs1_addr(id_ex_rs1_addr),
        .id_ex_rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_reg_write(mem_wb_reg_write),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .mem_mem_rs2_addr(ex_mem_rs2_addr),
        .mem_mem_write(ex_mem_mem_write),
        .forward_mem()
    );

    always @(*) begin
        case (forward_a)
            2'b00: ex_alu_input_a = id_ex_rs1_data;           // No forwarding
            2'b01: ex_alu_input_a = wb_write_data;            // Forward from WB
            2'b10: ex_alu_input_a = ex_mem_alu_result;        // Forward from MEM
            default: ex_alu_input_a = id_ex_rs1_data;
        endcase
    end

    always @(*) begin
        case (forward_b)
            2'b00: ex_alu_input_b_mux_out = id_ex_rs2_data;   // No forwarding
            2'b01: ex_alu_input_b_mux_out = wb_write_data;    // Forward from WB
            2'b10: ex_alu_input_b_mux_out = ex_mem_alu_result; // Forward from MEM
            default: ex_alu_input_b_mux_out = id_ex_rs2_data;
        endcase
    end

    assign ex_alu_input_b = id_ex_alu_src ? id_ex_immediate : ex_alu_input_b_mux_out;

    alu alu_unit(
        .a(ex_alu_input_a),
        .b(ex_alu_input_b),
        .alu_control(ex_alu_control),
        .result(ex_alu_result),
        .zero(ex_zero_flag)
    );

    // Branch target address calculation
    assign branch_target = id_ex_pc + id_ex_immediate;
    
    // Branch decision
    assign ex_branch_taken = (id_ex_branch & ex_zero_flag) | id_ex_jump;
    
    // Branch prediction resolution
    assign branch_resolved = id_ex_branch | id_ex_jump;
    assign actual_branch_outcome = ex_branch_taken;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ex_mem_pc_plus4 <= 32'h00000000;
            ex_mem_branch_target <= 32'h00000000;
            ex_mem_zero_flag <= 1'b0;
            ex_mem_branch_taken <= 1'b0;
            ex_mem_alu_result <= 32'h00000000;
            ex_mem_rs2_data <= 32'h00000000;
            ex_mem_rd_addr <= 5'b00000;
            ex_mem_funct3 <= 3'b000;
            
            // Reset control signals
            ex_mem_mem_to_reg <= 1'b0;
            ex_mem_reg_write <= 1'b0;
            ex_mem_mem_read <= 1'b0;
            ex_mem_mem_write <= 1'b0;
            ex_mem_branch <= 1'b0;
            ex_mem_jump <= 1'b0;
        end
        else begin
            ex_mem_pc_plus4 <= pc_plus4;
            ex_mem_branch_target <= branch_target;
            ex_mem_zero_flag <= ex_zero_flag;
            ex_mem_branch_taken <= ex_branch_taken;
            ex_mem_alu_result <= ex_alu_result;
            ex_mem_rs2_data <= ex_alu_input_b_mux_out;
            ex_mem_rd_addr <= id_ex_rd_addr;
            ex_mem_funct3 <= id_ex_funct3;
            
            // Update control signals
            ex_mem_mem_to_reg <= id_ex_mem_to_reg;
            ex_mem_reg_write <= id_ex_reg_write;
            ex_mem_mem_read <= id_ex_mem_read;
            ex_mem_mem_write <= id_ex_mem_write;
            ex_mem_branch <= id_ex_branch;
            ex_mem_jump <= id_ex_jump;
        end
    end

    assign mem_pc_src = ex_mem_branch_taken;

    assign data_addr = ex_mem_alu_result;
    assign data_write = ex_mem_rs2_data;
    assign mem_write_en = ex_mem_mem_write;
    assign mem_read_en = ex_mem_mem_read;
    assign mem_read_data = data_read;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_wb_alu_result <= 32'h00000000;
            mem_wb_read_data <= 32'h00000000;
            mem_wb_rd_addr <= 5'b00000;
            
            // Reset control signals
            mem_wb_mem_to_reg <= 1'b0;
            mem_wb_reg_write <= 1'b0;
        end
        else begin
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_read_data <= mem_read_data;
            mem_wb_rd_addr <= ex_mem_rd_addr;
            
            // Update control signals
            mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
            mem_wb_reg_write <= ex_mem_reg_write;
        end
    end

    assign wb_write_data = mem_wb_mem_to_reg ? mem_wb_read_data : mem_wb_alu_result;


endmodule