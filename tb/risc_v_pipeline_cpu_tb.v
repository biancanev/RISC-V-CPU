// risc_v_pipeline_tb.v - Testbench for Pipelined RISC-V Processor

`timescale 1ns/1ps

module risc_v_pipeline_tb;
    // Clock and reset signals
    reg clk;
    reg reset;
    
    // Memory interface signals
    wire [31:0] instr_addr;
    reg [31:0] instruction;
    wire [31:0] data_addr;
    wire [31:0] data_write;
    reg [31:0] data_read;
    wire mem_write_en;
    wire mem_read_en;
    
    // Cycle counter
    integer cycle_count;
    
    // For instruction and data memory simulation
    reg [31:0] instr_memory [0:1023];
    reg [31:0] data_memory [0:1023];
    
    // For visualization
    integer i;
    reg [4:0] pipeline_stage [0:4]; // Track instruction in each stage
    reg [31:0] pipeline_pc [0:4];   // Track PC in each stage
    
    // Instantiate the CPU
    risc_v_pipeline_cpu cpu(
        .clk(clk),
        .reset(reset),
        .instr_addr(instr_addr),
        .instruction(instruction),
        .data_addr(data_addr),
        .data_write(data_write),
        .data_read(data_read),
        .mem_write_en(mem_write_en),
        .mem_read_en(mem_read_en)
    );
    
    // Clock generation
    always begin
        #5 clk = ~clk; // 10ns clock period (100 MHz)
    end
    
    // Instruction memory read simulation
    always @(*) begin
        instruction = instr_memory[instr_addr[11:2]]; // Word-aligned access
    end
    
    // Data memory read/write simulation
    always @(posedge clk) begin
        if (mem_write_en)
            data_memory[data_addr[11:2]] <= data_write;
    end
    
    always @(*) begin
        if (mem_read_en)
            data_read = data_memory[data_addr[11:2]];
        else
            data_read = 32'h00000000;
    end
    
    // Instruction decoder task - no return value, just display
    task display_instruction;
        input [31:0] instr;
        
        reg [6:0] opcode;
        reg [2:0] funct3;
        reg [6:0] funct7;
        reg [4:0] rd, rs1, rs2;
        reg [11:0] imm_i;
        reg [11:0] imm_s;
        reg [12:0] imm_b;
        reg [31:0] imm_u;
        reg [20:0] imm_j;
        
        begin
            opcode = instr[6:0];
            rd = instr[11:7];
            funct3 = instr[14:12];
            rs1 = instr[19:15];
            rs2 = instr[24:20];
            funct7 = instr[31:25];
            
            imm_i = instr[31:20];
            imm_s = {instr[31:25], instr[11:7]};
            imm_b = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            imm_u = {instr[31:12], 12'b0};
            imm_j = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            
            case (opcode)
                7'b0110011: begin // R-type
                    case ({funct7[5], funct3})
                        4'b0000: $write("add x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b1000: $write("sub x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0001: $write("sll x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0010: $write("slt x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0011: $write("sltu x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0100: $write("xor x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0101: $write("srl x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b1101: $write("sra x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0110: $write("or x%0d, x%0d, x%0d", rd, rs1, rs2);
                        4'b0111: $write("and x%0d, x%0d, x%0d", rd, rs1, rs2);
                        default: $write("unknown");
                    endcase
                end
                
                7'b0010011: begin // I-type ALU
                    case (funct3)
                        3'b000: $write("addi x%0d, x%0d, %0d", rd, rs1, $signed(imm_i));
                        3'b010: $write("slti x%0d, x%0d, %0d", rd, rs1, $signed(imm_i));
                        3'b011: $write("sltiu x%0d, x%0d, %0d", rd, rs1, imm_i);
                        3'b100: $write("xori x%0d, x%0d, %0d", rd, rs1, $signed(imm_i));
                        3'b110: $write("ori x%0d, x%0d, %0d", rd, rs1, $signed(imm_i));
                        3'b111: $write("andi x%0d, x%0d, %0d", rd, rs1, $signed(imm_i));
                        3'b001: $write("slli x%0d, x%0d, %0d", rd, rs1, instr[24:20]);
                        3'b101: begin
                            if (funct7[5])
                                $write("srai x%0d, x%0d, %0d", rd, rs1, instr[24:20]);
                            else
                                $write("srli x%0d, x%0d, %0d", rd, rs1, instr[24:20]);
                        end
                        default: $write("unknown");
                    endcase
                end
                
                7'b0000011: begin // I-type Load
                    case (funct3)
                        3'b000: $write("lb x%0d, %0d(x%0d)", rd, $signed(imm_i), rs1);
                        3'b001: $write("lh x%0d, %0d(x%0d)", rd, $signed(imm_i), rs1);
                        3'b010: $write("lw x%0d, %0d(x%0d)", rd, $signed(imm_i), rs1);
                        3'b100: $write("lbu x%0d, %0d(x%0d)", rd, $signed(imm_i), rs1);
                        3'b101: $write("lhu x%0d, %0d(x%0d)", rd, $signed(imm_i), rs1);
                        default: $write("unknown");
                    endcase
                end
                
                7'b0100011: begin // S-type
                    case (funct3)
                        3'b000: $write("sb x%0d, %0d(x%0d)", rs2, $signed(imm_s), rs1);
                        3'b001: $write("sh x%0d, %0d(x%0d)", rs2, $signed(imm_s), rs1);
                        3'b010: $write("sw x%0d, %0d(x%0d)", rs2, $signed(imm_s), rs1);
                        default: $write("unknown");
                    endcase
                end
                
                7'b1100011: begin // B-type
                    case (funct3)
                        3'b000: $write("beq x%0d, x%0d, %0d", rs1, rs2, $signed(imm_b));
                        3'b001: $write("bne x%0d, x%0d, %0d", rs1, rs2, $signed(imm_b));
                        3'b100: $write("blt x%0d, x%0d, %0d", rs1, rs2, $signed(imm_b));
                        3'b101: $write("bge x%0d, x%0d, %0d", rs1, rs2, $signed(imm_b));
                        3'b110: $write("bltu x%0d, x%0d, %0d", rs1, rs2, $signed(imm_b));
                        3'b111: $write("bgeu x%0d, x%0d, %0d", rs1, rs2, $signed(imm_b));
                        default: $write("unknown");
                    endcase
                end
                
                7'b0110111: $write("lui x%0d, 0x%0h", rd, instr[31:12]);
                7'b0010111: $write("auipc x%0d, 0x%0h", rd, instr[31:12]);
                7'b1101111: $write("jal x%0d, %0d", rd, $signed(imm_j));
                7'b1100111: $write("jalr x%0d, x%0d, %0d", rd, rs1, $signed(imm_i));
                
                default: $write("unknown");
            endcase
        end
    endtask
    
    // Function to get register name as a string (can still return a value)
    function [31:0] reg_name;
        input [4:0] reg_num;
        begin
            case (reg_num)
                5'd0: reg_name = "zero";
                5'd1: reg_name = "ra";
                5'd2: reg_name = "sp";
                5'd3: reg_name = "gp";
                5'd4: reg_name = "tp";
                5'd5: reg_name = "t0";
                5'd6: reg_name = "t1";
                5'd7: reg_name = "t2";
                5'd8: reg_name = "s0";
                5'd9: reg_name = "s1";
                5'd10: reg_name = "a0";
                5'd11: reg_name = "a1";
                5'd12: reg_name = "a2";
                5'd13: reg_name = "a3";
                5'd14: reg_name = "a4";
                5'd15: reg_name = "a5";
                5'd16: reg_name = "a6";
                5'd17: reg_name = "a7";
                5'd18: reg_name = "s2";
                5'd19: reg_name = "s3";
                5'd20: reg_name = "s4";
                5'd21: reg_name = "s5";
                5'd22: reg_name = "s6";
                5'd23: reg_name = "s7";
                5'd24: reg_name = "s8";
                5'd25: reg_name = "s9";
                5'd26: reg_name = "s10";
                5'd27: reg_name = "s11";
                5'd28: reg_name = "t3";
                5'd29: reg_name = "t4";
                5'd30: reg_name = "t5";
                5'd31: reg_name = "t6";
                default: reg_name = "x??";
            endcase
        end
    endfunction
    
    // Test program
    initial begin
        // Simple test program to calculate first few Fibonacci numbers
        // Registers used:
        // x10 (a0) - current fibonacci number
        // x11 (a1) - next fibonacci number
        // x12 (a2) - temporary for addition
        // x13 (a3) - loop counter
        // x14 (a4) - loop limit
        // Memory layout:
        // 0x00: Program start
        // 0x100: Data section (results stored here)
        
        // Initialize memory with program
        instr_memory[0] = 32'h00000293;  // addi t0, zero, 0    # i = 0
        instr_memory[1] = 32'h00100313;  // addi t1, zero, 1    # a = 1
        instr_memory[2] = 32'h00100393;  // addi t2, zero, 1    # b = 1
        instr_memory[3] = 32'h00A00e13;  // addi t3, zero, 10   # limit = 10
        instr_memory[4] = 32'h00500023;  // sb t0, 0(zero)      # mem[0] = i
        instr_memory[5] = 32'h00628023;  // sb t2, 0(t0)        # mem[i] = b
        instr_memory[6] = 32'h006283B3;  // add t2, t0, t1      # b = a + b
        instr_memory[7] = 32'h00030333;  // add t1, t2, zero    # a = b
        instr_memory[8] = 32'h00128293;  // addi t0, t0, 1      # i++
        instr_memory[9] = 32'hFDC2CAE3;  // blt t0, t3, loop    # if i < limit goto loop
        instr_memory[10] = 32'h00100073; // ebreak              # End simulation
        
        // Clear data memory
        for (i = 0; i < 1024; i = i + 1)
            data_memory[i] = 32'h00000000;
            
        // Initialize simulation signals
        clk = 0;
        reset = 1;
        cycle_count = 0;
        
        // Initialize pipeline visualization
        for (i = 0; i < 5; i = i + 1) begin
            pipeline_stage[i] = 5'h1F; // Invalid register
            pipeline_pc[i] = 32'hxxxxxxxx;
        end
        
        // Apply reset for 2 clock cycles
        #20 reset = 0;
        
        // Run for 100 cycles or until ebreak
        for (i = 0; i < 100; i = i + 1) begin
            @(posedge clk);
            cycle_count = cycle_count + 1;
            
            // Detect ebreak instruction to end simulation
            if (cpu.if_id_instruction == 32'h00100073 && 
                cpu.if_id_pc >= 4*10) begin
                #10;
                $display("EBREAK instruction detected - ending simulation");
                i = 100; // This effectively breaks out of the loop
            end
        end
        
        // Display final state of data memory
        $display("\nFinal Data Memory (First 16 words):");
        $display("Addr \tValue \t\tDecimal");
        for (i = 0; i < 16; i = i + 1) begin
            $display("0x%h\t0x%h\t%d", i*4, data_memory[i], data_memory[i]);
        end
        
        // End simulation
        #20 $finish;
    end
    
    // Waveform dump
    initial begin
        $dumpfile("cpu_simulation.vcd");
        $dumpvars(0, risc_v_pipeline_tb);
    end
    
    // Pipeline visualization
    always @(posedge clk) begin
        if (!reset) begin
            // Update pipeline stage visualization
            $display("\n==== CYCLE %0d ====", cycle_count);
            
            // Display pipeline stages with instruction mnemonics
            $display("----- PIPELINE STATE -----");
            
            // IF stage
            $write("IF:  PC=0x%h, Instr=%h [", cpu.pc, cpu.if_instruction);
            display_instruction(cpu.if_instruction);
            $display("]");
            
            // ID stage
            $write("ID:  PC=0x%h, Instr=%h [", cpu.if_id_pc, cpu.if_id_instruction);
            display_instruction(cpu.if_id_instruction);
            $display("]");
            
            // EX stage
            $display("EX:  PC=0x%h, RS1=%h, RS2=%h, IMM=%h", 
                    cpu.id_ex_pc, cpu.id_ex_rs1_data, cpu.id_ex_rs2_data, 
                    cpu.id_ex_immediate);
            $display("     ALUOp=%b, ALUCtrl=%b, ALUSrc=%b", 
                    cpu.id_ex_alu_op, cpu.ex_alu_control, cpu.id_ex_alu_src);
            $display("     RegWrite=%b, RD=%d, Result=%h", 
                    cpu.id_ex_reg_write, cpu.id_ex_rd_addr, cpu.ex_alu_result);
            
            // MEM stage
            $display("MEM: PC=0x%h, ALUResult=%h, MemWrite=%b, MemRead=%b", 
                    cpu.ex_mem_pc_plus4 - 4, cpu.ex_mem_alu_result, 
                    cpu.ex_mem_mem_write, cpu.ex_mem_mem_read);
            $display("     Branch=%b, Zero=%b, BranchTaken=%b", 
                    cpu.ex_mem_branch, cpu.ex_mem_zero_flag, cpu.ex_mem_branch_taken);
            
            // WB stage
            $display("WB:  RegWrite=%b, RD=%d, Data=%h", 
                    cpu.mem_wb_reg_write, cpu.mem_wb_rd_addr, cpu.wb_write_data);
            
            // Display forwarding information
            if (cpu.forward_a != 2'b00 || cpu.forward_b != 2'b00) begin
                $display("----- FORWARDING -----");
                if (cpu.forward_a != 2'b00) begin
                    $write("RS1 Forwarding: %b - from %s to ", cpu.forward_a, 
                            (cpu.forward_a == 2'b01) ? "WB" : "MEM");
                    $write("%s", reg_name(cpu.id_ex_rs1_addr));
                    $display("");
                end
                
                if (cpu.forward_b != 2'b00) begin
                    $write("RS2 Forwarding: %b - from %s to ", cpu.forward_b, 
                            (cpu.forward_b == 2'b01) ? "WB" : "MEM");
                    $write("%s", reg_name(cpu.id_ex_rs2_addr));
                    $display("");
                end
            end
            
            // Display hazard information
            if (cpu.stall_pipeline || cpu.flush_pipeline) begin
                $display("----- HAZARDS -----");
                if (cpu.stall_pipeline)
                    $display("Pipeline Stall detected");
                if (cpu.flush_pipeline)
                    $display("Pipeline Flush detected");
            end
            
            // Display register file updates
            if (cpu.mem_wb_reg_write && cpu.mem_wb_rd_addr != 0) begin
                $display("----- REGISTER UPDATE -----");
                $write("x%0d (", cpu.mem_wb_rd_addr);
                $write("%s", reg_name(cpu.mem_wb_rd_addr));
                $display(") = 0x%h", cpu.wb_write_data);
            end
            
            // Display memory updates
            if (cpu.ex_mem_mem_write) begin
                $display("----- MEMORY UPDATE -----");
                $display("MEM[0x%h] = 0x%h", 
                        cpu.ex_mem_alu_result, 
                        cpu.ex_mem_rs2_data);
            end
            
            $display("------------------------");
        end
    end
    
    // Track simulation progress
    always @(posedge clk) begin
        if (!reset && cycle_count % 10 == 0) begin
            $display("\n*** Completed %0d simulation cycles ***", cycle_count);
        end
    end

endmodule