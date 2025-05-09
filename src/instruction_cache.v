module instruction_cache #(
    parameter CACHE_SIZE = 1024,       // Cache size in words (4KB)
    parameter BLOCK_SIZE = 4,          // Block size in words (16B)
    parameter ASSOCIATIVITY = 2        // 2-way set associative
)(
    input clk,
    input reset,
    
    // CPU interface
    input [31:0] addr,
    output reg [31:0] data_out,
    input read_req,
    output reg ready,
    
    // Memory controller interface
    output reg [31:0] mem_addr,
    input [31:0] mem_data,
    output reg mem_read_req,
    input mem_ready
);

    // Cache parameters
    localparam SETS = CACHE_SIZE / (BLOCK_SIZE * ASSOCIATIVITY);
    localparam SET_BITS = $clog2(SETS);
    localparam BLOCK_BITS = $clog2(BLOCK_SIZE);
    localparam TAG_BITS = 32 - SET_BITS - BLOCK_BITS - 2; // 2 for byte offset in a word
    
    // Cache storage
    reg [31:0] cache_data [0:SETS-1][0:ASSOCIATIVITY-1][0:BLOCK_SIZE-1];
    reg [TAG_BITS-1:0] cache_tags [0:SETS-1][0:ASSOCIATIVITY-1];
    reg cache_valid [0:SETS-1][0:ASSOCIATIVITY-1];
    reg [ASSOCIATIVITY-1:0] lru [0:SETS-1]; // LRU bits
    
    // Cache addressing
    wire [SET_BITS-1:0] set_index = addr[SET_BITS+BLOCK_BITS+1:BLOCK_BITS+2];
    wire [BLOCK_BITS-1:0] block_offset = addr[BLOCK_BITS+1:2];
    wire [TAG_BITS-1:0] tag = addr[31:SET_BITS+BLOCK_BITS+2];
    
    // Cache controller states
    localparam IDLE = 2'b00;
    localparam COMPARE_TAG = 2'b01;
    localparam ALLOCATE = 2'b10;
    
    reg [1:0] state, next_state;
    
    // Temporary variables
    reg hit;
    reg [ASSOCIATIVITY-1:0] hit_way;
    reg [ASSOCIATIVITY-1:0] replace_way;
    integer i, j;
    
    // FSM state update
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            
            // Reset cache valid bits
            for (i = 0; i < SETS; i = i + 1) begin
                for (j = 0; j < ASSOCIATIVITY; j = j + 1) begin
                    cache_valid[i][j] <= 1'b0;
                end
                lru[i] <= 0;
            end
        end
        else begin
            state <= next_state;
        end
    end
    
    // Cache hit/miss check
    always @(*) begin
        hit = 1'b0;
        hit_way = 0;
        
        for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
            if (cache_valid[set_index][i] && cache_tags[set_index][i] == tag) begin
                hit = 1'b1;
                hit_way = i;
            end
        end
    end
    
    // LRU way selection for replacement
    always @(*) begin
        replace_way = ~lru[set_index];
    end
    
    // FSM next state logic
    always @(*) begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (read_req)
                    next_state = COMPARE_TAG;
            end
            
            COMPARE_TAG: begin
                if (hit)
                    next_state = IDLE;
                else
                    next_state = ALLOCATE;
            end
            
            ALLOCATE: begin
                if (mem_ready)
                    next_state = COMPARE_TAG;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Cache controller outputs
    always @(posedge clk) begin
        case (state)
            IDLE: begin
                ready <= 1'b0;
                mem_read_req <= 1'b0;
            end
            
            COMPARE_TAG: begin
                if (hit) begin
                    data_out <= cache_data[set_index][hit_way][block_offset];
                    ready <= 1'b1;
                    // Update LRU bits
                    lru[set_index] <= hit_way;
                end else begin
                    mem_addr <= {addr[31:BLOCK_BITS+2], {(BLOCK_BITS+2){1'b0}}};
                    mem_read_req <= 1'b1;
                    ready <= 1'b0;
                end
            end
            
            ALLOCATE: begin
                if (mem_ready) begin
                    // Update cache with memory data
                    for (i = 0; i < BLOCK_SIZE; i = i + 1) begin
                        cache_data[set_index][replace_way][i] <= mem_data; // This would be a block in real hardware
                    end
                    cache_tags[set_index][replace_way] <= tag;
                    cache_valid[set_index][replace_way] <= 1'b1;
                    lru[set_index] <= replace_way;
                    mem_read_req <= 1'b0;
                end
            end
        endcase
    end

endmodule