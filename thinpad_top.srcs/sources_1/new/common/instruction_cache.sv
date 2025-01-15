module InstructionCache (
    input wire clk,
    input wire rst,
    input wire [31:0] addr,          // if 模块传来的地址
    input wire read_en,              // 读取使能
    output reg [31:0] data,          // 输出数据
    output reg hit,                  // 缓存命中标志

    // 与 IF 模块交互
    input wire [31:0] if_mem_data,   // 从 IF 模块读取的数据
    input wire if_mem_ready,          // 写入使能

    input wire fence_i
);

localparam CACHE_SIZE = 32;         // 每路缓存大小
localparam INDEX_BITS = 5;           // 索引位数
localparam TAG_BITS = 17;            // 标签位数

typedef struct packed {
    logic valid;
    logic [TAG_BITS-1:0] tag;
    logic [31:0] data;
} cache_line_t;

// 缓存存储器
cache_line_t cache_mem0[CACHE_SIZE];
cache_line_t cache_mem1[CACHE_SIZE];

// 索引和标签
logic [INDEX_BITS-1:0] index;
logic [TAG_BITS-1:0] tag;

// 决定哪一行应该被替换
logic lru[CACHE_SIZE];

always_comb begin
    hit = 0;
    data = 32'b0;

    index = addr[INDEX_BITS+1:2];
    tag = addr[23:INDEX_BITS+2];
    
    // 检查缓存命中
    if (cache_mem0[index].valid && cache_mem0[index].tag == tag) begin
        hit = 1;
        data = cache_mem0[index].data;
    end else if (cache_mem1[index].valid && cache_mem1[index].tag == tag) begin
        hit = 1;
        data = cache_mem1[index].data;
    end
end

// 缓存更新逻辑已禁用
always_ff @(posedge clk) begin
    if (rst) begin
        for (int i = 0; i < CACHE_SIZE; i++) begin
            cache_mem0[i] <= 0;  
            cache_mem1[i] <= 0;  
            lru[i] <= 0;  
        end
    end else begin
        if(fence_i) begin
        //清空缓存
            for(int i = 0; i < CACHE_SIZE; i++) begin
                cache_mem0[i].valid = 0;
                cache_mem1[i].valid = 0;
            end
        end
        if (if_mem_ready && !hit) begin
            if (lru[index] == 0) begin
                cache_mem0[index].valid <= 1;
                cache_mem0[index].tag <= tag;
                cache_mem0[index].data <= if_mem_data;
                lru[index] <= 1; // 更新 LRU 位
            end else begin
                cache_mem1[index].valid <= 1;
                cache_mem1[index].tag <= tag;
                cache_mem1[index].data <= if_mem_data;
                lru[index] <= 0; // 更新 LRU 位
            end
        end else if (hit) begin
            if (cache_mem0[index].valid && cache_mem0[index].tag == tag) begin
                lru[index] <= 1;
            end else if (cache_mem1[index].valid && cache_mem1[index].tag == tag) begin
                lru[index] <= 0;
            end
        end
    end
end
endmodule