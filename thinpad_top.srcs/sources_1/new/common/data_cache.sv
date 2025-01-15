module data_cache (  
    input wire clk,  
    input wire rst,  

    // 与 MMU 的接口（与总线一致）  
    input wire [31:0] mmu_adr_i,    // 地址输入  
    input wire [31:0] mmu_dat_i,    // 写数据输入  
    output reg [31:0] mmu_dat_o,    // 读数据输出  
    input wire mmu_we_i,            // 写使能输入  
    input wire [3:0] mmu_sel_i,     // 字节选择输入  
    input wire mmu_stb_i,           // 选通信号输入  
    input wire mmu_cyc_i,           // 周期信号输入  
    output reg mmu_ack_o,           // 确认信号输出  
    output reg mmu_err_o,  
    output reg mmu_rty_o,           // 错误信号输出  

    // 与 Wishbone 总线的接口  
    output reg [31:0] wb_adr_o,    // 地址输出  
    output reg [31:0] wb_dat_o,    // 写数据输出到总线  
    input wire [31:0] wb_dat_i,     // 从总线接收的数据  
    output reg wb_we_o,            // 写使能输出到总线  
    output reg [3:0] wb_sel_o,     // 字节选择输出到总线  
    output reg wb_stb_o,           // 选通信号输出到总线  
    output reg wb_cyc_o,           // 周期信号输出到总线  
    input wire wb_ack_i,            // 总线确认信号输入  
    input wire wb_err_i,  
    input wire wb_rty_i             // 总线错误信号输入  
);  

    // 缓存参数  
    localparam CACHE_LINES = 32;          // 缓存行数  
    localparam CACHE_WAYS = 2;            // 2 路组相联  
    localparam CACHE_LINE_SIZE = 4;       // 缓存行大小  
    localparam INDEX_WIDTH = $clog2(CACHE_LINES);  
    localparam OFFSET_WIDTH = $clog2(CACHE_LINE_SIZE);  
    localparam TAG_WIDTH = 32 - INDEX_WIDTH - OFFSET_WIDTH;  

    // 缓存行定义  
    typedef struct packed {  
        logic valid;                      // 有效位  
        logic [TAG_WIDTH-1:0] tag;        // 地址标签  
        logic [31:0] data;                // 数据  
    } cache_line_t;  

    // 缓存存储器  
    cache_line_t cache_mem0[CACHE_LINES];  
    cache_line_t cache_mem1[CACHE_LINES];  

    // 替换策略  
    logic lru[CACHE_LINES]; // 0: cache_mem0 最近使用，1: cache_mem1 最近使用  

    // 地址解析  
    logic [TAG_WIDTH-1:0] tag = mmu_adr_i[31:INDEX_WIDTH + OFFSET_WIDTH];  
    logic [INDEX_WIDTH-1:0] index = mmu_adr_i[INDEX_WIDTH + OFFSET_WIDTH - 1:OFFSET_WIDTH];  
    logic [OFFSET_WIDTH-1:0] offset = mmu_adr_i[OFFSET_WIDTH - 1:0];  

    // 缓存命中标志  
    logic hit0 = cache_mem0[index].valid && (cache_mem0[index].tag == tag);  
    logic hit1 = cache_mem1[index].valid && (cache_mem1[index].tag == tag);  
    logic history_ack = 0;
    // 组合逻辑处理  
    always_comb begin  
        mmu_ack_o = 0;  
        mmu_dat_o = 0;  
        mmu_err_o = 0;  
        mmu_rty_o = 0;  

        // 默认总线信号：保持上一次状态，除非有新的总线操作  
        wb_adr_o = 32'b0;  
        wb_dat_o = 32'b0;  
        wb_we_o  = 1'b0;  
        wb_sel_o = 4'b0;  
        wb_stb_o = 1'b0;  
        wb_cyc_o = 1'b0;  

        if (mmu_stb_i && mmu_cyc_i) begin  
            if (mmu_adr_i[31] != 1'b1) begin  
                // 地址 < 0x80000000，直接访问总线，绕过缓存  
                wb_adr_o = mmu_adr_i;  
                wb_dat_o = mmu_dat_i;  
                wb_we_o  = mmu_we_i;  
                wb_sel_o = mmu_sel_i;  
                wb_stb_o = 1;  
                wb_cyc_o = 1;  
                mmu_dat_o = wb_dat_i;  
                mmu_ack_o = wb_ack_i;
                mmu_err_o = wb_err_i;
                mmu_rty_o = wb_rty_i;
            end else if (!mmu_we_i) begin  
                // 读操作  
                if (hit0) begin  
                    // 命中 cache_mem0  
                    mmu_ack_o = 1;  
                    mmu_dat_o = cache_mem0[index].data;  
                end else if (hit1) begin  
                    // 命中 cache_mem1  
                    mmu_ack_o = 1;  
                    mmu_dat_o = cache_mem1[index].data;  
                end else begin  
                    // 未命中，发起总线读取  
                    wb_adr_o = mmu_adr_i;  
                    wb_we_o  = 0;  
                    wb_sel_o = mmu_sel_i;
                    wb_stb_o = 1;  
                    wb_cyc_o = 1;  
                    if (wb_ack_i | history_ack) begin  
                        // 总线返回数据，立即给出 mmu_ack_o 和 mmu_dat_o  
                        mmu_ack_o = 1;  
                        mmu_dat_o = wb_dat_i;  
                    end  
                end  
            end else begin  
                // 写操作  
                wb_adr_o = mmu_adr_i;  
                wb_dat_o = mmu_dat_i;  
                wb_we_o  = 1;  
                wb_sel_o = mmu_sel_i;
                wb_stb_o = 1;  
                wb_cyc_o = 1;  
                if (wb_ack_i) begin  
                    // 写操作完成  
                    mmu_ack_o = 1;  
                end  
            end  
        end
    end  

    // 时序逻辑  
    always_ff @(posedge clk or posedge rst) begin  
        if (rst) begin  
            // 复位缓存和 LRU  
            integer i;  
            for (i = 0; i < CACHE_LINES; i = i + 1) begin  
                cache_mem0[i].valid <= 1'b0;  
                cache_mem0[i].tag <= 0;
                cache_mem0[i].data <= 0;
                cache_mem1[i].valid <= 1'b0;
                cache_mem1[i].tag <= 0;
                cache_mem1[i].data <= 0;  
                lru[i] <= 1'b0;  
            end  
            history_ack <= 0;
        end else begin  
            if (mmu_stb_i && mmu_cyc_i && (mmu_adr_i[31] == 1'b1)) begin  
                if (!mmu_we_i) begin  
                    // 读操作  
                    if (!hit0 && !hit1 && wb_ack_i) begin
                        history_ack <= 1;
                        // 未命中且总线返回数据，更新缓存  
                        if (!cache_mem0[index].valid) begin  
                            // 填充 cache_mem0  
                            cache_mem0[index].valid <= 1'b1;  
                            cache_mem0[index].tag <= tag;  
                            cache_mem0[index].data <= wb_dat_i;  
                            lru[index] <= 1'b1; // 最近使用 cache_mem0  
                        end else if (!cache_mem1[index].valid) begin  
                            // 填充 cache_mem1  
                            cache_mem1[index].valid <= 1'b1;  
                            cache_mem1[index].tag <= tag;  
                            cache_mem1[index].data <= wb_dat_i;  
                            lru[index] <= 1'b0; // 最近使用 cache_mem1  
                        end else begin  
                            // 两路均有效，按照 LRU 替换  
                            if (lru[index] == 1'b0) begin  
                                // 替换 cache_mem0  
                                cache_mem0[index].tag <= tag;  
                                cache_mem0[index].data <= wb_dat_i;  
                                lru[index] <= 1'b1;  
                            end else begin  
                                // 替换 cache_mem1  
                                cache_mem1[index].tag <= tag;  
                                cache_mem1[index].data <= wb_dat_i;  
                                lru[index] <= 1'b0;  
                            end  
                        end  
                    end else if (hit0) begin  
                        // 命中 cache_mem0，更新 LRU  
                        lru[index] <= 1'b1;  
                    end else if (hit1) begin  
                        // 命中 cache_mem1，更新 LRU  
                        lru[index] <= 1'b0;  
                    end  
                end else begin  
                    // 写操作  
                    if (wb_ack_i) begin  
                        // 写操作完成后更新缓存  
                        if (hit0) begin  
                            // 更新 cache_mem0  
                            cache_mem0[index].data <= mmu_dat_i;  
                            lru[index] <= 1'b1;  
                        end else if (hit1) begin  
                            // 更新 cache_mem1  
                            cache_mem1[index].data <= mmu_dat_i;  
                            lru[index] <= 1'b0;  
                        end else begin  
                            // 未命中，按照 LRU 替换  
                            if (lru[index] == 1'b0) begin  
                                // 替换 cache_mem0  
                                cache_mem0[index].valid <= 1'b1;  
                                cache_mem0[index].tag <= tag;  
                                cache_mem0[index].data <= mmu_dat_i;  
                                lru[index] <= 1'b1;  
                            end else begin  
                                // 替换 cache_mem1  
                                cache_mem1[index].valid <= 1'b1;  
                                cache_mem1[index].tag <= tag;  
                                cache_mem1[index].data <= mmu_dat_i;  
                                lru[index] <= 1'b0;  
                            end  
                        end  
                    end  
                end  
            end  else begin
                history_ack <= 0;
            end
        end  
    end  

endmodule