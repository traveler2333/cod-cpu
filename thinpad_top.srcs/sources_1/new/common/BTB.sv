module BTB #(
    parameter BTB_SIZE = 16 // BTB 表大小（例如 16 项，支持直接映射）
)(
    input wire clk,
    input wire reset,

    input wire BTB_stall,              // BTB 是否暂停 
    // 查询接口——pc
    input wire [31:0] pc,              // 当前 PC
    output reg [31:0] btb_target,      // BTB 提供的目标地址
    output reg btb_hit,                // BTB 是否命中

    // 查询接口——id
    input wire [31:0] pc_exe,           // ID 阶段的 PC
    output reg [31:0] btb_target_exe,   // BTB 提供的目标地址
    output reg btb_hit_exe,             // BTB 是否命中

    // 更新接口
    input wire update_en,              // 更新使能
    input wire [31:0] update_pc,       // 更新的 PC
    input wire [31:0] update_target,   // 更新的目标地址
    input wire update_valid            // 更新有效位
);
    // BTB 表项定义
    typedef struct packed {
        logic [31:0] tag;              // 分支指令地址
        logic [31:0] target;           // 分支目标地址
        logic valid;                   // 有效位
    } btb_entry_t;

    // BTB 表
    btb_entry_t btb_table[BTB_SIZE];

    // 索引计算
    logic [$clog2(BTB_SIZE)-1:0] index;
    logic [$clog2(BTB_SIZE)-1:0] index_exe;
    assign index = pc[$clog2(BTB_SIZE)+1:2]; // 假设 PC 对齐到 4 字节
    assign index_exe = pc_exe[$clog2(BTB_SIZE)+1:2]; // 假设 PC 对齐到 4 字节

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < BTB_SIZE; i++) begin
                btb_table[i].valid <= 0;
                btb_table[i].tag <= 0;
                btb_table[i].target <= 0;
            end
        end else if (update_en&&!BTB_stall) begin
            btb_table[update_pc[$clog2(BTB_SIZE)+1:2]].tag <= update_pc;
            btb_table[update_pc[$clog2(BTB_SIZE)+1:2]].target <= update_target;
            btb_table[update_pc[$clog2(BTB_SIZE)+1:2]].valid <= update_valid;
        end
    end

    always_comb begin
        if (btb_table[index].valid && btb_table[index].tag == pc) begin
            btb_target = btb_table[index].target;
            btb_hit = 1;
        end else begin
            btb_target = pc + 4;
            btb_hit = 0;
        end
        btb_hit = 0;
    end

    always_comb begin
        if (btb_table[index_exe].valid && btb_table[index_exe].tag == pc_exe) begin
            btb_target_exe = btb_table[index_exe].target;
            btb_hit_exe = 1;
        end else begin
            btb_target_exe = pc_exe + 4;
            btb_hit_exe = 0;
        end
        btb_hit_exe = 0;
    end
endmodule
