module PC_MUX(  
    input wire clk,                    // 系统时钟信号  
    input wire reset,                  // 异步复位信号  
    input wire branch_en,              // beq结果：分支是否被采用  
    input wire [31:0] branch_addr,     // 分支目标地址

    // 来自 BTB 的信号
    input wire [31:0] btb_target,      // BTB 提供的目标地址
    input wire btb_hit,                // BTB 是否命中

    // 来自hazard的异常跳转信号
    input wire error_branch_en,        // 异常跳转信号
    input wire [31:0] error_branch_target,// 异常跳转地址

    input wire stall,                  // 流水线暂停信号
    output reg [31:0] pc               // 当前的程序计数器值  
);  
//内部计算pc+4, 按照分支预测结果选择pc+4或者分支目标地址

logic [31:0] next_pc;
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        pc <= 32'h8000_0000;
        next_pc <= 32'h8000_0004;
    end else begin
        if (error_branch_en) begin
            next_pc <= error_branch_target;
        end else if (branch_en) begin
            next_pc <= branch_addr;
        end else if (btb_hit) begin
            next_pc <= btb_target;
        end

        if (stall) begin
            pc <= pc;
        end else if (error_branch_en) begin
            pc <= error_branch_target;
            next_pc <= error_branch_target+4;
        end else if(branch_en)begin
            pc <= branch_addr;
            next_pc <= branch_addr+4;
        end else if(btb_hit) begin
            pc <= btb_target;
            next_pc <= btb_target+4;
        end else begin
            pc <= next_pc;
            next_pc <= next_pc+4;
        end
    end
end

endmodule