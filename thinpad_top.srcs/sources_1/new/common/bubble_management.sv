module Hazard(  
    input wire clk,  
    input wire rst,  
    //-----------解决结构冲突的信号-----------
    // IF 模块的控制信号
    input wire if_stall, // 是否暂停IF模块

    // MEM 模块的控制信号
    input wire mem_stall, // 是否暂停MEM模块

    //-----------解决结构冲突的信号-----------
    //-----------解决load-use冲突的信号-----------
    // ID-EXE 阶段信号  
    input wire [4:0] exe_r_a,  
    input wire [4:0] exe_r_b,

    // EXE-MEM 阶段信号  
    input wire [4:0] mem_rd,  
    input wire mem_read,
    //-----------解决load-use冲突的信号-----------

    //-----------解决branch冲突的信号-----------
    // ID-EXE 阶段信号
    input wire branch_en_i, // 分支是否被采用，这个控制信号同时也被传向PC_MUX模块
    output reg branch_en_o, // 分支是否被采用，这个控制信号同时也被传向PC_MUX模块

    // MEM 发出的异常跳转信号
    input wire mem_branch_en_i,
    input wire [31:0] mem_branch_target_i,
    output reg mem_branch_en_o,
    output reg [31:0] mem_branch_target_o,
    //-----------解决branch冲突的信号-----------

    //-----------解决stall的信号-----------
    // PC_MUX 模块的控制信号
    output reg pc_stall, // pc使能

    // IF/ID 流水线寄存器控制  
    output reg if_id_stall,  
    output reg if_id_flush, // 如果确认跳转，那么清空IF/ID寄存器 

    // ID/EX 流水线寄存器控制   
    output reg id_exe_stall,  
    output reg id_exe_nop, // 这是设置ID/EX保存的所有控制信号为0 

    // EX/MEM 流水线寄存器控制  
    output reg exe_mem_stall,  
    output reg exe_mem_nop, // 当产生load-use冲突时，若harzard判断不得不暂停流水线，那么设置EX/MEM保存的所有控制信号为0
    // MEM/WB 流水线寄存器控制
    output reg mem_wb_nop, // MEM/WB流水线寄存器控制
    output reg mem_wb_stall // MEM/WB流水线寄存器控制
);  
logic branch_o_tag = 0;
logic error_branch_tag = 0;
logic [31:0] error_branch_target;
always_ff  @(posedge clk) begin
    if(rst) begin
        branch_o_tag <= 0;
        error_branch_tag <= 0;
        error_branch_target <= 0;
    end else begin
        if(branch_en_o & pc_stall) begin
            branch_o_tag <= 1;
        end
        if(branch_o_tag & !pc_stall) begin
            branch_o_tag <= 0;
        end

        if(mem_branch_en_o & pc_stall) begin
            if (~error_branch_tag) begin
                error_branch_target <= mem_branch_target_i;
            end
            error_branch_tag <= 1;
        end
        if(error_branch_tag & !pc_stall) begin
            error_branch_tag <= 0;
            error_branch_target <= 0;
        end
    end
end

always_comb begin
    // 默认不进行stall
    pc_stall = 0;
    if_id_stall = 0;
    if_id_flush = 0;
    id_exe_stall = 0;
    id_exe_nop = 0;
    exe_mem_stall = 0;
    exe_mem_nop = 0;
    mem_wb_nop = 0;
    mem_wb_stall = 0;
    
    branch_en_o = branch_en_i;
    // 判断是否为load指令
    // 判断写入寄存器和源寄存器是否相同
    if ((exe_r_a == mem_rd ||exe_r_b == mem_rd) && mem_read) begin
        branch_en_o = 0; //无效分支
        // 进行stall
        pc_stall = 1;
        if_id_stall = 1;
        if_id_flush = 0;
        id_exe_stall = 1;
        if (!mem_stall) begin
            exe_mem_nop = 1;
        end
    end

    if (mem_stall) begin
        // 当 mem_stall 为 1，暂停流水线
        pc_stall = 1;
        if_id_stall = 1;
        if_id_flush = 0;
        id_exe_stall = 1;
        id_exe_nop = 0;
        exe_mem_stall = 1;
        exe_mem_nop = 0;
        // mem_wb_nop = 1;
        mem_wb_stall = 1;
    end else begin
        if (if_stall) begin
            // 当 if_stall 为 1，暂停 PC
            pc_stall = 1;
            if_id_flush = 1;
        end
        if (branch_en_o | branch_o_tag) begin
            // 当发生分支跳转且 mem_stall 为 0，刷新 IF/ID 寄存器，插入气泡
            if_id_flush = 1;
            id_exe_nop = 1;
        end
    end

    if (mem_branch_en_i | error_branch_tag) begin
        // 当发生异常跳转，刷新 IF/ID/EXE/MEM 寄存器，插入气泡
        if_id_flush = 1;
        id_exe_nop = 1;
        exe_mem_nop = 1;
    end
end

assign mem_branch_en_o = mem_branch_en_i | error_branch_tag;
assign mem_branch_target_o = (error_branch_tag) ? error_branch_target : mem_branch_target_i;

endmodule