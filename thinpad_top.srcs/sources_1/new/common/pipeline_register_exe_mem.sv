module pipline_register_exe_mem(
    input wire clk,
    input wire rst,
    input wire stall,
    input wire nop,

    // 地址相关信号：
    input wire [31:0] addr_i,
    output reg [31:0] addr_o,

    // 数据相关信号：
    input wire [31:0] data_i,
    output reg [31:0] data_o,

    // 数据相关信号,直接从id/exe寄存器传递过来，同样直接传递给mem/wb寄存器
    input wire [4:0] rf_raddr_d_i,
    output reg [4:0] rf_raddr_d_o, //需要分叉到exe板块，判断前传

    input wire [4:0] inst_type_i,
    output reg [4:0] inst_type_o,

    // csr信号
    input wire [11:0] csr_addr_i,
    output reg [11:0] csr_addr_o,

    input wire [31:0] pc_i,
    output reg [31:0] pc_o,

    //控制信号：
    input wire mem_read_i,
    input wire mem_we_i,
    input wire [1:0] mem_size_i,
    input wire rf_wen_i,

    output reg mem_read_o, //需要分叉到exe板块，判断前传
    output reg mem_we_o,
    output reg [1:0] mem_size_o,
    output reg rf_wen_o,

    //exception_code和page_fault
    input wire [3:0] exception_code_i,
    input wire page_fault_i,
    output reg [3:0] exception_code_o,
    output reg page_fault_o
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        addr_o <= 32'b0;
        data_o <= 32'b0;
        rf_raddr_d_o <= 5'b0;
        inst_type_o <= 5'b0;
        csr_addr_o <= 12'b0;
        pc_o <= 32'b0;
        mem_read_o <= 1'b0;
        mem_we_o <= 1'b0;
        mem_size_o <= 2'b0;
        rf_wen_o <= 1'b0;
        exception_code_o <= 4'b0;
        page_fault_o <= 1'b0;
    end else if (nop) begin
        addr_o <= 32'b0;
        data_o <= 32'b0;
        rf_raddr_d_o <= 5'b0;
        inst_type_o <= 5'b0;
        csr_addr_o <= 12'b0;
        pc_o <= 32'b0;
        mem_read_o <= 1'b0;
        mem_we_o <= 1'b0;
        mem_size_o <= 2'b0;
        rf_wen_o <= 1'b0;
        exception_code_o <= 4'b0;
        page_fault_o <= 1'b0;
    end else if (!stall) begin
        addr_o <= addr_i;
        data_o <= data_i;
        rf_raddr_d_o <= rf_raddr_d_i;
        inst_type_o <= inst_type_i;
        csr_addr_o <= csr_addr_i;
        pc_o <= pc_i;
        mem_read_o <= mem_read_i;
        mem_we_o <= mem_we_i;
        mem_size_o <= mem_size_i;
        rf_wen_o <= rf_wen_i;
        exception_code_o <= exception_code_i;
        page_fault_o <= page_fault_i;
    end
end
endmodule