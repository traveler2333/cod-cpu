module pipline_register_mem_wb(
    input wire clk,
    input wire rst,

    input wire [31:0] rf_wdata_i,
    input wire [ 4:0] rf_addr_d_i,
    input wire rf_wen_i,

    //这几个输出都会分叉，需要仔细检查,这里直接连接到regfile的输入端口
    output reg [31:0] rf_wdata_o,
    output reg [ 4:0] rf_addr_d_o,
    output reg rf_wen_o,

    input wire stall,
    input wire nop
);
always @(posedge clk or posedge rst) begin
    if (rst) begin
        rf_wdata_o <= 32'b0;
        rf_addr_d_o <= 5'b0;
        rf_wen_o <= 1'b0;
    end else if (nop) begin
        rf_wdata_o <= 32'b0;
        rf_addr_d_o <= 5'b0;
        rf_wen_o <= 1'b0;
    end else if (~stall) begin
        rf_wdata_o <= rf_wdata_i;
        rf_addr_d_o <= rf_addr_d_i;
        rf_wen_o <= rf_wen_i;
    end
end
endmodule