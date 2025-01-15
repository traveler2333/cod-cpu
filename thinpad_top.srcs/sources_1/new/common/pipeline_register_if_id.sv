module pipline_register_if_id(
    input wire clk,
    input wire rst,
    input wire flush_i,
    input wire stall_i,

    input wire [31:0] pc_i,
    input wire [31:0] instr_i,

    output reg [31:0] pc_o,
    output reg [31:0] instr_o,

    //exception_code和page_fault
    input reg [3:0] exception_code_i,
    input reg page_fault_i,
    output reg [3:0] exception_code_o,
    output reg page_fault_o

);
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pc_o <= 32'b0;
        instr_o <= 32'b0;
        exception_code_o <= 4'b0;
        page_fault_o <= 1'b0;
    end else if (stall_i) begin
        pc_o <= pc_o;
        instr_o <= instr_o;
        exception_code_o <= exception_code_o;
        page_fault_o <= page_fault_o;
    end else if (flush_i) begin
        pc_o <= 32'b0;
        instr_o <= 32'b0;
        exception_code_o <= 4'b0;
        page_fault_o <= 1'b0;
    end else begin
        pc_o <= pc_i;
        instr_o <= instr_i;
        exception_code_o <= exception_code_i;
        page_fault_o <= page_fault_i;
    end
end
endmodule