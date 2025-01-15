module Regfile (  
    input wire clk,  
    input wire reset,  

    // WB�׶�д�˿�  
    input wire [4:0] wb_waddr,  
    input wire [31:0] wb_wdata,  
    input wire wb_we,  //дʹ��

    // ID�׶ζ��˿�A  
    input wire [4:0] id_raddr_a,  
    output wire [31:0] id_rdata_a,  

    // ID�׶ζ��˿�B  
    input wire [4:0] id_raddr_b,  
    output wire [31:0] id_rdata_b  
);  

    reg [31:0] regs[31:0];   

    // ���˿�������߼�����ǰ�ݻ���  
    // �������ַ��д��ַ��ͬ������д�룬ֱ�ӷ���д�������  
    assign id_rdata_a = (id_raddr_a == 5'b0) ? 32'b0 :  
                       (wb_we && (id_raddr_a == wb_waddr)) ? wb_wdata :  
                       regs[id_raddr_a];  

    assign id_rdata_b = (id_raddr_b == 5'b0) ? 32'b0 :  
                       (wb_we && (id_raddr_b == wb_waddr)) ? wb_wdata :  
                       regs[id_raddr_b];  

    // д�����   
    always_ff @(posedge clk) begin  
        if (reset) begin  
            integer i;  
            for (i = 0; i < 32; i = i + 1) begin  
                regs[i] <= 32'b0;  
            end  
        end  
        else if (wb_we && (wb_waddr != 5'b0)) begin  
            regs[wb_waddr] <= wb_wdata;  
        end  
    end  

endmodule