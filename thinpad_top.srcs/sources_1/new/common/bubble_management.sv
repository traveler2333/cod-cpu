module Hazard(  
    input wire clk,  
    input wire rst,  
    //-----------����ṹ��ͻ���ź�-----------
    // IF ģ��Ŀ����ź�
    input wire if_stall, // �Ƿ���ͣIFģ��

    // MEM ģ��Ŀ����ź�
    input wire mem_stall, // �Ƿ���ͣMEMģ��

    //-----------����ṹ��ͻ���ź�-----------
    //-----------���load-use��ͻ���ź�-----------
    // ID-EXE �׶��ź�  
    input wire [4:0] exe_r_a,  
    input wire [4:0] exe_r_b,

    // EXE-MEM �׶��ź�  
    input wire [4:0] mem_rd,  
    input wire mem_read,
    //-----------���load-use��ͻ���ź�-----------

    //-----------���branch��ͻ���ź�-----------
    // ID-EXE �׶��ź�
    input wire branch_en_i, // ��֧�Ƿ񱻲��ã���������ź�ͬʱҲ������PC_MUXģ��
    output reg branch_en_o, // ��֧�Ƿ񱻲��ã���������ź�ͬʱҲ������PC_MUXģ��

    // MEM �������쳣��ת�ź�
    input wire mem_branch_en_i,
    input wire [31:0] mem_branch_target_i,
    output reg mem_branch_en_o,
    output reg [31:0] mem_branch_target_o,
    //-----------���branch��ͻ���ź�-----------

    //-----------���stall���ź�-----------
    // PC_MUX ģ��Ŀ����ź�
    output reg pc_stall, // pcʹ��

    // IF/ID ��ˮ�߼Ĵ�������  
    output reg if_id_stall,  
    output reg if_id_flush, // ���ȷ����ת����ô���IF/ID�Ĵ��� 

    // ID/EX ��ˮ�߼Ĵ�������   
    output reg id_exe_stall,  
    output reg id_exe_nop, // ��������ID/EX��������п����ź�Ϊ0 

    // EX/MEM ��ˮ�߼Ĵ�������  
    output reg exe_mem_stall,  
    output reg exe_mem_nop, // ������load-use��ͻʱ����harzard�жϲ��ò���ͣ��ˮ�ߣ���ô����EX/MEM��������п����ź�Ϊ0
    // MEM/WB ��ˮ�߼Ĵ�������
    output reg mem_wb_nop, // MEM/WB��ˮ�߼Ĵ�������
    output reg mem_wb_stall // MEM/WB��ˮ�߼Ĵ�������
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
    // Ĭ�ϲ�����stall
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
    // �ж��Ƿ�Ϊloadָ��
    // �ж�д��Ĵ�����Դ�Ĵ����Ƿ���ͬ
    if ((exe_r_a == mem_rd ||exe_r_b == mem_rd) && mem_read) begin
        branch_en_o = 0; //��Ч��֧
        // ����stall
        pc_stall = 1;
        if_id_stall = 1;
        if_id_flush = 0;
        id_exe_stall = 1;
        if (!mem_stall) begin
            exe_mem_nop = 1;
        end
    end

    if (mem_stall) begin
        // �� mem_stall Ϊ 1����ͣ��ˮ��
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
            // �� if_stall Ϊ 1����ͣ PC
            pc_stall = 1;
            if_id_flush = 1;
        end
        if (branch_en_o | branch_o_tag) begin
            // ��������֧��ת�� mem_stall Ϊ 0��ˢ�� IF/ID �Ĵ�������������
            if_id_flush = 1;
            id_exe_nop = 1;
        end
    end

    if (mem_branch_en_i | error_branch_tag) begin
        // �������쳣��ת��ˢ�� IF/ID/EXE/MEM �Ĵ�������������
        if_id_flush = 1;
        id_exe_nop = 1;
        exe_mem_nop = 1;
    end
end

assign mem_branch_en_o = mem_branch_en_i | error_branch_tag;
assign mem_branch_target_o = (error_branch_tag) ? error_branch_target : mem_branch_target_i;

endmodule