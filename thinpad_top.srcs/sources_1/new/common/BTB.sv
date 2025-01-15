module BTB #(
    parameter BTB_SIZE = 16 // BTB ���С������ 16 �֧��ֱ��ӳ�䣩
)(
    input wire clk,
    input wire reset,

    input wire BTB_stall,              // BTB �Ƿ���ͣ 
    // ��ѯ�ӿڡ���pc
    input wire [31:0] pc,              // ��ǰ PC
    output reg [31:0] btb_target,      // BTB �ṩ��Ŀ���ַ
    output reg btb_hit,                // BTB �Ƿ�����

    // ��ѯ�ӿڡ���id
    input wire [31:0] pc_exe,           // ID �׶ε� PC
    output reg [31:0] btb_target_exe,   // BTB �ṩ��Ŀ���ַ
    output reg btb_hit_exe,             // BTB �Ƿ�����

    // ���½ӿ�
    input wire update_en,              // ����ʹ��
    input wire [31:0] update_pc,       // ���µ� PC
    input wire [31:0] update_target,   // ���µ�Ŀ���ַ
    input wire update_valid            // ������Чλ
);
    // BTB �����
    typedef struct packed {
        logic [31:0] tag;              // ��ָ֧���ַ
        logic [31:0] target;           // ��֧Ŀ���ַ
        logic valid;                   // ��Чλ
    } btb_entry_t;

    // BTB ��
    btb_entry_t btb_table[BTB_SIZE];

    // ��������
    logic [$clog2(BTB_SIZE)-1:0] index;
    logic [$clog2(BTB_SIZE)-1:0] index_exe;
    assign index = pc[$clog2(BTB_SIZE)+1:2]; // ���� PC ���뵽 4 �ֽ�
    assign index_exe = pc_exe[$clog2(BTB_SIZE)+1:2]; // ���� PC ���뵽 4 �ֽ�

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
