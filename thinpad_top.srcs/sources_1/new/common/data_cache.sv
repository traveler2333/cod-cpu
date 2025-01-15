module data_cache (  
    input wire clk,  
    input wire rst,  

    // �� MMU �Ľӿڣ�������һ�£�  
    input wire [31:0] mmu_adr_i,    // ��ַ����  
    input wire [31:0] mmu_dat_i,    // д��������  
    output reg [31:0] mmu_dat_o,    // ���������  
    input wire mmu_we_i,            // дʹ������  
    input wire [3:0] mmu_sel_i,     // �ֽ�ѡ������  
    input wire mmu_stb_i,           // ѡͨ�ź�����  
    input wire mmu_cyc_i,           // �����ź�����  
    output reg mmu_ack_o,           // ȷ���ź����  
    output reg mmu_err_o,  
    output reg mmu_rty_o,           // �����ź����  

    // �� Wishbone ���ߵĽӿ�  
    output reg [31:0] wb_adr_o,    // ��ַ���  
    output reg [31:0] wb_dat_o,    // д�������������  
    input wire [31:0] wb_dat_i,     // �����߽��յ�����  
    output reg wb_we_o,            // дʹ�����������  
    output reg [3:0] wb_sel_o,     // �ֽ�ѡ�����������  
    output reg wb_stb_o,           // ѡͨ�ź����������  
    output reg wb_cyc_o,           // �����ź����������  
    input wire wb_ack_i,            // ����ȷ���ź�����  
    input wire wb_err_i,  
    input wire wb_rty_i             // ���ߴ����ź�����  
);  

    // �������  
    localparam CACHE_LINES = 32;          // ��������  
    localparam CACHE_WAYS = 2;            // 2 ·������  
    localparam CACHE_LINE_SIZE = 4;       // �����д�С  
    localparam INDEX_WIDTH = $clog2(CACHE_LINES);  
    localparam OFFSET_WIDTH = $clog2(CACHE_LINE_SIZE);  
    localparam TAG_WIDTH = 32 - INDEX_WIDTH - OFFSET_WIDTH;  

    // �����ж���  
    typedef struct packed {  
        logic valid;                      // ��Чλ  
        logic [TAG_WIDTH-1:0] tag;        // ��ַ��ǩ  
        logic [31:0] data;                // ����  
    } cache_line_t;  

    // ����洢��  
    cache_line_t cache_mem0[CACHE_LINES];  
    cache_line_t cache_mem1[CACHE_LINES];  

    // �滻����  
    logic lru[CACHE_LINES]; // 0: cache_mem0 ���ʹ�ã�1: cache_mem1 ���ʹ��  

    // ��ַ����  
    logic [TAG_WIDTH-1:0] tag = mmu_adr_i[31:INDEX_WIDTH + OFFSET_WIDTH];  
    logic [INDEX_WIDTH-1:0] index = mmu_adr_i[INDEX_WIDTH + OFFSET_WIDTH - 1:OFFSET_WIDTH];  
    logic [OFFSET_WIDTH-1:0] offset = mmu_adr_i[OFFSET_WIDTH - 1:0];  

    // �������б�־  
    logic hit0 = cache_mem0[index].valid && (cache_mem0[index].tag == tag);  
    logic hit1 = cache_mem1[index].valid && (cache_mem1[index].tag == tag);  
    logic history_ack = 0;
    // ����߼�����  
    always_comb begin  
        mmu_ack_o = 0;  
        mmu_dat_o = 0;  
        mmu_err_o = 0;  
        mmu_rty_o = 0;  

        // Ĭ�������źţ�������һ��״̬���������µ����߲���  
        wb_adr_o = 32'b0;  
        wb_dat_o = 32'b0;  
        wb_we_o  = 1'b0;  
        wb_sel_o = 4'b0;  
        wb_stb_o = 1'b0;  
        wb_cyc_o = 1'b0;  

        if (mmu_stb_i && mmu_cyc_i) begin  
            if (mmu_adr_i[31] != 1'b1) begin  
                // ��ַ < 0x80000000��ֱ�ӷ������ߣ��ƹ�����  
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
                // ������  
                if (hit0) begin  
                    // ���� cache_mem0  
                    mmu_ack_o = 1;  
                    mmu_dat_o = cache_mem0[index].data;  
                end else if (hit1) begin  
                    // ���� cache_mem1  
                    mmu_ack_o = 1;  
                    mmu_dat_o = cache_mem1[index].data;  
                end else begin  
                    // δ���У��������߶�ȡ  
                    wb_adr_o = mmu_adr_i;  
                    wb_we_o  = 0;  
                    wb_sel_o = mmu_sel_i;
                    wb_stb_o = 1;  
                    wb_cyc_o = 1;  
                    if (wb_ack_i | history_ack) begin  
                        // ���߷������ݣ��������� mmu_ack_o �� mmu_dat_o  
                        mmu_ack_o = 1;  
                        mmu_dat_o = wb_dat_i;  
                    end  
                end  
            end else begin  
                // д����  
                wb_adr_o = mmu_adr_i;  
                wb_dat_o = mmu_dat_i;  
                wb_we_o  = 1;  
                wb_sel_o = mmu_sel_i;
                wb_stb_o = 1;  
                wb_cyc_o = 1;  
                if (wb_ack_i) begin  
                    // д�������  
                    mmu_ack_o = 1;  
                end  
            end  
        end
    end  

    // ʱ���߼�  
    always_ff @(posedge clk or posedge rst) begin  
        if (rst) begin  
            // ��λ����� LRU  
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
                    // ������  
                    if (!hit0 && !hit1 && wb_ack_i) begin
                        history_ack <= 1;
                        // δ���������߷������ݣ����»���  
                        if (!cache_mem0[index].valid) begin  
                            // ��� cache_mem0  
                            cache_mem0[index].valid <= 1'b1;  
                            cache_mem0[index].tag <= tag;  
                            cache_mem0[index].data <= wb_dat_i;  
                            lru[index] <= 1'b1; // ���ʹ�� cache_mem0  
                        end else if (!cache_mem1[index].valid) begin  
                            // ��� cache_mem1  
                            cache_mem1[index].valid <= 1'b1;  
                            cache_mem1[index].tag <= tag;  
                            cache_mem1[index].data <= wb_dat_i;  
                            lru[index] <= 1'b0; // ���ʹ�� cache_mem1  
                        end else begin  
                            // ��·����Ч������ LRU �滻  
                            if (lru[index] == 1'b0) begin  
                                // �滻 cache_mem0  
                                cache_mem0[index].tag <= tag;  
                                cache_mem0[index].data <= wb_dat_i;  
                                lru[index] <= 1'b1;  
                            end else begin  
                                // �滻 cache_mem1  
                                cache_mem1[index].tag <= tag;  
                                cache_mem1[index].data <= wb_dat_i;  
                                lru[index] <= 1'b0;  
                            end  
                        end  
                    end else if (hit0) begin  
                        // ���� cache_mem0������ LRU  
                        lru[index] <= 1'b1;  
                    end else if (hit1) begin  
                        // ���� cache_mem1������ LRU  
                        lru[index] <= 1'b0;  
                    end  
                end else begin  
                    // д����  
                    if (wb_ack_i) begin  
                        // д������ɺ���»���  
                        if (hit0) begin  
                            // ���� cache_mem0  
                            cache_mem0[index].data <= mmu_dat_i;  
                            lru[index] <= 1'b1;  
                        end else if (hit1) begin  
                            // ���� cache_mem1  
                            cache_mem1[index].data <= mmu_dat_i;  
                            lru[index] <= 1'b0;  
                        end else begin  
                            // δ���У����� LRU �滻  
                            if (lru[index] == 1'b0) begin  
                                // �滻 cache_mem0  
                                cache_mem0[index].valid <= 1'b1;  
                                cache_mem0[index].tag <= tag;  
                                cache_mem0[index].data <= mmu_dat_i;  
                                lru[index] <= 1'b1;  
                            end else begin  
                                // �滻 cache_mem1  
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