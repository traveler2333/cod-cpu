module InstructionCache (
    input wire clk,
    input wire rst,
    input wire [31:0] addr,          // if ģ�鴫���ĵ�ַ
    input wire read_en,              // ��ȡʹ��
    output reg [31:0] data,          // �������
    output reg hit,                  // �������б�־

    // �� IF ģ�齻��
    input wire [31:0] if_mem_data,   // �� IF ģ���ȡ������
    input wire if_mem_ready,          // д��ʹ��

    input wire fence_i
);

localparam CACHE_SIZE = 32;         // ÿ·�����С
localparam INDEX_BITS = 5;           // ����λ��
localparam TAG_BITS = 17;            // ��ǩλ��

typedef struct packed {
    logic valid;
    logic [TAG_BITS-1:0] tag;
    logic [31:0] data;
} cache_line_t;

// ����洢��
cache_line_t cache_mem0[CACHE_SIZE];
cache_line_t cache_mem1[CACHE_SIZE];

// �����ͱ�ǩ
logic [INDEX_BITS-1:0] index;
logic [TAG_BITS-1:0] tag;

// ������һ��Ӧ�ñ��滻
logic lru[CACHE_SIZE];

always_comb begin
    hit = 0;
    data = 32'b0;

    index = addr[INDEX_BITS+1:2];
    tag = addr[23:INDEX_BITS+2];
    
    // ��黺������
    if (cache_mem0[index].valid && cache_mem0[index].tag == tag) begin
        hit = 1;
        data = cache_mem0[index].data;
    end else if (cache_mem1[index].valid && cache_mem1[index].tag == tag) begin
        hit = 1;
        data = cache_mem1[index].data;
    end
end

// ��������߼��ѽ���
always_ff @(posedge clk) begin
    if (rst) begin
        for (int i = 0; i < CACHE_SIZE; i++) begin
            cache_mem0[i] <= 0;  
            cache_mem1[i] <= 0;  
            lru[i] <= 0;  
        end
    end else begin
        if(fence_i) begin
        //��ջ���
            for(int i = 0; i < CACHE_SIZE; i++) begin
                cache_mem0[i].valid = 0;
                cache_mem1[i].valid = 0;
            end
        end
        if (if_mem_ready && !hit) begin
            if (lru[index] == 0) begin
                cache_mem0[index].valid <= 1;
                cache_mem0[index].tag <= tag;
                cache_mem0[index].data <= if_mem_data;
                lru[index] <= 1; // ���� LRU λ
            end else begin
                cache_mem1[index].valid <= 1;
                cache_mem1[index].tag <= tag;
                cache_mem1[index].data <= if_mem_data;
                lru[index] <= 0; // ���� LRU λ
            end
        end else if (hit) begin
            if (cache_mem0[index].valid && cache_mem0[index].tag == tag) begin
                lru[index] <= 1;
            end else if (cache_mem1[index].valid && cache_mem1[index].tag == tag) begin
                lru[index] <= 0;
            end
        end
    end
end
endmodule