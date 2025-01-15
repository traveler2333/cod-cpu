module MMU(
    input wire clk,
    input wire rst,

    // 控制信号，mode
    input wire [1:0] mode, 

    // 页表基地址
    input wire [31:0] page_table_base,

    // if或者mem传来的信号
    input wire [31:0] virtual_addr,
    input wire [31:0] if_mem_wb_dat_i,
    input wire if_mem_wb_we_i,
    input wire [3:0] if_mem_wb_sel_i,
    input wire if_mem_wb_stb_i,
    input wire if_mem_wb_cyc_i,

    // wishbone传来的信号给if或者mem
    output reg [31:0] if_mem_wb_dat_o,
    output reg if_mem_wb_ack_o,
    output reg [3:0] exception_code,
    output reg page_fault,  // 缺页异常信号

    // Wishbone 接口
    output reg [31:0] wb_adr_o,
    output reg [31:0] wb_dat_o,
    input wire [31:0] wb_dat_i,
    output reg wb_we_o,
    output reg [3:0] wb_sel_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    input wire wb_err_i,
    input wire wb_rty_i,
    output reg wb_cyc_o,

    // 控制是mem还是if模块查询页表
    input wire mem_if_sel,

    output reg [31:0] tlb_addr,
    output reg tlb_read_en,
    input wire [31:0] tlb_data,
    input wire tlb_hit,
    //传给cache的数据用来更新
    output reg [31:0] tlb_data_update,
    output reg tlb_write_en
);

typedef enum logic [2:0] {
    IDLE,
    TRANSLATE,
    WAIT_ACK,
    WAIT_MEM_ACK,
    STB_DOWN
} state_t;

state_t state, next_state;
reg [31:0] pte;
reg [1:0] level;

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= IDLE;
        level <= 2;
        pte <= 32'b0;
    end else begin
        state <= next_state;
        if(state == IDLE) begin
            pte <= 32'b0;
            level <= 2;
        end
        else if(state == TRANSLATE) begin
            level <= level - 1;
            if(tlb_hit) begin
                pte <= tlb_data;
            end
        end
        if (state == WAIT_ACK && wb_ack_i) begin
            pte <= wb_dat_i;
        end
    end
end

always_comb begin
    next_state = state;
    page_fault = 1'b0;
    wb_cyc_o = 1'b0;
    wb_stb_o = 1'b0;
    wb_we_o = 1'b0;
    wb_sel_o = 4'b1111;
    wb_adr_o = 32'b0;
    wb_dat_o = 32'b0;
    if_mem_wb_ack_o = 1'b0;
    if_mem_wb_dat_o = 32'b0;
    exception_code = 4'b0;
    tlb_addr = 0;
    tlb_data_update = 0;
    tlb_write_en = 0;
    tlb_read_en = 1'b0;

    case (state)
        IDLE: begin
            if (if_mem_wb_stb_i && if_mem_wb_cyc_i) begin
                if (mode == 2'b11) begin 
                    wb_cyc_o = 1'b1;
                    wb_stb_o = 1'b1;
                    wb_adr_o = virtual_addr;
                    wb_dat_o = if_mem_wb_dat_i;
                    wb_we_o = if_mem_wb_we_i;
                    wb_sel_o = if_mem_wb_sel_i;
                    if_mem_wb_dat_o = wb_dat_i;
                    if_mem_wb_ack_o = wb_ack_i;
                    next_state = IDLE;
                end else begin
                    if((virtual_addr >= 32'h00000000 && virtual_addr <= 32'h002FFFFF) || (virtual_addr >= 32'h7FC10000 && virtual_addr <= 32'h7FFFFFFF))begin 
                    next_state = TRANSLATE;
                    end else begin
                        wb_cyc_o = 1'b1;
                        wb_stb_o = 1'b1;
                        wb_adr_o = virtual_addr;
                        wb_dat_o = if_mem_wb_dat_i;
                        wb_we_o = if_mem_wb_we_i;
                        wb_sel_o = if_mem_wb_sel_i;
                        if_mem_wb_dat_o = wb_dat_i;
                        if_mem_wb_ack_o = wb_ack_i;
                        next_state = IDLE;
                    end
                end
            end
        end
        TRANSLATE: begin
            wb_cyc_o = 1'b1;
            wb_stb_o = 1'b1;
            if (level == 2) begin
                wb_adr_o = {page_table_base[21:0], virtual_addr[31:22]} << 2;  // 一级页表项地址
                tlb_addr = {page_table_base[21:0], virtual_addr[31:22]};
                tlb_read_en = 1'b1;
                next_state = WAIT_ACK;
                if (tlb_hit) begin
                    next_state = TRANSLATE;
                end
            end else begin
                wb_adr_o = {pte[31:10], virtual_addr[21:12]} << 2;  // 二级页表项地址
                wb_cyc_o = 1'b0;
                wb_stb_o = 1'b0;
                tlb_addr = {pte[31:10], virtual_addr[21:12]};
                tlb_read_en = 1'b1;
                next_state = WAIT_ACK;
                if (tlb_hit) begin
                    next_state = WAIT_MEM_ACK;
                end
            end
        end
        WAIT_ACK: begin
            wb_cyc_o = 1'b1;
            wb_stb_o = 1'b1;
            if(level == 1) begin
                wb_adr_o = {page_table_base[21:0], virtual_addr[31:22]} << 2;  // 一级页表项地址
                tlb_addr = {page_table_base[21:0], virtual_addr[31:22]};
            end else begin
                wb_adr_o = {pte[31:10], virtual_addr[21:12]} << 2;  // 二级页表项地址
                tlb_addr = {pte[31:10], virtual_addr[21:12]};
            end
            if (wb_ack_i) begin
                if (wb_dat_i[0] == 1'b0) begin
                    page_fault = 1'b1;
                    // 指令页面错误异常 (EX_INST_PAGE_FAULT = 12)
                    exception_code = 4'b1100;
                    next_state = IDLE;
                end else if (wb_dat_i[1] == 1'b1 || wb_dat_i[3] == 1'b1) begin  // 检查是否为叶子节点
                    if (mem_if_sel==1'b0 && wb_dat_i[3] == 1'b0) begin
                        page_fault = 1'b1;
                        //指令页面错误异常 (EX_INST_PAGE_FAULT = 12)：
                        exception_code = 4'b1100;
                        next_state = IDLE;
                    end 
                    else if (mem_if_sel==1'b1 && wb_dat_i[1] == 1'b0) begin
                        page_fault = 1'b1;
                        //数据页面错误异常 (EX_DATA_PAGE_FAULT = 13)：
                        exception_code = 4'b1101;
                        next_state = IDLE;
                    end
                    else if (mem_if_sel==1'b1 && wb_dat_i[2] == 1'b0) begin
                        page_fault = 1'b1;
                        //存储页面错误异常 EX_STORE_PAGE_FAULT = 15)：
                        exception_code = 4'b1111;
                        next_state = IDLE;
                    end else begin
                        tlb_write_en = 1'b1;
                        tlb_data_update = wb_dat_i;
                        next_state = STB_DOWN;
                    end
                end else begin
                    if (level < 0) begin
                        page_fault = 1'b1;
                        exception_code = 4'b1100;  
                        next_state = IDLE;
                    end else begin
                        tlb_write_en = 1'b1;
                        tlb_data_update = wb_dat_i;
                        next_state = TRANSLATE;
                    end
                end
            end
        end
        WAIT_MEM_ACK: begin
            wb_cyc_o = 1'b1;
            wb_stb_o = 1'b1;
            if(pte==32'b0) begin
                wb_adr_o = virtual_addr;
            end else begin
                wb_adr_o = (pte[31:10] << 12) + virtual_addr[11:0];
            end
            wb_dat_o = if_mem_wb_dat_i;
            wb_we_o = if_mem_wb_we_i;
            wb_sel_o = if_mem_wb_sel_i;
            if (wb_ack_i) begin
                if_mem_wb_ack_o = 1'b1;
                if_mem_wb_dat_o = wb_dat_i;
                next_state = IDLE;
            end
        end
        STB_DOWN: begin
            wb_cyc_o = 1'b0;
            wb_stb_o = 1'b0;
            next_state = WAIT_MEM_ACK;
        end
    endcase
end

endmodule