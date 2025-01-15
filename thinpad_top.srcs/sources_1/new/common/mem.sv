module MEM(
    input wire clk,
    input wire rst,
    
    // 对接 Wishbone 仲裁器
    output reg [31:0] wb_adr_o,       // 地址输出   
    output reg [31:0] wb_dat_o,       // 数据输出（占位）
    output reg wb_we_o,               // 写使能(取指阶段为零）   
    output reg [3:0] wb_sel_o,        // 字节选择信号  
    output reg wb_stb_o,              // 访问有效 
    output reg wb_cyc_o,              // 总线周期信号  

    input wire [31:0] wb_dat_i,       // 数据输入 
    input wire wb_ack_i,              // 应答信号  
    input wire [3:0] mem_exception_code,  // 异常代码
    input wire mem_page_fault,            // 缺页异常信号 
    input wire [3:0] if_exception_code,   // 异常代码
    input wire if_page_fault,             // 缺页异常信号
    
    //对接寄存器exe_mem
    input wire [31:0] exe_wb_adr_o,
    input wire [31:0] exe_wb_dat_o,
    input wire [4:0] inst_type_i,
    input wire [11:0] csr_addr_i,
    input wire [31:0] pc_i,
    input wire rf_wen_i,

    //控制信号：
    input wire mem_read,
    input wire mem_we,
    input wire [1:0] mem_size,
    
    //对接寄存器mem_wb,读的时候才有意义
    output reg [31:0] rf_wdata,
    output reg rf_wen_o,

    //结构冲突信号
    output reg mem_stall,

    //对接CSR寄存器
    output reg [4:0] csr_inst_type_o,
    output reg [11:0] csr_addr_o,
    output reg [31:0] csr_data_o,
    input wire [31:0] csr_data_i,

    //中断异常
    output reg branch_en,
    output reg [31:0] next_pc,

    //时钟中断信号
    input wire time_interrupt,
    output reg timeout,

    //页表异常
    output reg page_fault,

    //告诉mmu模块是mem
    output reg mem_mmu_sel,

    
    output reg fence_i,//是否是fence指令
    output reg sfence_vma, //是否是sfence.vma指令
    input wire dcache_fence_ack_i
);
typedef enum logic [4:0] {   
    ADD   = 0,  
    ADDI  = 1,   
    AND   = 2,  
    ANDI  = 3,  
    AUIPC = 4,  
    BEQ   = 5,  
    BNE   = 6,   
    JAL   = 7,  
    JALR  = 8,  
    LB    = 9,  
    LUI   = 10,  
    LW    = 11,  
    OR    = 12,  
    ORI   = 13,  
    SB    = 14,  
    SLLI  = 15,  
    SRLI  = 16,  
    SW    = 17,  
    XOR   = 18,  
    MINU  = 19,  
    PACK  = 20,  
    SBSET = 21,
    CSRRC = 22,
    CSRRS = 23,
    CSRRW = 24,
    EBREAK= 25,
    ECALL = 26,
    MRET  = 27,
    SLTU  = 28,
    NOP   = 29,
    FENCE = 30,
    SFENCE_VMA = 31
} instr_code;

localparam MTIME_ADDR = 32'h200bff8;
localparam MTIMECMP_ADDR = 32'h2004000;

logic [63:0] mtime;
logic [63:0] mtimecmp;

logic rw_mtime, rw_mtimecmp, csr_raw;

assign rw_mtime = (exe_wb_adr_o >= MTIME_ADDR) & (exe_wb_adr_o < MTIME_ADDR + 8) & (mem_read | mem_we);           // 判断是否访问mtime
assign rw_mtimecmp = (exe_wb_adr_o >= MTIMECMP_ADDR) & (exe_wb_adr_o < MTIMECMP_ADDR + 8) & (mem_read | mem_we);  // 判断是否访问mtimecmp

assign timeout = mtime >= mtimecmp;

assign page_fault = mem_page_fault | if_page_fault;

always_ff @(posedge clk) begin
    if(rst) begin
        wb_cyc_o <= 1'b0;
        wb_stb_o <= 1'b0;
        mtime <= 64'b0;
        mtimecmp <= 64'b0;
    end else if (time_interrupt | page_fault) begin
        mtime <= mtime + 1;
    end else begin
        mtime <= mtime + 1;
        if (!(rw_mtime | rw_mtimecmp | csr_raw)) begin
            if (wb_ack_i) begin
                wb_cyc_o <= 1'b0;
                wb_stb_o <= 1'b0;
            end else begin
                if (!mem_read & !mem_we) begin
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                end else begin
                    wb_cyc_o <= 1'b1;
                    wb_stb_o <= 1'b1;
                end
            end
        end
        else if (rw_mtime) begin    // 访问mtime
            if (mem_we) begin
                case (mem_size) 
                    2'b00: begin
                        case (exe_wb_adr_o[2:0])
                            3'b000: mtime[7:0] <= exe_wb_dat_o[7:0]; // SB
                            3'b001: mtime[15:8] <= exe_wb_dat_o[7:0]; // SB
                            3'b010: mtime[23:16] <= exe_wb_dat_o[7:0]; // SB
                            3'b011: mtime[31:24] <= exe_wb_dat_o[7:0]; // SB
                            3'b100: mtime[39:32] <= exe_wb_dat_o[7:0]; // SB
                            3'b101: mtime[47:40] <= exe_wb_dat_o[7:0]; // SB
                            3'b110: mtime[55:48] <= exe_wb_dat_o[7:0]; // SB
                            3'b111: mtime[63:56] <= exe_wb_dat_o[7:0]; // SB
                        endcase
                    end
                    2'b10: begin
                        case (exe_wb_adr_o[2:0])
                            3'b000: mtime[31:0] <= exe_wb_dat_o; // SW
                            3'b100: mtime[63:32] <= exe_wb_dat_o; // SW
                        endcase
                    end
                endcase
            end
        end else if (rw_mtimecmp) begin   // 访问mtimecmp
            if (mem_we) begin
                case (mem_size) 
                    2'b00: begin
                        case (exe_wb_adr_o[2:0])
                            3'b000: mtimecmp[7:0] <= exe_wb_dat_o[7:0]; // SB
                            3'b001: mtimecmp[15:8] <= exe_wb_dat_o[7:0]; // SB
                            3'b010: mtimecmp[23:16] <= exe_wb_dat_o[7:0]; // SB
                            3'b011: mtimecmp[31:24] <= exe_wb_dat_o[7:0]; // SB
                            3'b100: mtimecmp[39:32] <= exe_wb_dat_o[7:0]; // SB
                            3'b101: mtimecmp[47:40] <= exe_wb_dat_o[7:0]; // SB
                            3'b110: mtimecmp[55:48] <= exe_wb_dat_o[7:0]; // SB
                            3'b111: mtimecmp[63:56] <= exe_wb_dat_o[7:0]; // SB
                        endcase
                    end
                    2'b10: begin
                        case (exe_wb_adr_o[2:0])
                            3'b000: mtimecmp[31:0] <= exe_wb_dat_o; // SW
                            3'b100: mtimecmp[63:32] <= exe_wb_dat_o; // SW
                        endcase
                    end
                endcase
            end
        end
    end
end

logic [1:0] byte_offset;
assign byte_offset = exe_wb_adr_o[1:0]; // 计算字节偏移
assign csr_raw = inst_type_i == CSRRC | inst_type_i == CSRRS | inst_type_i == CSRRW; // 判断是否为csr指令
assign mem_stall =( (mem_read | mem_we) & (!wb_ack_i) & !(rw_mtime | rw_mtimecmp) & !csr_raw & !time_interrupt ) | (fence_i & !dcache_fence_ack_i); // 判断是否为结构冲突
assign rf_wen_o = rf_wen_i & !time_interrupt; // 判断是否写寄存器
always_comb begin
    wb_we_o = 1'b0;
    wb_adr_o = 32'b0;
    wb_dat_o = 32'b0;
    wb_sel_o = 4'b1111;
    rf_wdata = exe_wb_adr_o;
    csr_inst_type_o = NOP;
    csr_data_o = 32'b0;
    csr_addr_o = 12'b0;
    branch_en = 1'b0;
    next_pc = 32'b0;

    mem_mmu_sel = 1'b1;
    fence_i = 1'b0;
    sfence_vma = 1'b0;

    if (page_fault) begin
        csr_data_o = pc_i;
        next_pc = csr_data_i;
        branch_en = 1'b1;
    end
    
    if (time_interrupt) begin
        rf_wdata = 32'b0;
        csr_data_o = pc_i;
        next_pc = csr_data_i;
        branch_en = 1'b1;
    end else if ((rw_mtime | rw_mtimecmp) & mem_read) begin
        if (rw_mtime) begin
            case (mem_size)
                2'b00: begin
                    case (exe_wb_adr_o[2:0])
                        3'b000: rf_wdata = {{24{mtime[7]}} ,mtime[7:0]}; // LB
                        3'b001: rf_wdata = {{24{mtime[15]}}, mtime[15:8]}; // LB
                        3'b010: rf_wdata = {{24{mtime[23]}}, mtime[23:16]}; // LB
                        3'b011: rf_wdata = {{24{mtime[31]}}, mtime[31:24]}; // LB
                        3'b100: rf_wdata = {{24{mtime[39]}}, mtime[39:32]}; // LB
                        3'b101: rf_wdata = {{24{mtime[47]}}, mtime[47:40]}; // LB
                        3'b110: rf_wdata = {{24{mtime[55]}}, mtime[55:48]}; // LB
                        3'b111: rf_wdata = {{24{mtime[63]}}, mtime[63:56]}; // LB
                    endcase
                end
                2'b10: begin
                    case (exe_wb_adr_o[2:0])
                        3'b000: rf_wdata = mtime[31:0]; // LW
                        3'b100: rf_wdata = mtime[63:32]; // LW
                    endcase
                end
            endcase
        end else if (rw_mtimecmp) begin
            case (mem_size)
                2'b00: begin
                    case (exe_wb_adr_o[2:0])
                        3'b000: rf_wdata = {{24{mtimecmp[7]}} ,mtimecmp[7:0]}; // LB
                        3'b001: rf_wdata = {{24{mtimecmp[15]}}, mtimecmp[15:8]}; // LB
                        3'b010: rf_wdata = {{24{mtimecmp[23]}}, mtimecmp[23:16]}; // LB
                        3'b011: rf_wdata = {{24{mtimecmp[31]}}, mtimecmp[31:24]}; // LB
                        3'b100: rf_wdata = {{24{mtimecmp[39]}}, mtimecmp[39:32]}; // LB
                        3'b101: rf_wdata = {{24{mtimecmp[47]}}, mtimecmp[47:40]}; // LB
                        3'b110: rf_wdata = {{24{mtimecmp[55]}}, mtimecmp[55:48]}; // LB
                        3'b111: rf_wdata = {{24{mtimecmp[63]}}, mtimecmp[63:56]}; // LB
                    endcase
                end
                2'b10: begin
                    case (exe_wb_adr_o[2:0])
                        3'b000: rf_wdata = mtimecmp[31:0]; // LW
                        3'b100: rf_wdata = mtimecmp[63:32]; // LW
                    endcase
                end
            endcase
        end
    end else if ((mem_read || mem_we) & !(rw_mtime | rw_mtimecmp) & !csr_raw) begin
        wb_we_o = 1'b0;
        if (mem_we) begin
                wb_we_o = 1'b1;  // 写使能
                wb_adr_o = exe_wb_adr_o;  // 地址
                // 写入数据
                case (mem_size)
                    2'b00: begin  // SB
                        wb_sel_o = 4'b0001 << (exe_wb_adr_o[1:0]);
                        wb_dat_o = exe_wb_dat_o << (8 * exe_wb_adr_o[1:0]);
                    end
                    2'b10: begin  // SW
                        wb_sel_o = 4'b1111;
                        wb_dat_o = exe_wb_dat_o; // 32位写入
                    end
                endcase
        end else begin  // 读取数据
            wb_adr_o = exe_wb_adr_o;
            wb_sel_o = 4'b1111;
        end

        // 等待数据返回
        if (wb_ack_i) begin
            if (mem_read) begin
                case (mem_size)
                    2'b00: begin
                        case (byte_offset)
                            2'b00: rf_wdata = {{24{wb_dat_i[7]}}, wb_dat_i[7:0]}; // LB
                            2'b01: rf_wdata = {{24{wb_dat_i[15]}}, wb_dat_i[15:8]}; // LB
                            2'b10: rf_wdata = {{24{wb_dat_i[23]}}, wb_dat_i[23:16]}; // LB
                            2'b11: rf_wdata = {{24{wb_dat_i[31]}}, wb_dat_i[31:24]}; // LB
                        endcase
                    end
                    2'b10: rf_wdata = wb_dat_i; // LW
                endcase
            end
        end
    end else begin
        // csr寄存器和异常
        case (inst_type_i)
            CSRRC, CSRRS, CSRRW: begin
                csr_addr_o = csr_addr_i;
                csr_inst_type_o = inst_type_i;
                csr_data_o = exe_wb_adr_o;
                rf_wdata = csr_data_i;
            end
            EBREAK, ECALL, MRET: begin
                csr_inst_type_o = inst_type_i;
                csr_data_o = pc_i;
                next_pc = csr_data_i;
                branch_en = 1'b1;
            end
        endcase
    end
end

endmodule