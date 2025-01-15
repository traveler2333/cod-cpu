`default_nettype none

module thinpad_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮开关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时为 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时为 1
    output wire [15:0] leds,       // 16 位 LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信号
    output wire uart_rdn,        // 读串口信号，低有效
    output wire uart_wrn,        // 写串口信号，低有效
    input  wire uart_dataready,  // 串口数据准备好
    input  wire uart_tbre,       // 发送数据标志
    input  wire uart_tsre,       // 数据发送完毕标志

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire base_ram_ce_n,  // BaseRAM 片选，低有效
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有效
    output wire base_ram_we_n,  // BaseRAM 写使能，低有效

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire ext_ram_ce_n,  // ExtRAM 片选，低有效
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有效
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有效

    // 直连串口信号
    output wire txd,  // 直连串口发送端
    input  wire rxd,  // 直连串口接收端

    // Flash 存储器信号，参考 JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,  // Flash 片选信号，低有效
    output wire flash_oe_n,  // Flash 读使能信号，低有效
    output wire flash_we_n,  // Flash 写使能信号，低有效
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

    // USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素，3 位
    output wire [2:0] video_green,  // 绿色像素，3 位
    output wire [1:0] video_blue,   // 蓝色像素，2 位
    output wire       video_hsync,  // 行同步（水平同步）信号
    output wire       video_vsync,  // 场同步（垂直同步）信号
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐区
);

  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_20M;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设置
      .clk_out2(clk_20M),  // 时钟输出 2，频率在 IP 配置界面中设置
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出，"1"表示时钟稳定，
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_50M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end

  logic sys_clk;
  logic sys_rst;

  assign sys_clk = clk_50M;
  assign sys_rst = reset_of_clk10M;

  // 本实验不使用 CPLD 串口，禁用防止总线冲突
  assign uart_rdn = 1'b1;
  assign uart_wrn = 1'b1;
  
  //---------------------------正式开始接线-------------------

  // 定义 IF 和 MEM 模块的仲裁器连接信号
  logic [31:0] if_mmu_adr_o, if_mmu_dat_o, if_mmu_dat_i;
  logic        if_mmu_we_o, if_mmu_stb_o, if_mmu_ack_i, if_mmu_cyc_o;
  logic [3:0]  if_mmu_sel_o;

  logic [31:0] mem_mmu_adr_o, mem_mmu_dat_o, mem_mmu_dat_i;
  logic        mem_mmu_we_o, mem_mmu_stb_o, mem_mmu_ack_i,  mem_mmu_cyc_o;
  logic [3:0]  mem_mmu_sel_o;

  // 定义仲裁器和 MUX 之间的接口信号
  logic        arb_wbm_cyc_o;
  logic        arb_wbm_stb_o;
  logic        arb_wbm_ack_i;
  logic [31:0] arb_wbm_adr_o;
  logic [31:0] arb_wbm_dat_o;
  logic [31:0] arb_wbm_dat_i;
  logic [3:0]  arb_wbm_sel_o;
  logic        arb_wbm_we_o;

// 定义各个模块的发出信号：
    // Hazard
    logic hazard_pc_stall, hazard_if_id_stall, hazard_id_exe_stall, hazard_exe_mem_stall;
    logic hazard_if_id_flush, hazard_id_exe_nop, hazard_exe_mem_nop;
    logic hazard_mem_wb_nop, hazard_mem_wb_stall, hazard_branch_en;
    logic hazard_error_branch_en;
    logic [31:0] hazard_error_branch_target;
    // BTB
    logic [31:0] btb_target;
    logic btb_hit;
    logic [31:0] btb_target_exe;
    logic btb_hit_exe;
    // ALU
    logic [31:0] alu_result;
    // Regfile
    logic [31:0] rf_rdata_a, rf_rdata_b;
    // PC_MUX
    logic [31:0] pc;
    // IF_IMM
    logic [31:0] if_instr;
    logic if_stall;
    // pipline_register_if_id
    logic [31:0] if_id_pc, if_id_instr;
    // ID
    logic [4:0] id_rf_raddr_a, id_rf_raddr_b;
    logic [4:0] id_inst_type;
    logic [3:0] id_alu_op;
    logic [31:0] id_rdata_a, id_rdata_b;
    logic [4:0] id_raddr_a, id_raddr_b, id_raddr_rd;
    logic id_mem_read, id_mem_we;
    logic [1:0] id_mem_size;
    logic id_rf_wen;
    // pipline_register_id_exe
    logic [31:0] id_exe_rdata_a, id_exe_rdata_b;
    logic [3:0] id_exe_alu_op;
    logic [4:0] id_exe_raddr_a, id_exe_raddr_b, id_exe_raddr_d;
    logic id_exe_mem_read, id_exe_mem_we;
    logic [1:0] id_exe_mem_size;
    logic id_exe_rf_wen;
    logic [4:0] id_exe_inst_type;
    logic [31:0] id_exe_pc;

    // EXE
    logic [31:0] exe_alu_a, exe_alu_b, exe_alu_result;
    logic [3:0] exe_alu_op;

    logic [4:0] exe_inst_type;
    logic exe_branch_en;
    logic [31:0] exe_next_pc;

    logic [31:0] exe_update_pc, exe_update_target;
    logic exe_update_valid, exe_update_en;
    
    logic [11:0] exe_csr_addr;
    logic [31:0] exe_pc;
    logic [31:0] exe_wdata;

    // pipline_register_exe_mem
    logic [31:0] exe_mem_addr;
    logic [4:0] exe_mem_rf_raddr_d;
    logic exe_mem_mem_read, exe_mem_mem_we;
    logic [1:0] exe_mem_mem_size;
    logic exe_mem_rf_wen;
    logic [31:0]exe_mem_wdata;
    logic [4:0] exe_mem_inst_type;
    logic [11:0] exe_mem_csr_addr;
    logic [31:0] exe_mem_pc;

    // MEM
    logic [31:0] mem_rf_wdata;
    logic mem_stall;
    logic [31:0] mem_csr_data;
    logic [4:0] mem_inst_type;
    logic [11:0] mem_csr_addr;
    logic mem_rf_wen;
    logic mem_branch_en;
    logic [31:0] mem_next_pc;
    logic timeout;
    logic page_fault;

    // CSR
    logic [31:0] csr_data;
    logic time_interrupt;

    // pipline_register_mem_wb
    logic [31:0] mem_wb_rf_wdata;
    logic [ 4:0] mem_wb_rf_addr_d;
    logic mem_wb_rf_wen;

    //instruction_cache
    logic [31:0] instr_cache_addr;
    logic instr_cache_read_en;
    logic instr_cache_write_en;
    logic [31:0] instr_cache_data;
    logic instr_cache_hit;
    logic [31:0] instr_cache_data_update;

    //mode
    logic [1:0] if_mem_mode;
    logic [31:0] if_mem_page_table_base;
    logic [3:0] if_mmu_exception_code;
    logic if_mmu_page_fault;
    logic [3:0] mem_exception_code;
    logic mem_page_fault;

    //wishbone接口
    logic [31:0] mmuif_wb_adr_o;
    logic [31:0] mmuif_wb_dat_o;
    logic [31:0] mmuif_wb_dat_i;
    logic mmuif_wb_we_o;
    logic [3:0] mmuif_wb_sel_o;
    logic mmuif_wb_stb_o;
    logic mmuif_wb_ack_i;
    logic mmuif_wb_err_i;
    logic mmuif_wb_rty_i;
    logic mmuif_wb_cyc_o;

    logic [31:0] mmumem_wb_adr_o;
    logic [31:0] mmumem_wb_dat_o;
    logic [31:0] mmumem_wb_dat_i;
    logic mmumem_wb_we_o;
    logic [3:0] mmumem_wb_sel_o;
    logic mmumem_wb_stb_o;
    logic mmumem_wb_ack_i;
    logic mmumem_wb_err_i;
    logic mmumem_wb_rty_i;
    logic mmumem_wb_cyc_o;
    //控制if还是mem
    logic if_mmu_sel;
    logic mem_mmu_sel;

    logic fence_i;
    logic sfence_vma;
    PC_MUX u_pc_mux (
        .clk(sys_clk),
        .reset(sys_rst),
        .branch_en(hazard_branch_en),
        .branch_addr(exe_next_pc),
        
        .btb_target(btb_target),
        .btb_hit(btb_hit),

        .error_branch_en(hazard_error_branch_en),
        .error_branch_target(hazard_error_branch_target),

        .stall(hazard_pc_stall),
        .pc(pc)
    );

    BTB u_btb (
        .clk(sys_clk),
        .reset(sys_rst),

        .BTB_stall(hazard_pc_stall),

        .pc(pc),
        .btb_target(btb_target),
        .btb_hit(btb_hit),

        .pc_exe(id_exe_pc),
        .btb_target_exe(btb_target_exe),
        .btb_hit_exe(btb_hit_exe),

        .update_en(exe_update_en),
        .update_pc(exe_update_pc),
        .update_target(exe_update_target),
        .update_valid(exe_update_valid)
    );

    InstructionCache u_icache (
        .clk(sys_clk),
        .rst(sys_rst),

        .addr(instr_cache_addr),
        .read_en(instr_cache_read_en),
        .if_mem_ready(instr_cache_write_en),
        .data(instr_cache_data),
        .hit(instr_cache_hit),
        .if_mem_data(instr_cache_data_update),
        .fence_i(fence_i)
    );

    logic [31:0] if_mmu_tlb_addr;
    logic if_mmu_tlb_read_en;
    logic [31:0] if_tlb_mmu_data;
    logic if_mmu_tlb_hit;
    logic [31:0] if_mmu_tlb_data;
    logic if_mmu_tlb_ready;
    TLB u_tlb_if (
        .clk(sys_clk),
        .rst(sys_rst),

        .addr(if_mmu_tlb_addr),
        .read_en(if_mmu_tlb_read_en),
        .data(if_tlb_mmu_data),
        .hit(if_mmu_tlb_hit),
        .mmu_data(if_mmu_tlb_data),
        .mmu_ready(if_mmu_tlb_ready),
        .sfence_vma(sfence_vma)
    );

    logic [31:0] mem_mmu_tlb_addr;
    logic mem_mmu_tlb_read_en;
    logic [31:0] mem_tlb_mmu_data;
    logic mem_mmu_tlb_hit;
    logic [31:0] mem_mmu_tlb_data;
    logic mem_mmu_tlb_ready;

    TLB u_tlb_mem (
        .clk(sys_clk),
        .rst(sys_rst),

        .addr(mem_mmu_tlb_addr),
        .read_en(mem_mmu_tlb_read_en),
        .data(mem_tlb_mmu_data),
        .hit(mem_mmu_tlb_hit),
        .mmu_data(mem_mmu_tlb_data),
        .mmu_ready(mem_mmu_tlb_ready),
        .sfence_vma(sfence_vma)
    );

    logic [3:0] if_exception_code,if_id_exception_code,id_exe_exception_code,exe_mem_exception_code;
    logic if_page_fault,if_id_page_fault,id_exe_page_fault,exe_mem_page_fault;

    IF_IMM u_if_imm (
        .clk(sys_clk),
        .rst(sys_rst),
        .pc(pc),

        .wb_adr_o(if_mmu_adr_o),
        .wb_dat_o(if_mmu_dat_o),
        .wb_we_o(if_mmu_we_o),
        .wb_sel_o(if_mmu_sel_o),
        .wb_cyc_o(if_mmu_cyc_o),
        .wb_stb_o(if_mmu_stb_o),

        .wb_dat_i(if_mmu_dat_i),
        .wb_ack_i(if_mmu_ack_i),
        .exception_code(if_mmu_exception_code),
        .page_fault(if_mmu_page_fault),

        .instruction(if_instr),
        .if_stall(if_stall),
        .exception_code_o(if_exception_code),
        .page_fault_o(if_page_fault),

        .icache_addr(instr_cache_addr),
        .icache_read_en(instr_cache_read_en),
        .icache_data(instr_cache_data),
        .icache_hit(instr_cache_hit),
        .icache_data_update(instr_cache_data_update),
        .icache_write_en(instr_cache_write_en),

        .if_mmu_sel(if_mmu_sel)
    );

    MMU mmu_inst_if (
        .clk(sys_clk),
        .rst(sys_rst),

        //mode
        .mode(if_mem_mode),
        .page_table_base(if_mem_page_table_base),

        //if传入的信号
        .virtual_addr(if_mmu_adr_o),
        .if_mem_wb_dat_i(if_mmu_dat_o),
        .if_mem_wb_we_i(if_mmu_we_o),
        .if_mem_wb_sel_i(if_mmu_sel_o),
        .if_mem_wb_stb_i(if_mmu_stb_o),
        .if_mem_wb_cyc_i(if_mmu_cyc_o),

        //传给if的信号
        .if_mem_wb_dat_o(if_mmu_dat_i),
        .if_mem_wb_ack_o(if_mmu_ack_i),
        .exception_code(if_mmu_exception_code),
        .page_fault(if_mmu_page_fault),

        //wishbone接口
        .wb_adr_o(mmuif_wb_adr_o),
        .wb_dat_o(mmuif_wb_dat_o),
        .wb_dat_i(mmuif_wb_dat_i),
        .wb_we_o(mmuif_wb_we_o),
        .wb_sel_o(mmuif_wb_sel_o),
        .wb_stb_o(mmuif_wb_stb_o),
        .wb_ack_i(mmuif_wb_ack_i),
        .wb_err_i(mmuif_wb_err_i),
        .wb_rty_i(mmuif_wb_rty_i),
        .wb_cyc_o(mmuif_wb_cyc_o),

        //控制if还是mem
        .mem_if_sel(if_mmu_sel),

        .tlb_addr(if_mmu_tlb_addr),
        .tlb_read_en(if_mmu_tlb_read_en),
        .tlb_data(if_tlb_mmu_data),
        .tlb_hit(if_mmu_tlb_hit),
        .tlb_data_update(if_mmu_tlb_data),
        .tlb_write_en(if_mmu_tlb_ready)
    );


    pipline_register_if_id u_if_id (
        .clk(sys_clk),
        .rst(sys_rst),
        .stall_i(hazard_if_id_stall),
        .flush_i(hazard_if_id_flush),
        .pc_i(pc),
        .instr_i(if_instr),
        .pc_o(if_id_pc),
        .instr_o(if_id_instr),

        .exception_code_i(if_exception_code),
        .page_fault_i(if_page_fault),
        .exception_code_o(if_id_exception_code),
        .page_fault_o(if_id_page_fault)
    );

    logic [31:0] id_imm;
    ID u_id (
        .instr(if_id_instr),
        .pc(if_id_pc),

        .rf_raddr_a(id_rf_raddr_a),
        .rf_raddr_b(id_rf_raddr_b),
        .rf_rdata_a_i(rf_rdata_a),
        .rf_rdata_b_i(rf_rdata_b),

        .inst_type_o(id_inst_type),
        .imm_o(id_imm),

        // .operand1(id_operand1),
        // .operand2(id_operand2),
        .rdata_a(id_rdata_a),
        .rdata_b(id_rdata_b),
        .alu_op(id_alu_op),

        // .id_wdata(id_wdata),

        .raddr_a(id_raddr_a),
        .raddr_b(id_raddr_b),
        .raddr_rd(id_raddr_rd),

        .mem_read(id_mem_read),
        .mem_we(id_mem_we),
        .mem_size(id_mem_size),
        .rf_wen(id_rf_wen)
    );

    logic [31:0] id_exe_imm;
    pipline_register_id_exe u_id_exe (
        .clk(sys_clk),
        .rst(sys_rst),
        .stall_i(hazard_id_exe_stall),
        .nop_i(hazard_id_exe_nop),

        .inst_type_i(id_inst_type),
        .inst_type_o(id_exe_inst_type),
        .pc_i(if_id_pc),
        .pc_o(id_exe_pc),
        .imm_i(id_imm),
        .imm_o(id_exe_imm),

        .rdata_a_i(id_rdata_a),
        .rdata_b_i(id_rdata_b),
        .alu_op_i(id_alu_op),

        .rdata_a_o(id_exe_rdata_a),
        .rdata_b_o(id_exe_rdata_b),
        .alu_op_o(id_exe_alu_op),

        .raddr_a_i(id_raddr_a),
        .raddr_b_i(id_raddr_b),
        .raddr_d_i(id_raddr_rd),

        .raddr_a_o(id_exe_raddr_a),
        .raddr_b_o(id_exe_raddr_b),
        .raddr_d_o(id_exe_raddr_d),

        .mem_read_i(id_mem_read),
        .mem_we_i(id_mem_we),
        .mem_size_i(id_mem_size),
        .rf_wen_i(id_rf_wen),

        .mem_read_o(id_exe_mem_read),
        .mem_we_o(id_exe_mem_we),
        .mem_size_o(id_exe_mem_size),
        .rf_wen_o(id_exe_rf_wen),

        .exception_code_i(if_id_exception_code),
        .page_fault_i(if_id_page_fault),
        .exception_code_o(id_exe_exception_code),
        .page_fault_o(id_exe_page_fault)
    );


    EXE u_exe (
        .clk(sys_clk),
        .rst(sys_rst),
        .stall(hazard_id_exe_stall),
        .inst_type_i(id_exe_inst_type),
        .inst_type_o(exe_inst_type),
        .pc_i(id_exe_pc),
        .imm_i(id_exe_imm),


        .alu_op_i(id_exe_alu_op),

        .alu_a(exe_alu_a),
        .alu_b(exe_alu_b),
        .op(exe_alu_op),

        .alu_result_i(alu_result),
        .alu_result_o(exe_alu_result),
        .wdata_o(exe_wdata),
        .csr_addr_o(exe_csr_addr),
        .pc_o(exe_pc),

        .rf_raddr_a(id_exe_raddr_a),
        .rf_raddr_b(id_exe_raddr_b),
        .rdata_a_i(id_exe_rdata_a),
        .rdata_b_i(id_exe_rdata_b),

        .next_raddr_d(exe_mem_rf_raddr_d),
        .next_exe_mem_result(exe_mem_addr),

        .next_mem_read(exe_mem_mem_read),
        .next_rf_wen(exe_mem_rf_wen),

        .mem_wb_result(mem_wb_rf_wdata),
        .mem_wb_raddr_rd(mem_wb_rf_addr_d),

        .mem_wb_rf_wen(mem_wb_rf_wen),

        .next_pc(exe_next_pc),
        .branch_en(exe_branch_en),

        .btb_target_exe(btb_target_exe),
        .btb_hit_exe(btb_hit_exe),

        .update_en(exe_update_en),
        .update_pc(exe_update_pc),
        .update_target(exe_update_target),
        .update_valid(exe_update_valid)
    );



    pipline_register_exe_mem u_exe_mem (
        .clk(sys_clk),
        .rst(sys_rst),
        .stall(hazard_exe_mem_stall),
        .nop(hazard_exe_mem_nop),

        .addr_i(exe_alu_result),
        .addr_o(exe_mem_addr),

        .data_i(exe_wdata),
        .data_o(exe_mem_wdata),

        .rf_raddr_d_i(id_exe_raddr_d),
        .rf_raddr_d_o(exe_mem_rf_raddr_d),

        .inst_type_i(exe_inst_type),
        .inst_type_o(exe_mem_inst_type),

        .csr_addr_i(exe_csr_addr),
        .csr_addr_o(exe_mem_csr_addr),

        .pc_i(exe_pc),
        .pc_o(exe_mem_pc),

        .mem_read_i(id_exe_mem_read),
        .mem_we_i(id_exe_mem_we),
        .mem_size_i(id_exe_mem_size),
        .rf_wen_i(id_exe_rf_wen),

        .mem_read_o(exe_mem_mem_read),
        .mem_we_o(exe_mem_mem_we),
        .mem_size_o(exe_mem_mem_size),
        .rf_wen_o(exe_mem_rf_wen),
        
        //exception_code和page_fault
        .exception_code_i(id_exe_exception_code),
        .page_fault_i(id_exe_page_fault),
        .exception_code_o(exe_mem_exception_code),
        .page_fault_o(exe_mem_page_fault)
    );
    logic dcache_fence_ack;
    MEM u_mem (
        .clk(sys_clk),
        .rst(sys_rst),
        
        .wb_adr_o(mem_mmu_adr_o),
        .wb_dat_o(mem_mmu_dat_o),
        .wb_we_o(mem_mmu_we_o),
        .wb_sel_o(mem_mmu_sel_o),
        .wb_stb_o(mem_mmu_stb_o),
        .wb_cyc_o(mem_mmu_cyc_o),

        .wb_ack_i(mem_mmu_ack_i),
        .wb_dat_i(mem_mmu_dat_i),
        .mem_exception_code(mem_exception_code),
        .mem_page_fault(mem_page_fault),
        .if_exception_code(exe_mem_exception_code),
        .if_page_fault(exe_mem_page_fault),

        .exe_wb_adr_o(exe_mem_addr),
        .exe_wb_dat_o(exe_mem_wdata),
        .inst_type_i(exe_mem_inst_type),
        .csr_addr_i(exe_mem_csr_addr),
        .pc_i(exe_mem_pc),
        .rf_wen_i(exe_mem_rf_wen),
        .mem_read(exe_mem_mem_read),
        .mem_we(exe_mem_mem_we),
        .mem_size(exe_mem_mem_size),
        .rf_wdata(mem_rf_wdata),
        .rf_wen_o(mem_rf_wen),
        .mem_stall(mem_stall),

        .csr_inst_type_o(mem_inst_type),
        .csr_addr_o(mem_csr_addr),
        .csr_data_o(mem_csr_data),
        .csr_data_i(csr_data),
        .branch_en(mem_branch_en),
        .next_pc(mem_next_pc),

        .timeout(timeout),
        .time_interrupt(time_interrupt),

        .page_fault(page_fault),

        .mem_mmu_sel(mem_mmu_sel),
        
        .fence_i(fence_i),
        .sfence_vma(sfence_vma),
        .dcache_fence_ack_i(dcache_fence_ack)
    );


    MMU mmu_inst_mem (
        .clk(sys_clk),
        .rst(sys_rst),
        //mode
        .mode(if_mem_mode),
        .page_table_base(if_mem_page_table_base),

        //mem传入的信号
        .virtual_addr(mem_mmu_adr_o),
        .if_mem_wb_dat_i(mem_mmu_dat_o),
        .if_mem_wb_we_i(mem_mmu_we_o),
        .if_mem_wb_sel_i(mem_mmu_sel_o),
        .if_mem_wb_stb_i(mem_mmu_stb_o),
        .if_mem_wb_cyc_i(mem_mmu_cyc_o),

        //传给mem的信号
        .if_mem_wb_dat_o(mem_mmu_dat_i),
        .if_mem_wb_ack_o(mem_mmu_ack_i),
        .exception_code(mem_exception_code),
        .page_fault(mem_page_fault),

        //wishbone接口
        .wb_adr_o(mmumem_wb_adr_o),
        .wb_dat_o(mmumem_wb_dat_o),
        .wb_dat_i(mmumem_wb_dat_i),
        .wb_we_o(mmumem_wb_we_o),
        .wb_sel_o(mmumem_wb_sel_o),
        .wb_stb_o(mmumem_wb_stb_o),
        .wb_ack_i(mmumem_wb_ack_i),
        .wb_err_i(mmumem_wb_err_i),
        .wb_rty_i(mmumem_wb_rty_i),
        .wb_cyc_o(mmumem_wb_cyc_o),

        //控制mem还是mem
        .mem_if_sel(mem_mmu_sel),

        .tlb_addr(mem_mmu_tlb_addr),
        .tlb_read_en(mem_mmu_tlb_read_en),
        .tlb_data(mem_tlb_mmu_data),
        .tlb_hit(mem_mmu_tlb_hit),
        .tlb_data_update(mem_mmu_tlb_data),
        .tlb_write_en(mem_mmu_tlb_ready)
    );
    logic [31:0] dcache_wb_adr_o;
    logic [31:0] dcache_wb_dat_o;
    logic [31:0] dcache_wb_dat_i;
    logic dcache_wb_we_o;
    logic [3:0] dcache_wb_sel_o;
    logic dcache_wb_stb_o;
    logic dcache_wb_cyc_o;
    logic dcache_wb_ack_i;
    logic dcache_wb_err_i;
    logic dcache_wb_rty_i;
    data_cache dcache (
        .clk(sys_clk),
        .rst(sys_rst),

        // 与 MMU 的接口
        .mmu_adr_i(mmumem_wb_adr_o),
        .mmu_dat_i(mmumem_wb_dat_o),
        .mmu_dat_o(mmumem_wb_dat_i),
        .mmu_we_i(mmumem_wb_we_o),
        .mmu_sel_i(mmumem_wb_sel_o),
        .mmu_stb_i(mmumem_wb_stb_o),
        .mmu_ack_o(mmumem_wb_ack_i),
        .mmu_err_o(mmumem_wb_err_i),
        .mmu_rty_o(mmumem_wb_rty_i),
        .mmu_cyc_i(mmumem_wb_cyc_o),

        // 与 Wishbone 总线的接口
        .wb_adr_o(dcache_wb_adr_o),
        .wb_dat_o(dcache_wb_dat_o),
        .wb_dat_i(dcache_wb_dat_i),
        .wb_we_o(dcache_wb_we_o),
        .wb_sel_o(dcache_wb_sel_o),
        .wb_stb_o(dcache_wb_stb_o),
        .wb_cyc_o(dcache_wb_cyc_o),
        .wb_ack_i(dcache_wb_ack_i),
        .wb_err_i(dcache_wb_err_i),
        .wb_rty_i(dcache_wb_rty_i)
    );

    pipline_register_mem_wb u_mem_wb (
        .clk(sys_clk),
        .rst(sys_rst),

        .rf_wdata_i(mem_rf_wdata),
        .rf_addr_d_i(exe_mem_rf_raddr_d),
        .rf_wen_i(mem_rf_wen),

        .rf_wdata_o(mem_wb_rf_wdata),
        .rf_addr_d_o(mem_wb_rf_addr_d),
        .rf_wen_o(mem_wb_rf_wen),

        .stall(hazard_mem_wb_stall),
        .nop(hazard_mem_wb_nop)
    );




    Regfile u_regfile (
        .clk(sys_clk),
        .reset(sys_rst),

        .wb_waddr(mem_wb_rf_addr_d),
        .wb_wdata(mem_wb_rf_wdata),
        .wb_we(mem_wb_rf_wen),

        .id_raddr_a(id_rf_raddr_a),
        .id_raddr_b(id_rf_raddr_b),
        .id_rdata_a(rf_rdata_a),
        .id_rdata_b(rf_rdata_b)
    );


    Csr csr (
        .clk(sys_clk),
        .rst(sys_rst),

        .inst_type(mem_inst_type),
        .data_i(mem_csr_data),
        .csr_addr_i(mem_csr_addr),
        .data_o(csr_data),

        .time_interrupt(time_interrupt),
        .timeout(timeout),

        .page_fault(page_fault),

        .mode(if_mem_mode),
        .page_table_base(if_mem_page_table_base)
    );


    Hazard u_hazard (
        .clk(sys_clk),
        .rst(sys_rst),

        .if_stall(if_stall),
        .mem_stall(mem_stall),

        .exe_r_a(id_exe_raddr_a),
        .exe_r_b(id_exe_raddr_b),
        .mem_rd(exe_mem_rf_raddr_d),
        .mem_read(exe_mem_mem_read),

        .branch_en_i(exe_branch_en),
        .branch_en_o(hazard_branch_en),

        .mem_branch_en_i(mem_branch_en),
        .mem_branch_target_i(mem_next_pc),
        .mem_branch_en_o(hazard_error_branch_en),
        .mem_branch_target_o(hazard_error_branch_target),

        .pc_stall(hazard_pc_stall),

        .if_id_stall(hazard_if_id_stall),
        .if_id_flush(hazard_if_id_flush),

        .id_exe_stall(hazard_id_exe_stall),
        .id_exe_nop(hazard_id_exe_nop),

        .exe_mem_stall(hazard_exe_mem_stall),
        .exe_mem_nop(hazard_exe_mem_nop),

        .mem_wb_nop(hazard_mem_wb_nop),
        .mem_wb_stall(hazard_mem_wb_stall)
    );


    alu u_alu (
        .a(exe_alu_a),
        .b(exe_alu_b),
        .op(exe_alu_op),
        .y(alu_result)
    );



  /* =========== 仲裁器 begin =========== */
  wb_arbiter_2 #(
      .DATA_WIDTH(32),
      .ADDR_WIDTH(32)
  ) u_wb_arbiter_2 (
      .clk(sys_clk),
      .rst(sys_rst),

      // Wishbone master 0 input (IF)
      .wbm0_adr_i(mmuif_wb_adr_o),
      .wbm0_dat_i(mmuif_wb_dat_o),
      .wbm0_dat_o(mmuif_wb_dat_i),
      .wbm0_we_i(mmuif_wb_we_o),
      .wbm0_sel_i(mmuif_wb_sel_o),
      .wbm0_stb_i(mmuif_wb_stb_o),
      .wbm0_ack_o(mmuif_wb_ack_i),
      .wbm0_err_o(mmuif_wb_err_i),
      .wbm0_rty_o(mmuif_wb_rty_i),
      .wbm0_cyc_i(mmuif_wb_cyc_o),

      // Wishbone master 1 input (MEM)
      .wbm1_adr_i(dcache_wb_adr_o),
      .wbm1_dat_i(dcache_wb_dat_o),
      .wbm1_dat_o(dcache_wb_dat_i),
      .wbm1_we_i(dcache_wb_we_o),
      .wbm1_sel_i(dcache_wb_sel_o),
      .wbm1_stb_i(dcache_wb_stb_o),
      .wbm1_ack_o(dcache_wb_ack_i),
      .wbm1_err_o(dcache_wb_err_i),
      .wbm1_rty_o(dcache_wb_rty_i),
      .wbm1_cyc_i(dcache_wb_cyc_o),

      // Wishbone slave output (to MUX)
      .wbs_adr_o(arb_wbm_adr_o),
      .wbs_dat_i(arb_wbm_dat_i),
      .wbs_dat_o(arb_wbm_dat_o),
      .wbs_we_o(arb_wbm_we_o),
      .wbs_sel_o(arb_wbm_sel_o),
      .wbs_stb_o(arb_wbm_stb_o),
      .wbs_ack_i(arb_wbm_ack_i),
      .wbs_err_i(),
      .wbs_rty_i(),
      .wbs_cyc_o(arb_wbm_cyc_o)
  );
  /* =========== 仲裁器 end =========== */

  /* =========== MUX begin =========== */
  logic wbs0_cyc_o;
  logic wbs0_stb_o;
  logic wbs0_ack_i;
  logic [31:0] wbs0_adr_o;
  logic [31:0] wbs0_dat_o;
  logic [31:0] wbs0_dat_i;
  logic [3:0] wbs0_sel_o;
  logic wbs0_we_o;

  logic wbs1_cyc_o;
  logic wbs1_stb_o;
  logic wbs1_ack_i;
  logic [31:0] wbs1_adr_o;
  logic [31:0] wbs1_dat_o;
  logic [31:0] wbs1_dat_i;
  logic [3:0] wbs1_sel_o;
  logic wbs1_we_o;

  logic wbs2_cyc_o;
  logic wbs2_stb_o;
  logic wbs2_ack_i;
  logic [31:0] wbs2_adr_o;
  logic [31:0] wbs2_dat_o;
  logic [31:0] wbs2_dat_i;
  logic [3:0] wbs2_sel_o;
  logic wbs2_we_o;

  
  wb_mux_3 wb_mux (
      .clk(sys_clk),
      .rst(sys_rst),

      // Master interface (to Arbiter)
      .wbm_adr_i(arb_wbm_adr_o),
      .wbm_dat_i(arb_wbm_dat_o),
      .wbm_dat_o(arb_wbm_dat_i),
      .wbm_we_i (arb_wbm_we_o),
      .wbm_sel_i(arb_wbm_sel_o),
      .wbm_stb_i(arb_wbm_stb_o),
      .wbm_ack_o(arb_wbm_ack_i),
      .wbm_err_o(),
      .wbm_rty_o(),
      .wbm_cyc_i(arb_wbm_cyc_o),

      // Slave interface 0 (to BaseRAM controller)
      // Address range: 0x8000_0000 ~ 0x803F_FFFF
      .wbs0_addr    (32'h8000_0000),
      .wbs0_addr_msk(32'hFFC0_0000),

      .wbs0_adr_o(wbs0_adr_o),
      .wbs0_dat_i(wbs0_dat_i),
      .wbs0_dat_o(wbs0_dat_o),
      .wbs0_we_o (wbs0_we_o),
      .wbs0_sel_o(wbs0_sel_o),
      .wbs0_stb_o(wbs0_stb_o),
      .wbs0_ack_i(wbs0_ack_i),
      .wbs0_err_i('0),
      .wbs0_rty_i('0),
      .wbs0_cyc_o(wbs0_cyc_o),

      // Slave interface 1 (to ExtRAM controller)
      // Address range: 0x8040_0000 ~ 0x807F_FFFF
      .wbs1_addr    (32'h8040_0000),
      .wbs1_addr_msk(32'hFFC0_0000),

      .wbs1_adr_o(wbs1_adr_o),
      .wbs1_dat_i(wbs1_dat_i),
      .wbs1_dat_o(wbs1_dat_o),
      .wbs1_we_o (wbs1_we_o),
      .wbs1_sel_o(wbs1_sel_o),
      .wbs1_stb_o(wbs1_stb_o),
      .wbs1_ack_i(wbs1_ack_i),
      .wbs1_err_i('0),
      .wbs1_rty_i('0),
      .wbs1_cyc_o(wbs1_cyc_o),

      // Slave interface 2 (to UART controller)
      // Address range: 0x1000_0000 ~ 0x1000_FFFF
      .wbs2_addr    (32'h1000_0000),
      .wbs2_addr_msk(32'hFFFF_0000),

      .wbs2_adr_o(wbs2_adr_o),
      .wbs2_dat_i(wbs2_dat_i),
      .wbs2_dat_o(wbs2_dat_o),
      .wbs2_we_o (wbs2_we_o),
      .wbs2_sel_o(wbs2_sel_o),
      .wbs2_stb_o(wbs2_stb_o),
      .wbs2_ack_i(wbs2_ack_i),
      .wbs2_err_i('0),
      .wbs2_rty_i('0),
      .wbs2_cyc_o(wbs2_cyc_o)
  );
  /* =========== MUX end =========== */
  /* =========== Slaves begin =========== */
  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_base (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs0_cyc_o),
      .wb_stb_i(wbs0_stb_o),
      .wb_ack_o(wbs0_ack_i),
      .wb_adr_i(wbs0_adr_o),
      .wb_dat_i(wbs0_dat_o),
      .wb_dat_o(wbs0_dat_i),
      .wb_sel_i(wbs0_sel_o),
      .wb_we_i (wbs0_we_o),

      // To SRAM chip
      .sram_addr(base_ram_addr),
      .sram_data(base_ram_data),
      .sram_ce_n(base_ram_ce_n),
      .sram_oe_n(base_ram_oe_n),
      .sram_we_n(base_ram_we_n),
      .sram_be_n(base_ram_be_n)
  );

  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_ext (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs1_cyc_o),
      .wb_stb_i(wbs1_stb_o),
      .wb_ack_o(wbs1_ack_i),
      .wb_adr_i(wbs1_adr_o),
      .wb_dat_i(wbs1_dat_o),
      .wb_dat_o(wbs1_dat_i),
      .wb_sel_i(wbs1_sel_o),
      .wb_we_i (wbs1_we_o),

      // To SRAM chip
      .sram_addr(ext_ram_addr),
      .sram_data(ext_ram_data),
      .sram_ce_n(ext_ram_ce_n),
      .sram_oe_n(ext_ram_oe_n),
      .sram_we_n(ext_ram_we_n),
      .sram_be_n(ext_ram_be_n)
  );

  // 串口控制器模块
  // NOTE: 如果修改系统时钟频率，也需要修改此处的时钟频率参数
  uart_controller #(
      .CLK_FREQ(50_000_000),
      .BAUD    (115200)
  ) uart_controller (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      .wb_cyc_i(wbs2_cyc_o),
      .wb_stb_i(wbs2_stb_o),
      .wb_ack_o(wbs2_ack_i),
      .wb_adr_i(wbs2_adr_o),
      .wb_dat_i(wbs2_dat_o),
      .wb_dat_o(wbs2_dat_i),
      .wb_sel_i(wbs2_sel_o),
      .wb_we_i (wbs2_we_o),

      // to UART pins
      .uart_txd_o(txd),
      .uart_rxd_i(rxd)
  );
  /* =========== Slaves end =========== */
endmodule
