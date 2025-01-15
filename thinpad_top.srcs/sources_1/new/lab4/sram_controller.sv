module sram_controller #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,

    parameter SRAM_ADDR_WIDTH = 20,
    parameter SRAM_DATA_WIDTH = 32,

    localparam SRAM_BYTES = SRAM_DATA_WIDTH / 8,
    localparam SRAM_BYTE_WIDTH = $clog2(SRAM_BYTES)
) (
    // clk and reset
    input wire clk_i,
    input wire rst_i,

    // wishbone slave interface
    input wire wb_cyc_i, // wishbone 时钟
    input wire wb_stb_i, // wishbone 使能
    output reg wb_ack_o, // wishbone 应答
    input wire [ADDR_WIDTH-1:0] wb_adr_i, // wishbone 地址
    input wire [DATA_WIDTH-1:0] wb_dat_i, // wishbone 数据
    output reg [DATA_WIDTH-1:0] wb_dat_o, // wishbone 数据输出
    input wire [DATA_WIDTH/8-1:0] wb_sel_i, // wishbone 字节选择
    input wire wb_we_i, // wishbone 写使能

    // sram interface
    output reg [SRAM_ADDR_WIDTH-1:0] sram_addr, // SRAM 地址
    inout wire [SRAM_DATA_WIDTH-1:0] sram_data, // SRAM 数据
    output reg sram_ce_n, // SRAM 片选，低有效
    output reg sram_oe_n, // SRAM 输出使能，低有效
    output reg sram_we_n, // SRAM 写使能，低有效
    output reg [SRAM_BYTES-1:0] sram_be_n // SRAM 字节使能, 低有效。如果不使用字节使能，请保持为0
);

    // 数据总线方向控制
    wire [31:0] sram_data_i_comb;
    reg [31:0] sram_data_o_reg;

    assign sram_data = (!wb_we_i) ? 32'bz : sram_data_o_reg; // 读操作时，数据总线为高阻态
    assign sram_data_i_comb = sram_data;

    // 定义状态
    typedef enum logic [1:0] {
        STATE_IDLE = 2'b00,
        STATE_READ = 2'b01,
        STATE_WRITE = 2'b10,
        STATE_WRITE_2 = 2'b11
    } state_t;
    state_t state;
    // 初始化
    always_comb begin
        sram_ce_n = ~ (wb_cyc_i && wb_stb_i);
        sram_oe_n = wb_we_i; //当写使能时，禁止读
        sram_be_n = ~ wb_sel_i;
        sram_addr = wb_adr_i[ADDR_WIDTH-1:2];
        sram_data_o_reg = (wb_we_i)? wb_dat_i : 32'b0;
        wb_dat_o = sram_data_i_comb;
    end

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            wb_ack_o <= 1'b0;
            state <= STATE_IDLE;
            sram_we_n <= 1'b1;
        end else begin
            case(state)
                STATE_IDLE: begin
                    if (wb_cyc_i && wb_stb_i) begin
                        if (wb_we_i) begin
                            sram_we_n <= 1'b0; // 写操作使能
                            state <= STATE_WRITE;
                        end else begin
                            wb_ack_o <= 1'b1; // 读操作立即应答
                            state <= STATE_READ;
                        end
                    end
                end
                STATE_READ: begin      
                    wb_ack_o <= 1'b0;
                    state <= STATE_IDLE;
                end
                STATE_WRITE: begin
                    sram_we_n <= 1'b1;
                    wb_ack_o <= 1'b1;
                    state <= STATE_WRITE_2;
                end
                STATE_WRITE_2: begin
                    sram_we_n <= 1'b1;
                    wb_ack_o <= 1'b0;
                    state <= STATE_IDLE;
                end
                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule