module IF_IMM(   // 取指令模块
    input wire clk,        
    input wire rst,                
    input wire [31:0] pc,                   

    // 给mmu
    output reg [31:0] wb_adr_o,       // 
    output reg [31:0] wb_dat_o,       // 
    output reg wb_we_o,               //    
    output reg [3:0] wb_sel_o,        //   
    output reg wb_stb_o,              // 
    output reg wb_cyc_o,              //   

    // 从mmu得到的数据
    input wire [31:0] wb_dat_i,       //  
    input wire wb_ack_i,              //  
    input wire [3:0] exception_code,  
    input wire page_fault,            

    // IF_ID  
    output reg [31:0] instruction,    //  
    output reg if_stall, //  ack 
    //把exception_code和page_fault传给下一个模块
    output reg [3:0] exception_code_o,
    output reg page_fault_o,

    // 与 InstructionCache 模块交互
    output reg [31:0] icache_addr,
    output reg icache_read_en,
    input wire [31:0] icache_data,
    input wire icache_hit,
    //传给cache的数据用来更新
    output reg [31:0] icache_data_update,
    output reg icache_write_en,

    output reg if_mmu_sel
); 

assign if_stall = ~ (icache_hit | wb_ack_i);
always_ff @(posedge clk) begin
    if(rst) begin
        wb_cyc_o <= 1'b0;
        wb_stb_o <= 1'b0;
    end else begin
        if(icache_hit) begin
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
        end else if (wb_ack_i) begin
            wb_cyc_o <= 1'b0;
            wb_stb_o <= 1'b0;
        end else begin
            wb_cyc_o <= 1'b1;
            wb_stb_o <= 1'b1;
        end
    end
end

always_comb begin
    if_mmu_sel = 1'b0;
    icache_addr = pc;
    instruction = 32'h0;
    icache_data_update = 0;
    icache_write_en = 1'b0;
    icache_read_en = 1'b1;
    exception_code_o = exception_code;
    page_fault_o = page_fault;

    if(icache_hit&&!wb_ack_i) begin
        instruction = icache_data;
    end
    if (wb_ack_i) begin
        instruction = wb_dat_i;
        icache_data_update = wb_dat_i;
        icache_write_en = 1'b1;
    end
    wb_adr_o = pc;
    wb_dat_o = 32'b0;
    wb_we_o = 1'b0;
    wb_sel_o = 4'b1111;
end

endmodule