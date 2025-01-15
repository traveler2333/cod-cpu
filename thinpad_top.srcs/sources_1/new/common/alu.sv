module alu (
    input  wire [31:0] a,      // ALU ???? A
    input  wire [31:0] b,      // ALU ???? B
    input  wire [3:0]  op, // ALU ??????
    output reg [31:0] y       // ALU ???
);

    always_comb begin
        case (op)
            4'b0001: y = a + b;           // ADD
            4'b0010: y = a - b;           // SUB
            4'b0011: y = a & b;           // AND
            4'b0100: y = a | b;           // OR
            4'b0101: y = a ^ b;           // XOR
            4'b0110: y = ~a;              // NOT
            4'b0111: y = a << b[4:0];          // SLL
            4'b1000: y = a >> b[4:0];          // SRL
            4'b1001: y = $signed(a) >>> b[4:0]; // SRA
            4'b1010: y = (a << b[4:0]) | (a >> (32- b[4:0])); // ROL
            default: y = 32'b0;           // NOP
        endcase
    end

endmodule