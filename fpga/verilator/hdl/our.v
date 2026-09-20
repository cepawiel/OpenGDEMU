module our(
    input  wire i_reset,
    input  wire i_clk,
    output wire c
);
    assign c = i_reset | i_clk;
    initial begin $display("Hello World"); $finish; end
endmodule
