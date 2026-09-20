
module Register 
#(
    parameter WIDTH = 8,
    parameter RESET = 0
)
(
    input               i_clk,
    input               i_reset,

    input               i_we,
    input [WIDTH-1:0]   i_data_in,
    output [WIDTH-1:0]  o_data_out
);

reg [WIDTH-1:0]  data;
reg [WIDTH-1:0]  data_next;
assign o_data_out = data;

always @(posedge i_clk) begin
    data <= data_next;
end

always @(*) begin
    data_next = data;

    if (i_reset) begin
        data_next = RESET;
    end else begin
        if (i_we) begin
            data_next = i_data_in;
        end

    end
end

endmodule // Register
