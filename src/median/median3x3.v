module median3x3 #(parameter WIDTH=1280,HEIGHT=720,FILTER_ENABLE=1)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 output source_valid,source_sop,source_eop,
 output [7:0] source_data,
 output [7:0] source_gaussian
);
 wire v,s,e;
 wire [71:0] p;
 Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
   .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),
   .sink_eop(sink_eop),.sink_data(sink_data),
   .matrix_valid(v),.matrix_sop(s),.matrix_eop(e),
   .matrix_p11(p[71:64]),.matrix_p12(p[63:56]),.matrix_p13(p[55:48]),
   .matrix_p21(p[47:40]),.matrix_p22(p[39:32]),.matrix_p23(p[31:24]),
   .matrix_p31(p[23:16]),.matrix_p32(p[15:8]),.matrix_p33(p[7:0]));
 median9_pipeline #(.FILTER_ENABLE(FILTER_ENABLE)) sorter(
   .clk(clk),.rst_n(rst_n),.sink_valid(v),.sink_sop(s),.sink_eop(e),
   .sink_pixels(p),.source_valid(source_valid),.source_sop(source_sop),
   .source_eop(source_eop),.source_data(source_data),.source_gaussian(source_gaussian));
endmodule
