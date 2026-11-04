module canny_gradient_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 output source_valid,source_sop,source_eop,
 output [12:0] source_data
);
 wire wv,ws,we;
 wire [71:0] p;
 wire signed [10:0] gx,gy;
 wire [10:0] magnitude;
 wire [1:0] direction;
 Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
 .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_data),
 .matrix_valid(wv),.matrix_sop(ws),.matrix_eop(we),
 .matrix_p11(p[71:64]),.matrix_p12(p[63:56]),.matrix_p13(p[55:48]),
 .matrix_p21(p[47:40]),.matrix_p22(p[39:32]),.matrix_p23(p[31:24]),
 .matrix_p31(p[23:16]),.matrix_p32(p[15:8]),.matrix_p33(p[7:0]));
 sobel_gradient3x3 gradient(.clk(clk),.rst_n(rst_n),.sink_valid(wv),.sink_sop(ws),.sink_eop(we),.pixels(p),
 .source_valid(source_valid),.source_sop(source_sop),.source_eop(source_eop),
 .source_gx(gx),.source_gy(gy),.source_magnitude(magnitude),.source_center());
 gradient_direction quantize(.gx(gx),.gy(gy),.direction(direction));
 assign source_data={magnitude,direction};
endmodule
