// Centered, full-frame Sobel with replicated borders. The threshold is sampled
// on the first gradient of each output frame, so a frame cannot mix thresholds.
module sobel_l1_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 input [10:0] threshold,
 output reg source_valid,source_sop,source_eop,source_edge,
 output reg [10:0] source_magnitude,
 output reg [7:0] source_center,
 output reg [10:0] active_threshold,
 output reg [1:0] source_direction,
 output [10:0] source_compare_threshold
);
 wire wv,ws,we,gv,gs,ge;
 wire [71:0] p;
 wire [10:0] magnitude;
 wire signed [10:0] gx,gy;
 wire [1:0] direction;
 gradient_direction direction_bin(.gx(gx),.gy(gy),.direction(direction));
 wire [7:0] center;
 // User sensitivity scale: keep the T160 anchor and both endpoints, reject
 // more low-amplitude noise at T32, and compress the overly steep high range.
 // active_threshold/OSD retain the user's original 11-bit adjustment value.
 function [10:0] quality_threshold;
   input [10:0] user_threshold;
   reg [11:0] value,delta;
   begin
     if(user_threshold<=11'd32) value={1'b0,user_threshold}+({1'b0,user_threshold}>>1);
     else if(user_threshold<11'd160) begin
       delta=12'd160-{1'b0,user_threshold};
       value={1'b0,user_threshold}+(delta>>3);
     end else if(user_threshold<=11'd320) begin
       delta={1'b0,user_threshold}-12'd160;
       value=12'd160+(delta>>1);
     end else begin
       delta={1'b0,user_threshold}-12'd320;
       value={1'b0,user_threshold}-12'd80+(delta>>4);
     end
     quality_threshold=(value>12'd2047)?11'd2047:value[10:0];
   end
 endfunction
 assign source_compare_threshold=quality_threshold(active_threshold);
 wire [10:0] compare_threshold=quality_threshold(gs ? threshold : active_threshold);
 Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
   .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),
   .sink_eop(sink_eop),.sink_data(sink_data),
   .matrix_valid(wv),.matrix_sop(ws),.matrix_eop(we),
   .matrix_p11(p[71:64]),.matrix_p12(p[63:56]),.matrix_p13(p[55:48]),
   .matrix_p21(p[47:40]),.matrix_p22(p[39:32]),.matrix_p23(p[31:24]),
   .matrix_p31(p[23:16]),.matrix_p32(p[15:8]),.matrix_p33(p[7:0]));
 sobel_gradient3x3 gradient(.clk(clk),.rst_n(rst_n),
   .sink_valid(wv),.sink_sop(ws),.sink_eop(we),.pixels(p),
   .source_valid(gv),.source_sop(gs),.source_eop(ge),
   .source_gx(gx),.source_gy(gy),.source_magnitude(magnitude),.source_center(center));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     source_valid<=0;source_sop<=0;source_eop<=0;source_edge<=0;
     source_magnitude<=0;source_center<=0;active_threshold<=0;source_direction<=0;
   end else begin
     source_valid<=gv;source_sop<=gs;source_eop<=ge;
     source_magnitude<=magnitude;source_center<=center;
     source_direction<=direction;
     source_edge<=(magnitude>compare_threshold);
     if(gv && gs) active_threshold<=threshold;
   end
 end
endmodule
