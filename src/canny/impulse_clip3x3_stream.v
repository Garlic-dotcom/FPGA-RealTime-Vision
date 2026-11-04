// Structure-preserving isolated-range clamp before the Canny Gaussian.
// Only a center strictly outside the eight-neighbor range by >24 is replaced.
// Replicated boundaries, one output per pixel, and four pipelined tree stages.
module impulse_clip3x3_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 output source_valid,source_sop,source_eop,
 output reg [7:0] source_data
);
 wire wv,ws,we;
 wire [7:0] a,b,c,d,e,f,g,h,i;
 Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
 .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_data),
 .matrix_valid(wv),.matrix_sop(ws),.matrix_eop(we),
 .matrix_p11(a),.matrix_p12(b),.matrix_p13(c),.matrix_p21(d),.matrix_p22(e),.matrix_p23(f),
 .matrix_p31(g),.matrix_p32(h),.matrix_p33(i));
 function [7:0] min8;
   input [7:0] x,y;
   begin min8=(x<y)?x:y;end
 endfunction
 function [7:0] max8;
   input [7:0] x,y;
   begin max8=(x>y)?x:y;end
 endfunction
 reg [7:0] lo10,lo11,lo12,lo13,hi10,hi11,hi12,hi13,center1;
 reg [7:0] lo20,lo21,hi20,hi21,center2;
 reg [7:0] minimum3,maximum3,center3;
 // These aligned internal observations have no consumer in synthesized logic.
 // Simulation checks them independently against the raw replicated window.
 reg [7:0] source_minimum,source_maximum,source_center;
 reg [3:0] valid_pipe,sop_pipe,eop_pipe;
 assign source_valid=valid_pipe[3];
 assign source_sop=sop_pipe[3];
 assign source_eop=eop_pipe[3];
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     lo10<=0;lo11<=0;lo12<=0;lo13<=0;hi10<=0;hi11<=0;hi12<=0;hi13<=0;center1<=0;
     lo20<=0;lo21<=0;hi20<=0;hi21<=0;center2<=0;
     minimum3<=0;maximum3<=0;center3<=0;
     source_data<=0;source_minimum<=0;source_maximum<=0;source_center<=0;
     valid_pipe<=0;sop_pipe<=0;eop_pipe<=0;
   end else begin
     lo10<=min8(a,b);lo11<=min8(c,d);lo12<=min8(f,g);lo13<=min8(h,i);
     hi10<=max8(a,b);hi11<=max8(c,d);hi12<=max8(f,g);hi13<=max8(h,i);center1<=e;
     lo20<=min8(lo10,lo11);lo21<=min8(lo12,lo13);
     hi20<=max8(hi10,hi11);hi21<=max8(hi12,hi13);center2<=center1;
     minimum3<=min8(lo20,lo21);maximum3<=max8(hi20,hi21);center3<=center2;
     // Nine-bit comparisons keep 255+24 and 0-24 from wrapping.
     source_data<=({1'b0,center3}>{1'b0,maximum3}+9'd24)?maximum3:
                  ({1'b0,center3}+9'd24<{1'b0,minimum3})?minimum3:center3;
     source_minimum<=minimum3;source_maximum<=maximum3;source_center<=center3;
     valid_pipe<={valid_pipe[2:0],wv};sop_pipe<={sop_pipe[2:0],ws};eop_pipe<={eop_pipe[2:0],we};
   end
 end
endmodule
