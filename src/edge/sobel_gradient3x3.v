// Three arithmetic stages after a valid 3x3 window. Full precision Sobel:
// gx=right-left, gy=bottom-top, magnitude=abs(gx)+abs(gy).
// Each signed derivative is 11 bits; the unsigned L1 sum is 11 bits.
module sobel_gradient3x3(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [71:0] pixels,
 output source_valid,source_sop,source_eop,
 output reg signed [10:0] source_gx,source_gy,
 output reg [10:0] source_magnitude,
 output [7:0] source_center
);
 wire [7:0] p11=pixels[71:64],p12=pixels[63:56],p13=pixels[55:48];
 wire [7:0] p21=pixels[47:40],p22=pixels[39:32],p23=pixels[31:24];
 wire [7:0] p31=pixels[23:16],p32=pixels[15:8],p33=pixels[7:0];
 reg [9:0] left_sum,right_sum,top_sum,bottom_sum;
 reg signed [10:0] gx,gy;
 wire [10:0] abs_x=gx[10] ? -gx : gx;
 wire [10:0] abs_y=gy[10] ? -gy : gy;
 reg [2:0] valid_pipe,sop_pipe,eop_pipe;
 reg [23:0] center_pipe;
 assign source_valid=valid_pipe[2];
 assign source_sop=sop_pipe[2];
 assign source_eop=eop_pipe[2];
 assign source_center=center_pipe[23:16];
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     left_sum<=0;right_sum<=0;top_sum<=0;bottom_sum<=0;
     gx<=0;gy<=0;source_gx<=0;source_gy<=0;source_magnitude<=0;
     valid_pipe<=0;sop_pipe<=0;eop_pipe<=0;center_pipe<=0;
   end else begin
     left_sum<={2'b00,p11}+{1'b0,p21,1'b0}+{2'b00,p31};
     right_sum<={2'b00,p13}+{1'b0,p23,1'b0}+{2'b00,p33};
     top_sum<={2'b00,p11}+{1'b0,p12,1'b0}+{2'b00,p13};
     bottom_sum<={2'b00,p31}+{1'b0,p32,1'b0}+{2'b00,p33};
     gx<=$signed({1'b0,right_sum})-$signed({1'b0,left_sum});
     gy<=$signed({1'b0,bottom_sum})-$signed({1'b0,top_sum});
     source_gx<=gx;source_gy<=gy;source_magnitude<=abs_x+abs_y;
     valid_pipe<={valid_pipe[1:0],sink_valid};
     sop_pipe<={sop_pipe[1:0],sink_sop && sink_valid};
     eop_pipe<={eop_pipe[1:0],sink_eop && sink_valid};
     center_pipe<={center_pipe[15:0],p22};
   end
 end
endmodule
