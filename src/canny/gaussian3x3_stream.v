// Integer Gaussian [1 2 1; 2 4 2; 1 2 1]/16 with floor rounding.
module gaussian3x3_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 output source_valid,source_sop,source_eop,
 output [7:0] source_data
);
 wire wv,ws,we;
 wire [7:0] a,b,c,d,e,f,g,h,i;
 Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
 .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_data),
 .matrix_valid(wv),.matrix_sop(ws),.matrix_eop(we),
 .matrix_p11(a),.matrix_p12(b),.matrix_p13(c),.matrix_p21(d),.matrix_p22(e),.matrix_p23(f),
 .matrix_p31(g),.matrix_p32(h),.matrix_p33(i));
 reg [9:0] row0,row1,row2;
 reg [11:0] sum;
 reg [1:0] valid_pipe,sop_pipe,eop_pipe;
 assign source_valid=valid_pipe[1];assign source_sop=sop_pipe[1];assign source_eop=eop_pipe[1];
 assign source_data=sum[11:4];
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin row0<=0;row1<=0;row2<=0;sum<=0;valid_pipe<=0;sop_pipe<=0;eop_pipe<=0;end
   else begin
     row0<={2'b0,a}+{1'b0,b,1'b0}+{2'b0,c};
     row1<={2'b0,d}+{1'b0,e,1'b0}+{2'b0,f};
     row2<={2'b0,g}+{1'b0,h,1'b0}+{2'b0,i};
     sum<={2'b0,row0}+{1'b0,row1,1'b0}+{2'b0,row2};
     valid_pipe<={valid_pipe[0],wv};sop_pipe<={sop_pipe[0],ws};eop_pipe<={eop_pipe[0],we};
   end
 end
endmodule
