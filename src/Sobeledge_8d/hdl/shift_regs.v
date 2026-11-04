//元件功能：可配置延迟周期数和数据位宽的移位寄存器，主要用作延迟
module shift_regs#(
parameter DWIDTH     = 3,//数据位宽
parameter DELAY_DUTY = 4//延迟的时钟周期数，最小是1
)(
       input               clk,
		 input               rst_n,
		 input [DWIDTH-1:0]  idata,
		 output [DWIDTH-1:0] odata
);

reg [DWIDTH-1:0] idata_r[DELAY_DUTY-1:0];
genvar i;
generate
for(i=0; i<DELAY_DUTY-1; i=i+1)
    begin:delay_data
	 always@(posedge clk or negedge rst_n)
	     if(!rst_n)
		      idata_r[i+1] <= 'd0;
		  else
				idata_r[i+1] <= idata_r[i];
	 end
endgenerate
always@(posedge clk or negedge rst_n)
if(!rst_n)
    idata_r[0] <= 'd0;
else
    idata_r[0] <= idata;
	 
assign odata = idata_r[DELAY_DUTY-1];
endmodule 