//功能：FIFO突发读模块
module burst_read(
       input                clk,
		 input              rst_n,
		 input [DEPTH-1:0]  fifo_rdusedw,
		 input [DWIDTH-1:0] fifo_q,
		 output             fifo_rd,
		 output             ovalid,
		 output[DWIDTH-1:0] odata
);
parameter  WIDTH  = 1280;//视频宽度
parameter  DWIDTH = 8;
localparam DEPTH = 13;
localparam BURST = 1280;//突发读的长度等于视频宽度
localparam IDLE  = 1'b0;
localparam RD_S  = 1'b1;
reg        state_c,state_n;
reg [DEPTH-1:0] rd_cnt;
reg        fifo_rd_r;
reg        ovalid_r;
assign ovalid  = ovalid_r;
assign fifo_rd = fifo_rd_r;
assign odata   = fifo_q;
always@(posedge clk or negedge rst_n)
if(rst_n == 1'b0)
    state_c <= IDLE;
else
    state_c <= state_n;
always@(*)
begin
case(state_c)
IDLE:state_n = (fifo_rdusedw >= BURST)?RD_S:IDLE;
RD_S:state_n = (rd_cnt >= BURST-1)?IDLE:RD_S;
endcase
end
always@(posedge clk or negedge rst_n)
begin
if(rst_n == 1'b0)
    begin
	 rd_cnt    <= 'd0;
	 fifo_rd_r <= 1'b0;
	 ovalid_r  <= 1'b0;
	 end
else
    begin
	 ovalid_r <= fifo_rd_r;
	 case(state_c)
	 IDLE:begin
	     rd_cnt    <= 'd0;
		  fifo_rd_r <= 1'b0;
	 end
	 RD_S:begin
	     rd_cnt <= rd_cnt+1;
		  fifo_rd_r <= (rd_cnt <= BURST-1);
	 end
	 endcase
	 end
end
endmodule 