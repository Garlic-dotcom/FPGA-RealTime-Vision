module cmos_top( 
		 input                    clk,
		 input                    rst_n,
		 input                    cmos_pclk,
		 input                    cmos_vsync,
		 input                    cmos_href,
		 input [DWIDTH-1:0]       cmos_data,
		 output                   source_sop,
		 output                   source_valid,
		 output [2*DWIDTH-1:0]    source_data,
		 output                   source_eop,
		 input                    source_ready
);
parameter WIDTH  = 1280;
parameter HEIGHT = 720;
parameter DWIDTH = 8;
localparam DEPTH = 13;//$clog2(WIDTH<<1);

wire              dcfifo_wr_w,aclr_w,aclr_w0;
// OV5640 0x4740=0x21: VSYNC is HIGH during the valid frame, LOW in blanking.
// aclr_w0 is the registered VSYNC (despite the historical "aclr" name).
// Reset byte pairing and packet count ONLY during vertical blanking. Using
// aclr_w0 directly would hold capture in reset throughout every active frame.
assign aclr_w = ~rst_n | ~aclr_w0;
wire [DWIDTH-1:0] dcfifo_data_w;
cmos2dcfifo #(.DWIDTH(DWIDTH))
u_cmos2dcfifo_0(
       .cmos_pclk(cmos_pclk),
		 .rst_n(rst_n),
		 .cmos_vsync(cmos_vsync),
		 .cmos_href(cmos_href),
		 .cmos_data(cmos_data),
		 .dcfifo_wr(dcfifo_wr_w),
		 .dcfifo_data(dcfifo_data_w),
		 .dcfifo_aclr(aclr_w0)
);
/*
wire              dcfifo_rd_w;
wire [DWIDTH-1:0] dcfifo_q_w;
wire [DEPTH-1:0]  dcfifo_rdusedw_w;
//
reg aclr_r0,aclr_r1;
always@(posedge clk)
begin
aclr_r0 <= 0;
aclr_r1 <= aclr_r0;
end
//
assign aclr_w = ~rst_n;//系统复位 OR 帧清空(vsync期间拉高)
wire dc_busy;
cmos_dcfifo u_cmos_dcfifo_0 (
	.a_rst_i(aclr_w),
	.wdata(dcfifo_data_w),
	.rd_clk_i(clk),
	.rd_en_i(dcfifo_rd_w),
	.wr_clk_i(cmos_pclk),
	.wr_en_i(dcfifo_wr_w),
	.rdata(dcfifo_q_w),
	.rd_datacount_o(dcfifo_rdusedw_w),
	.wr_datacount_o(),
	.full_o(),
	.empty_o(),
	.rst_busy(dc_busy));
burst_read #(.WIDTH(WIDTH),
             .DWIDTH(DWIDTH))
u_burst_read_0(
       .clk(clk),
		 .rst_n(rst_n && !dc_busy),
		 .fifo_rdusedw(dcfifo_rdusedw_w),
		 .fifo_q(dcfifo_q_w),
		 .fifo_rd(dcfifo_rd_w),
		 .ovalid(scfifo_wr_w),
		 .odata(scfifo_data_w)
);
*/
wire [DWIDTH-1:0] scfifo_data_w;
wire              scfifo_wr_w;
wire [(DWIDTH<<1)-1:0] scfifo_q_w;
wire                   scfifo_rd_w,scfifo_empty_w;
cmos_scfifo u_cmos_scfifo_0 (
	.a_rst_i(aclr_w),
	.wdata(dcfifo_data_w),
	.clk_i(clk),
	.wr_en_i(dcfifo_wr_w),
	.rd_en_i(scfifo_rd_w),
	.rdata(scfifo_q_w),
	.empty_o(scfifo_empty_w),
	.full_o(),
	.rst_busy(),
	.wr_datacount_o(),
	.rd_datacount_o()
	);
cmos_fifo2st #(.DWIDTH(DWIDTH<<1),
               .WIDTH(WIDTH),
					.HEIGHT(HEIGHT))
u_cmos_fifo2st_0
            (.clk(clk),              //本模块将FIFO数据转化成非标Avalon-ST视频流，即不带控制包头，并且无视频数据包头，sop对应一帧的第一个有效数据,eop对应一帧的最后一个有效数据
             .rst_n(rst_n),
             .fifo_aclr(aclr_w),        //外部模块需要根据视频流特征生成扇出FIFO的异步aclr信号
             .fifo_empty(scfifo_empty_w),       //FIFO空标志信号
             .fifo_q({scfifo_q_w[7:0],scfifo_q_w[15:8]}),
             .fifo_rd(scfifo_rd_w),
			    .one_frame_done(),
             .source_sop(source_sop),
             .source_valid(source_valid),
             .source_data(source_data),
             .source_eop(source_eop),
             .source_ready(source_ready)
               );
endmodule 
