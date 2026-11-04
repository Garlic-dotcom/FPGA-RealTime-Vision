module cmos2dcfifo(
       input               cmos_pclk,
		 input               rst_n,
		 input               cmos_vsync,
		 input               cmos_href,
		 input [DWIDTH-1:0]  cmos_data,
		 output              dcfifo_wr,
		 output [DWIDTH-1:0] dcfifo_data,
		 output              dcfifo_aclr
);
parameter DWIDTH = 8;
localparam ABNDN = 20;//上电后丢弃的帧数
reg              cmos_vsync_r;//寄存一拍，这样编译器会自动使用IO高速buf来缓存一下数据，改善时序质量
reg              cmos_href_r;//寄存一拍，这样编译器会自动使用IO高速buf来缓存一下数据，改善时序质量
reg [DWIDTH-1:0] cmos_data_r;//寄存一拍，这样编译器会自动使用IO高速buf来缓存一下数据，改善时序质量
reg              dcfifo_wr_r;
reg [DWIDTH-1:0] dcfifo_data_r;
reg              dcfifo_aclr_r;//dcfifo的异步复位信号，高有效
reg [7:0]        invalid_frame_cnt;//上电之后抛弃一些无效帧再往dcfifo里写数据
reg              valid_frame_flag;//上电之后抛弃一些无效帧后此信号拉高
reg              cmos_vsync_r0,cmos_vsync_r1;//用以边沿检测cmos_vsync_r上升沿
reg [11:0]       x_cnt,y_cnt;
wire             cmos_vsync_r_rise;//cmos_vsync_r上升沿标志
assign cmos_vsync_r_rise = cmos_vsync_r0 && !cmos_vsync_r1;
assign dcfifo_wr   = dcfifo_wr_r;
assign dcfifo_data = dcfifo_data_r;
assign dcfifo_aclr = dcfifo_aclr_r;
always@(posedge cmos_pclk or negedge rst_n)
begin
if(rst_n == 1'b0)
    begin
	 cmos_vsync_r      <= 1'b0;
	 cmos_href_r       <= 1'b0;
	 cmos_data_r       <= {DWIDTH{1'b0}};
	 dcfifo_wr_r       <= 1'b0;
	 dcfifo_data_r     <= 'd0;//自动适应位宽
	 dcfifo_aclr_r     <= 1'b0;
	 invalid_frame_cnt <= 8'd0;
	 valid_frame_flag  <= 1'b0;
	 cmos_vsync_r0     <= 1'b0;
	 cmos_vsync_r1     <= 1'b0;
	 end
else
    begin
	 cmos_data_r       <= cmos_data;
	 cmos_href_r       <= cmos_href;
	 cmos_vsync_r      <= cmos_vsync;
	 cmos_vsync_r0     <= cmos_vsync_r;//边沿检测,这里的延迟不影响结果，因vsync一般都较长且vsync和第一个href之间间隔时钟数较多
	 cmos_vsync_r1     <= cmos_vsync_r0;//边沿检测
	 invalid_frame_cnt <= cmos_vsync_r_rise?((invalid_frame_cnt >= ABNDN)?invalid_frame_cnt:invalid_frame_cnt+8'd1):invalid_frame_cnt;//到达ABNDN后停止自增
	 valid_frame_flag  <= (invalid_frame_cnt >= ABNDN)?1'b1:valid_frame_flag;
	 dcfifo_wr_r       <= valid_frame_flag && cmos_href_r;
	 dcfifo_data_r     <= cmos_data_r;
	 dcfifo_aclr_r     <= cmos_vsync_r;//利用vsync_r进行dcfifo的清空
	 end
end
endmodule 
