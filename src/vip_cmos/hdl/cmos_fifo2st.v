module cmos_fifo2st(input               clk,              //本模块将FIFO数据转化成非标Avalon-ST视频流，即不带控制包头，并且无视频数据包头，sop对应一帧的第一个有效数据,eop对应一帧的最后一个有效数据
               input               rst_n,
               input               fifo_aclr,        //外部模块需要根据视频流特征生成扇出FIFO的异步aclr信号
               input               fifo_empty,       //FIFO空标志信号
               input [DWIDTH-1:0]  fifo_q,
               output              fifo_rd,
					output              one_frame_done,
               output              source_sop,
               output              source_valid,
               output [DWIDTH-1:0] source_data,
               output              source_eop,
               input               source_ready
               );
parameter DWIDTH        = 16;
parameter WIDTH         = 1280;
parameter HEIGHT        = 720;
localparam TOTAL_PIXELS = WIDTH*HEIGHT;
reg [31:0] pix_cnt;
reg        source_sop_r,source_valid_r,source_eop_r;
reg        one_frame_done_r;
reg        frame_complete;
assign one_frame_done = one_frame_done_r;
// A physical VSYNC clears fifo_aclr before each frame. Do not turn excess
// pixels into a new logical frame while downstream windows flush their tail.
assign fifo_rd      = (~fifo_empty) && source_ready && !frame_complete;
assign source_data  = fifo_q;
assign source_sop   = source_sop_r;
assign source_valid = source_valid_r;
assign source_eop   = source_eop_r;
always@(posedge clk)
begin
if(!rst_n || fifo_aclr)
    begin
    source_sop_r     <= 1'b0;
    source_valid_r   <= 1'b0;
    source_eop_r     <= 1'b0;
	 pix_cnt          <= 0;
	 one_frame_done_r <= 1'b0;
	 frame_complete   <= 1'b0;
    end
else
    begin
    source_valid_r <= fifo_rd;
    if(fifo_rd)
        begin
        if(pix_cnt >= TOTAL_PIXELS-1)
            frame_complete <= 1'b1;
        else
            pix_cnt <= pix_cnt+1;
        end
    else
        begin
        pix_cnt <= pix_cnt;
        end
    source_sop_r <= (pix_cnt == 0) && fifo_rd;
    source_eop_r <= (pix_cnt >= TOTAL_PIXELS-1) && fifo_rd;
	 one_frame_done_r <= source_eop_r;
    end

end
endmodule 
