//File name  :
//Module name:
//Author     :hanjunma111
//Email      :865021481@qq.com
//Version    :V1.0
//Abstract   :RGB转灰度，默认颜色通道为并行3通道，BPS可参数化设置
// Y = (R*306 + G*601 + B*117)/1024
//R:sink_data[3*BPS-1 : 2*BPS]
//G:sink_data[2*BPS-1 : BPS]
//B:sink_data[BPS-1   : 0]
//总延迟：4个clk 乘法器4个、加法2个
module vip_rgb2gray_proc #(
parameter BPS = 8
)(
    input              clk,
	input              rst_n,
    input              sink_sop,
    input              sink_valid,
    input  [3*BPS-1:0] sink_data,
    input              sink_eop,
    output             source_sop,
    output             source_valid,
    output [BPS-1:0]   source_data,
    output             source_eop
);
wire [BPS+9:0] y_r_w,y_g_w,y_b_w;
reg [BPS+9:0] add_r0,add_r1,add_r2;//y_r_w+y_g_w , y_b_w , add_r0+add_r1
assign source_data = add_r2[BPS+9:10];//截尾低10bits，因为系数是先乘以了1024的
vip_rgb2gray_mult mult_r (
    .clock(clk),
    .dataa(sink_data[3*BPS-1:2*BPS]),
    .datab(10'd306),
    .result(y_r_w)
);
vip_rgb2gray_mult mult_g (
    .clock(clk),
    .dataa(sink_data[2*BPS-1:BPS]),
    .datab(10'd601),
    .result(y_g_w)
);
vip_rgb2gray_mult mult_b (
    .clock(clk),
    .dataa(sink_data[BPS-1:0]),
    .datab(10'd117),
    .result(y_b_w)
);
always@(posedge clk or negedge rst_n)
begin
if(rst_n == 1'b0)
    begin
    add_r0 <= 'd0;
    add_r1 <= 'd0;
    add_r2 <= 'd0;
    end
else
    begin
    add_r0 <= y_r_w+y_g_w;//r+g先求和
    add_r1 <= y_b_w;//延迟一个时钟
    add_r2 <= add_r0+add_r1;
    end
end
vip_rgb2gray_shift_regs#(
    .DWIDTH(3),   //数据位宽
    .DELAY_DUTY(7)//延迟的时钟周期数，最小是1
)vip_rgb2gray_shift_regs_0
(
    .clk(clk),
    .rst_n(rst_n),
    .idata({sink_sop,sink_eop,sink_valid}),
    .odata({source_sop,source_eop,source_valid})
);
endmodule 