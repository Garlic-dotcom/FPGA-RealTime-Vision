// 基于移位寄存器的行缓存（抽头）模块，Efinity 可综合
// 用于替代 Altera 的 altshift_taps(linebuffers)
// 参数说明：
//   DISTANCE : 抽头间间距（每个抽头的移位深度）
//   DWIDTH   : 数据位宽
//   TAPS_NUM : 抽头数
module extract_region_c_shiftram_xtap #(
	parameter DISTANCE = 1280,   // 抽头间间距
	parameter DWIDTH   = 8,      // 数据位宽
	parameter TAPS_NUM = 2       // 抽头数
)(
	input	                     clken,
	input	                     clock,
	input	[DWIDTH-1:0]	         shiftin,
	output	[DWIDTH-1:0]	         shiftout,
	output	[TAPS_NUM*DWIDTH-1:0] taps
);

	// 移位链总深度 TAPS_NUM*DISTANCE
	reg [DWIDTH-1:0] shift_ram [0:TAPS_NUM*DISTANCE-1];
	integer i;

	always @(posedge clock) begin
		if (clken) begin
			shift_ram[0] <= shiftin;
			for (i = 1; i < TAPS_NUM*DISTANCE; i = i + 1)
				shift_ram[i] <= shift_ram[i-1];
		end
	end

	// 第 t+1 个抽头输出 shiftin 延迟 (t+1)*DISTANCE 个时钟后的数据
	// taps[DWIDTH-1:0]           -> 延迟 1*DISTANCE
	// taps[2*DWIDTH-1:DWIDTH]    -> 延迟 2*DISTANCE
	// ...
	// taps[TAPS_NUM*DWIDTH-1-:DWIDTH] -> 延迟 TAPS_NUM*DISTANCE
	genvar t;
	generate
		for (t = 0; t < TAPS_NUM; t = t + 1) begin : gen_tap
			assign taps[(t+1)*DWIDTH-1 -: DWIDTH] = shift_ram[(t+1)*DISTANCE-1];
		end
	endgenerate

	assign shiftout = taps[TAPS_NUM*DWIDTH-1 -: DWIDTH];

endmodule
