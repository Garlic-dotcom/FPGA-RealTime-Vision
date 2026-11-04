// 实现8位灰度图像数据的Sobel边缘检测
// WIDTH、HEIGHT为图像参数
module Sobeledge_8d_proc#(
parameter WIDTH=480,
parameter HEIGHT=320
)(
	input			    clk,
	input			    rst_n,
	input           edge_en,//边缘检测使能，高有效
	input           bin_en,//二值化使能，高有效
	//sink 
	input           sink_sop,
	input           sink_eop,
	input           sink_valid,
	input 	[7:0]  sink_data,			//原始数据
	//source
	output          source_sop,
	output          source_eop,
	output          source_valid,
	output  [7:0]   source_data			//Sobel边缘检测结果
);

// 3x3像素矩阵
wire [7:0]		matrix_p11,matrix_p12,matrix_p13;//窗口第一行数据
wire [7:0]		matrix_p21,matrix_p22,matrix_p23;//窗口第二行数据
wire [7:0]		matrix_p31,matrix_p32,matrix_p33;//窗口第三行数据
wire	matrix_sop,matrix_eop,matrix_valid;
wire [7:0]     matrix_p22_dly;//对matrix_p22延迟4个时钟，和Gmax对齐
Sobeledge_matrix_3x3 #(
.WIDTH(WIDTH),
.HEIGHT(HEIGHT)
)u_Sobeledge_matrix_3x3(
	.clk(clk),
	.rst_n(rst_n),
	
	.sink_sop(sink_sop),
	.sink_eop(sink_eop),
	.sink_valid(sink_valid),
	.sink_data(sink_data),
	
	.matrix_sop(matrix_sop),
	.matrix_eop(matrix_eop),
	.matrix_valid(matrix_valid),
	.matrix_p11(matrix_p11),
	.matrix_p12(matrix_p12),
	.matrix_p13(matrix_p13),
	.matrix_p21(matrix_p21),
	.matrix_p22(matrix_p22),
	.matrix_p23(matrix_p23),
	.matrix_p31(matrix_p31),
	.matrix_p32(matrix_p32),
	.matrix_p33(matrix_p33)
);
// Step 1 计算卷积并求绝对值
// Sobel算子
//																
// 		[-1	-2		-1]			[-2	-1		 0]		[-1		0	   +1]		 [0	+1		+2]
// 0°	   [0		0		 0]	45°	[-1	0		+1] 90°  [-2		0	   +2]  135° [-1	0		+1]
//		   [+1	+2		+1]			[0		+1		+2]		[-1		0	   +1]		 [-2	-1		 0]

// 		[+1	+2		+1]			[+2	+1		 0]		[+1		0	   -1]		 [0	-1		-2]
// 180°	[0		0		 0]	225°  [+1	0		-1] 270° [+2		0	   -2]  315° [+1	0		-1]
//		   [-1	-2		-1]			[0		-1		-2]		[+1		0	   -1]		 [+2	+1		 0]
//通过观察可以发现，180°和0°的算子是对称的，二者结果的绝对值是相等的，同理225°、270°、315°
//因此只需要计算4组算子的卷积即可
reg	[9:0]	G0_p,G0_n,G0;
reg	[9:0]	G45_p,G45_n,G45;
reg	[9:0]	G90_p,G90_n,G90;
reg	[9:0]	G135_p,G135_n,G135;
//0°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G0_p <= 'd0;
		G0_n <= 'd0;
		G0   <= 'd0;
	end
	else begin
		G0_p <= matrix_p11 + ({1'b0,matrix_p12,1'b0}) + matrix_p13;
		G0_n <= matrix_p31 + ({1'b0,matrix_p32,1'b0}) + matrix_p33;
		G0   <= (G0_p>=G0_n) ? G0_p-G0_n:G0_n-G0_p;
	end	
end
//45°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G45_p <= 'd0;
		G45_n <= 'd0;
		G45   <= 'd0;
	end
	else begin
		G45_p <= matrix_p23 + ({1'b0,matrix_p33,1'b0}) + matrix_p32;
		G45_n <= matrix_p12 + ({1'b0,matrix_p11,1'b0}) + matrix_p21;	
		G45   <= (G45_p>=G45_n) ? G45_p-G45_n:G45_n-G45_p;
	end	
end
//90°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G90_p <= 'd0;
		G90_n <= 'd0;
		G90   <= 'd0;
	end
	else begin
		G90_p <= matrix_p13 + ({1'b0,matrix_p23,1'b0}) + matrix_p33;
		G90_n <= matrix_p11 + ({1'b0,matrix_p21,1'b0}) + matrix_p31;
		G90   <= (G90_p>=G90_n) ? G90_p-G90_n:G90_n-G90_p;
	end	
end
//135°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G135_p <= 'd0;
		G135_n <= 'd0;
		G135   <= 'd0;
	end
	else begin
		G135_p <= matrix_p12 + ({1'b0,matrix_p13,1'b0}) + matrix_p23;
		G135_n <= matrix_p21 + ({1'b0,matrix_p31,1'b0}) + matrix_p32;
		G135   <= (G135_p>=G135_n) ? G135_p-G135_n:G135_n-G135_p;
	end	
end
// Step 2 找最大的G
// Gmax = max(G0,G45,G90,G135)
reg [9:0] Gmax_0_45,Gmax_90_135;
reg [9:0] Gmax;//边缘强度
always@(posedge clk or negedge rst_n)
begin
if(!rst_n)
    begin
    Gmax_0_45   <= 'd0;
    Gmax_90_135 <= 'd0;
    end
else
    begin
    Gmax_0_45   <= (G0 >= G45)?G0:G45;
    Gmax_90_135 <= (G90 >= G135)?G90:G135;
    end
end
always@(posedge clk or negedge rst_n)
begin
if(!rst_n)
	Gmax <= 'd0;
else
	Gmax <= (Gmax_0_45 >= Gmax_90_135)?Gmax_0_45:Gmax_90_135;
end
// Step 3 二值化、边缘强度、原始图选择
// 输入数据是中值滤波后的图像，所以根据论文中的算法，二值化只需要让边缘强度Gmax和本模块输入的数据中心点matrix_p22比较
assign 	source_data= edge_en?(bin_en?((Gmax[9:0] >= matrix_p22_dly)?8'hff:8'h0):((Gmax[9:0] > 10'd510)?8'hff:Gmax[8:1])):matrix_p22_dly;
shift_regs#(
.DWIDTH(3),     
.DELAY_DUTY(4)
)u_shift_regs_0(
    .clk(clk),
	.rst_n(rst_n),
	.idata({matrix_sop,matrix_valid,matrix_eop}),
	.odata({source_sop,source_valid,source_eop})
);
shift_regs#(
.DWIDTH(8),     
.DELAY_DUTY(4)
)u_shift_regs_1(
    .clk(clk),
	.rst_n(rst_n),
	.idata(matrix_p22),
	.odata(matrix_p22_dly)
);
endmodule