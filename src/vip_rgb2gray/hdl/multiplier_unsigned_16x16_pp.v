// 16位无符号乘法器，支持范围0~65535
// pipeline:5 clk
module unsigned_multiplier_16x16_pp (
    input wire clk,
    input wire rst_n,
    input wire [15:0] a,  // 16位无符号输入
    input wire [15:0] b,  // 16位无符号输入
    output reg [31:0] p   // 32位无符号输出
);

// 4位无符号乘法器函数
function [7:0] mult_4x4;
    input [3:0] a;
    input [3:0] b;
    begin
        mult_4x4 = a * b;  // 使用4位*4位乘法
    end
endfunction

// 流水线寄存器（无需符号寄存器）

// 4位段（直接使用输入）
wire [3:0] a0 = a[3:0]    ;
wire [3:0] a1 = a[7:4]    ;
wire [3:0] a2 = a[11:8]   ;
wire [3:0] a3 = a[15:12]  ;

wire [3:0] b0 = b[3:0]    ;
wire [3:0] b1 = b[7:4]    ;
wire [3:0] b2 = b[11:8]   ;
wire [3:0] b3 = b[15:12]  ;

// 部分积（第1级流水线）
reg [7:0] pp00, pp01, pp02, pp03;
reg [7:0] pp10, pp11, pp12, pp13;
reg [7:0] pp20, pp21, pp22, pp23;
reg [7:0] pp30, pp31, pp32, pp33;

// 对齐后的部分积（第2级流水线）
reg [31:0] app00, app01, app02, app03;
reg [31:0] app10, app11, app12, app13;
reg [31:0] app20, app21, app22, app23;
reg [31:0] app30, app31, app32, app33;

// Wallace树中间结果（第3级流水线）
reg [31:0] sum1, sum2, sum3, sum4;
reg [31:0] carry1, carry2, carry3, carry4;

// 第1级流水线：4x4乘法计算
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pp00 <= 8'b0; pp01 <= 8'b0; pp02 <= 8'b0; pp03 <= 8'b0;
        pp10 <= 8'b0; pp11 <= 8'b0; pp12 <= 8'b0; pp13 <= 8'b0;
        pp20 <= 8'b0; pp21 <= 8'b0; pp22 <= 8'b0; pp23 <= 8'b0;
        pp30 <= 8'b0; pp31 <= 8'b0; pp32 <= 8'b0; pp33 <= 8'b0;
    end else begin
        // 计算所有4x4部分积
        pp00 <= mult_4x4(a0, b0);
        pp01 <= mult_4x4(a0, b1);
        pp02 <= mult_4x4(a0, b2);
        pp03 <= mult_4x4(a0, b3);
        
        pp10 <= mult_4x4(a1, b0);
        pp11 <= mult_4x4(a1, b1);
        pp12 <= mult_4x4(a1, b2);
        pp13 <= mult_4x4(a1, b3);
        
        pp20 <= mult_4x4(a2, b0);
        pp21 <= mult_4x4(a2, b1);
        pp22 <= mult_4x4(a2, b2);
        pp23 <= mult_4x4(a2, b3);
        
        pp30 <= mult_4x4(a3, b0);
        pp31 <= mult_4x4(a3, b1);
        pp32 <= mult_4x4(a3, b2);
        pp33 <= mult_4x4(a3, b3);
    end
end

// 第2级流水线：部分积对齐
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        app00 <= 32'b0; app01 <= 32'b0; app02 <= 32'b0; app03 <= 32'b0;
        app10 <= 32'b0; app11 <= 32'b0; app12 <= 32'b0; app13 <= 32'b0;
        app20 <= 32'b0; app21 <= 32'b0; app22 <= 32'b0; app23 <= 32'b0;
        app30 <= 32'b0; app31 <= 32'b0; app32 <= 32'b0; app33 <= 32'b0;
    end else begin
        // 对齐部分积（根据权重移位）
        app00 <= {24'b0, pp00};
        app01 <= {20'b0, pp01, 4'b0};
        app02 <= {16'b0, pp02, 8'b0};
        app03 <= {12'b0, pp03, 12'b0};
        
        app10 <= {20'b0, pp10, 4'b0};
        app11 <= {16'b0, pp11, 8'b0};
        app12 <= {12'b0, pp12, 12'b0};
        app13 <= {8'b0, pp13, 16'b0};
        
        app20 <= {16'b0, pp20, 8'b0};
        app21 <= {12'b0, pp21, 12'b0};
        app22 <= {8'b0, pp22, 16'b0};
        app23 <= {4'b0, pp23, 20'b0};
        
        app30 <= {12'b0, pp30, 12'b0};
        app31 <= {8'b0, pp31, 16'b0};
        app32 <= {4'b0, pp32, 20'b0};
        app33 <= {pp33, 24'b0};
    end
end

// 第3级流水线：Wallace树压缩
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum1 <= 32'b0; sum2 <= 32'b0; sum3 <= 32'b0; sum4 <= 32'b0;
        carry1 <= 32'b0; carry2 <= 32'b0; carry3 <= 32'b0; carry4 <= 32'b0;
    end else begin
        // 第一级压缩
        sum1 <= app00 + app01 + app02 + app03;
        sum2 <= app10 + app11 + app12 + app13;
        sum3 <= app20 + app21 + app22 + app23;
        sum4 <= app30 + app31 + app32 + app33;
    end
end

// 第4级流水线：最终累加
reg [31:0] sum;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum <= 32'b0;
    end else begin
        // 最终加法
        sum <= sum1 + sum2 + sum3 + sum4;
    end
end

// 第5级流水线：输出结果
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        p <= 32'd0;
    end else begin
        p <= sum;  // 无符号乘法，直接输出
    end
end

endmodule