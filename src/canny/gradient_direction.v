// Four undirected bins in image coordinates (x right, y down).
// tan(22.5 degrees) ~= 424/1024. Comparisons avoid division and square root.
module gradient_direction(
 input signed [10:0] gx,gy,
 output [1:0] direction
);
 wire [10:0] ax=gx[10]?-gx:gx,ay=gy[10]?-gy:gy;
 wire [20:0] ax1024={ax,10'b0},ay1024={ay,10'b0};
 wire [20:0] ax424={2'b0,ax,8'b0}+{3'b0,ax,7'b0}+{5'b0,ax,5'b0}+{7'b0,ax,3'b0};
 wire [20:0] ay424={2'b0,ay,8'b0}+{3'b0,ay,7'b0}+{5'b0,ay,5'b0}+{7'b0,ay,3'b0};
 // 00: left/right; 01: top-left/bottom-right; 10: up/down; 11: top-right/bottom-left.
 assign direction=(ay1024<=ax424)?2'd0:(ax1024<=ay424)?2'd2:
                  (gx[10]==gy[10])?2'd1:2'd3;
endmodule
