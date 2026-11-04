// Gradient bins are normals to the displayed contour. Image y points down.
// 0: vertical contour, 1: / contour, 2: horizontal contour, 3: backslash.
module color_edge_overlay #(parameter STRENGTH_ENABLE=0)(
 input [15:0] rgb_in,
 input edge_valid,
 input [1:0] direction,
 output [15:0] rgb_out,
 input [10:0] magnitude,
 input strength_mode
);
 reg [15:0] color;
 always @* begin
   case(direction)
     0:color=16'hf800;
     1:color=16'hffe0;
     2:color=16'h07ff;
     3:color=16'hf81f;
   endcase
   if(STRENGTH_ENABLE && strength_mode)begin
     if(magnitude<192)color=16'h07ff;
     else if(magnitude<384)color=16'h07e0;
     else if(magnitude<576)color=16'hffe0;
     else color=16'hf800;
   end
 end
 assign rgb_out=edge_valid?color:rgb_in;
endmodule
