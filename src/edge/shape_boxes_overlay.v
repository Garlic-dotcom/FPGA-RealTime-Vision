// Previous completed-frame object boxes and class text; combinational only.
// Slot zero is in the low bits. Segmentation/geometry runs independently and
// cannot backpressure or change the Sobel/Canny image pixels outside overlays.
module shape_boxes_overlay #(
 parameter WIDTH=1280,HEIGHT=720,MAX_OBJECTS=8,
 parameter ROI_X0=(WIDTH>16)?4:0,ROI_X1=(WIDTH>16)?WIDTH-4:WIDTH,
 parameter ROI_Y0=(HEIGHT>16)?4:0,ROI_Y1=(HEIGHT>16)?HEIGHT-4:HEIGHT
)(
 input [11:0] x,y,
 input [3:0] object_count,
 input [2*MAX_OBJECTS-1:0] shapes,
 input [12*MAX_OBJECTS-1:0] boxes_x0,boxes_x1,boxes_y0,boxes_y1,
 input limited,
 input [15:0] rgb_in,
 output reg [15:0] rgb_out
);
 function [34:0] glyph;
  input [7:0] c;
  begin
   case(c)
    "1":glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
    "2":glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
    "3":glyph={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
    "4":glyph={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
    "5":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
    "6":glyph={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
    "7":glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
    "8":glyph={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
    "C":glyph={5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110};
    "E":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
    "I":glyph={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
    "L":glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
    "R":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
    "T":glyph={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
    default:glyph=0;
   endcase
  end
 endfunction
 integer i;
 reg [2:0] character,column,row;
 reg [11:0] bx0,bx1,by0,by1,ty;
 reg [1:0] kind;
 reg [15:0] color;
 reg [7:0] ch;
 reg [34:0] bitmap;
 reg text_active;
 reg [15:0] text_color;
 reg [11:0] text_x0,text_y0;
 reg [1:0] text_kind;
 reg [3:0] text_number;
 always @* begin
  rgb_out=rgb_in;
  bx0=0;bx1=0;by0=0;by1=0;ty=0;kind=0;color=0;ch=0;bitmap=0;
  text_active=0;text_color=0;
  text_x0=0;text_y0=0;text_kind=0;text_number=0;
  character=0;column=0;row=0;
  if(((x==ROI_X0 || x==ROI_X1-1) && y>=ROI_Y0 && y<ROI_Y1) ||
     ((y==ROI_Y0 || y==ROI_Y1-1) && x>=ROI_X0 && x<ROI_X1)) rgb_out=16'h07ff;
  for(i=0;i<MAX_OBJECTS;i=i+1) begin
   bx0=boxes_x0[i*12+:12];bx1=boxes_x1[i*12+:12];
   by0=boxes_y0[i*12+:12];by1=boxes_y1[i*12+:12];kind=shapes[i*2+:2];
   color=(kind==1)?16'h07e0:(kind==2)?16'hffe0:16'hfd20;
   if(i<object_count && kind!=0) begin
    if(((x==bx0 || x==bx1) && y>=by0 && y<=by1) ||
       ((y==by0 || y==by1) && x>=bx0 && x<=bx1)) rgb_out=color;
    ty=(by0>=9)?by0-9:by0+2;
    if(x>=bx0 && x<bx0+64 && y>=ty && y<ty+7) begin
     text_active=1;text_color=color;
     text_x0=bx0;text_y0=ty;text_kind=kind;text_number=i+1;
    end
   end
  end
  // Share one font ROM/mux for all objects. Overlapping label regions use
  // the last packed object, matching the deterministic box priority.
  character=(x-text_x0)>>3;column=(x-text_x0)&7;row=y-text_y0;
  case(character)
   0:ch=8'h30+text_number;
   1:ch=" ";
   2:ch=(text_kind==1)?"R":(text_kind==2)?"C":"T";
   3:ch=(text_kind==1)?"E":(text_kind==2)?"I":"R";
   4:ch=(text_kind==1)?"C":(text_kind==2)?"R":"I";
   5:ch=(text_kind==1)?"T":(text_kind==2)?"C":" ";
   6:ch=(text_kind==2)?"L":" ";
   7:ch=(text_kind==2)?"E":" ";
  endcase
  bitmap=glyph(ch);
  if(text_active && column<5 && bitmap[34-row*5-column]) rgb_out=text_color;
  // A capacity/framing limit clears boxes and gives a visible amber marker.
  if(limited && x>=ROI_X1-12 && x<ROI_X1 && y>=ROI_Y0 && y<ROI_Y0+8) rgb_out=16'hfd20;
 end
endmodule
