// Second status line. A is newest frozen frame; B is selected history age.
module frame_history_osd #(parameter HEIGHT=720, PRESENTATION_MODE=1)(
 input [11:0] x,y,input [15:0] status,input [15:0] rgb_in,output [15:0] rgb_out);
 wire [1:0] mode=status[13:12];
 wire [5:0] count=status[11:6],age=status[5:0];
 reg [31:0] label;
 always @* case(mode)
   0:label="LIVE";1:label="HOLD";2:label="PLAY";3:label="COMP";
 endcase
 wire [11:0] tx=x-12'd8,ty=y-12'd32;
 wire [3:0] index=tx[7:4];
 wire [2:0] col=tx[3:1],row=ty[3:1];
 reg [7:0] ch;
 always @* begin
   ch=" ";
   if(index<4) ch=label[31-index*8-:8];
   else case(index)
     5:ch="B";6:ch=8'h30+age/10;7:ch=8'h30+age%10;
     9:ch="N";10:ch=8'h30+count/10;11:ch=8'h30+count%10;
   endcase
 end
 function [34:0] glyph;
   input [7:0] c;
   begin
     case(c)
       "0":glyph={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
       "1":glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
       "2":glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
       "3":glyph={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
       "4":glyph={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
       "5":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
       "6":glyph={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
       "7":glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
       "8":glyph={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
       "9":glyph={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
       "A":glyph={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
       "B":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
       "C":glyph={5'b01110,5'b10001,5'b10000,5'b10000,5'b10000,5'b10001,5'b01110};
       "D":glyph={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
       "E":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
       "G":glyph={5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110};
       "I":glyph={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
       "L":glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
       "M":glyph={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
       "N":glyph={5'b10001,5'b11001,5'b11001,5'b10101,5'b10011,5'b10011,5'b10001};
       "O":glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
       "P":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
       "R":glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
       "S":glyph={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
       "T":glyph={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
       "Y":glyph={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
       "H":glyph={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
       "V":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
       default:glyph=35'd0;
     endcase
   end
 endfunction

 wire [34:0] bitmap=glyph(ch);
 wire [4:0] row_bits=bitmap[34-row*5-:5];
 wire [11:0] text_end=PRESENTATION_MODE?((mode==0)?12'd72:12'd136):12'd200;
 wire ink=x>=8 && x<text_end && y>=32 && y<46 && col<5 && row_bits[4-col];
 wire panel=x>=4 && x<text_end+4 && y>=28 && y<50;
 // Hide the artificial filter boundary between two different frame halves.
 wire seam=mode==3 && y>=HEIGHT/2-2 && y<=HEIGHT/2+2;
 assign rgb_out=status[15]?16'hf800:panel?(ink?16'hffff:16'h1082):seam?16'h07ff:rgb_in;
endmodule
