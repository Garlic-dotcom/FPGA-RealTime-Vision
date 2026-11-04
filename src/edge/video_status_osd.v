// Small 5x7 text, scaled by two. All coordinates and colors remain combinational;
// the parent registers the video stream before its frame-buffer write port.
module video_status_osd #(parameter DIAGNOSTICS_ENABLE=0, PRESENTATION_MODE=1, FILTER_PREVIEW_ENABLE=0,DIRECTION_LEGEND_ENABLE=0,STRENGTH_COLOR_ENABLE=0)(
 input [11:0] x,y,
 input [2:0] mode,
 input filter_gaussian,
 input color_direction,
 input [1:0] shape,
 input [15:0] threshold_bcd,
 input [159:0] diagnostics,
 input [6:0] recognition_threshold,
 input diagnostic_ready,
 input [3:0] object_count,
 input [15:0] rgb_in,
 output [15:0] rgb_out
);
 reg [47:0] label;
 always @* begin
   case(mode)
     0:label="SPLIT ";1:label="GRAY  ";2:label=(FILTER_PREVIEW_ENABLE && filter_gaussian)?"GAUSS ":"MEDIAN";
     3:label="MAG   ";4:label="BINARY";5:label=STRENGTH_COLOR_ENABLE?(color_direction?"RGB D ":"RGB S "):"RGB   ";
     6:label=PRESENTATION_MODE?"SHAPES":(shape==3)?"TRI   ":(shape==2)?"CIRCLE":(shape==1)?"RECT  ":"NONE  ";
     7:label="CANNY ";
     default:label="RGB   ";
   endcase
 end
 wire [11:0] tx=x-12'd8,ty=y-12'd8;
 wire [3:0] char_index=tx[7:4];
 wire [2:0] column=tx[3:1],row=ty[3:1];
 reg [7:0] ch;
 always @* begin
   ch=" ";
   if(char_index<6) ch=label[47-char_index*8-:8];
   else if(!PRESENTATION_MODE || mode==0 || (mode>=3 && mode<=5) || mode==7) case(char_index)
     6:ch="T";
     7:ch=8'h30+threshold_bcd[15:12];
     8:ch=8'h30+threshold_bcd[11:8];
     9:ch=8'h30+threshold_bcd[7:4];
     10:ch=8'h30+threshold_bcd[3:0];
   endcase
 end
 // Mode6 diagnostic panel, hexadecimal, at x512; away from history/FPS.
 // R=recognition threshold F=fault bits L=labels B=row bank peak
 // E=events C=components P=coarse candidates G=geometry passed
 // A=analysis clocks I=accepted D=completed S=skipped O=objects V=ready
 wire [11:0] diagnostic_x=x-12'd512,diagnostic_y=y-12'd8;
 wire [4:0] diagnostic_index=diagnostic_x[8:4];
 wire [1:0] diagnostic_line=diagnostic_y[6:5];
 reg [7:0] diagnostic_ch;
 function [7:0] hexchar;
   input [3:0] n;
   begin hexchar=(n<10)?8'h30+n:8'h41+(n-10);end
 endfunction
 always @* begin
   diagnostic_ch=" ";
   case(diagnostic_line)
    0:case(diagnostic_index)
      0:diagnostic_ch="R";1:diagnostic_ch=hexchar({1'b0,recognition_threshold[6:4]});2:diagnostic_ch=hexchar(recognition_threshold[3:0]);
      4:diagnostic_ch="F";5:diagnostic_ch=hexchar(diagnostics[7:4]);6:diagnostic_ch=hexchar(diagnostics[3:0]);
      8:diagnostic_ch="L";9:diagnostic_ch=hexchar(diagnostics[15:12]);10:diagnostic_ch=hexchar(diagnostics[11:8]);
      12:diagnostic_ch="B";13:diagnostic_ch=hexchar({3'b0,diagnostics[20]});14:diagnostic_ch=hexchar(diagnostics[19:16]);
    endcase
    1:case(diagnostic_index)
      0:diagnostic_ch="E";1:diagnostic_ch=hexchar(diagnostics[32:29]);2:diagnostic_ch=hexchar(diagnostics[28:25]);3:diagnostic_ch=hexchar(diagnostics[24:21]);
      5:diagnostic_ch="C";6:diagnostic_ch=hexchar({3'b0,diagnostics[37]});7:diagnostic_ch=hexchar(diagnostics[36:33]);
      9:diagnostic_ch="P";10:diagnostic_ch=hexchar({3'b0,diagnostics[42]});11:diagnostic_ch=hexchar(diagnostics[41:38]);
      13:diagnostic_ch="G";14:diagnostic_ch=hexchar({3'b0,diagnostics[47]});15:diagnostic_ch=hexchar(diagnostics[46:43]);
    endcase
    2:case(diagnostic_index)
      0:diagnostic_ch="A";1:diagnostic_ch=hexchar({2'b0,diagnostics[65:64]});2:diagnostic_ch=hexchar(diagnostics[63:60]);3:diagnostic_ch=hexchar(diagnostics[59:56]);4:diagnostic_ch=hexchar(diagnostics[55:52]);5:diagnostic_ch=hexchar(diagnostics[51:48]);
      7:diagnostic_ch="I";8:diagnostic_ch=hexchar(diagnostics[81:78]);9:diagnostic_ch=hexchar(diagnostics[77:74]);10:diagnostic_ch=hexchar(diagnostics[73:70]);11:diagnostic_ch=hexchar(diagnostics[69:66]);
      13:diagnostic_ch="D";14:diagnostic_ch=hexchar(diagnostics[97:94]);15:diagnostic_ch=hexchar(diagnostics[93:90]);16:diagnostic_ch=hexchar(diagnostics[89:86]);17:diagnostic_ch=hexchar(diagnostics[85:82]);
    endcase
    3:case(diagnostic_index)
      0:diagnostic_ch="S";1:diagnostic_ch=hexchar(diagnostics[129:126]);2:diagnostic_ch=hexchar(diagnostics[125:122]);3:diagnostic_ch=hexchar(diagnostics[121:118]);4:diagnostic_ch=hexchar(diagnostics[117:114]);5:diagnostic_ch=hexchar(diagnostics[113:110]);6:diagnostic_ch=hexchar(diagnostics[109:106]);7:diagnostic_ch=hexchar(diagnostics[105:102]);8:diagnostic_ch=hexchar(diagnostics[101:98]);
      10:diagnostic_ch="O";11:diagnostic_ch=hexchar(object_count);
      13:diagnostic_ch="V";14:diagnostic_ch=diagnostic_ready?"1":"0";
    endcase
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
       "F":glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
       "U":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
       "V":glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
       "G":glyph={5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110};
       "H":glyph={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
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
       default:glyph=35'd0;
     endcase
   end
 endfunction
 wire [34:0] bitmap=glyph(ch);
 wire [4:0] row_bits=bitmap[34-row*5-:5];
 wire show_threshold=!PRESENTATION_MODE || mode==0 || (mode>=3 && mode<=5) || mode==7;
 wire ink=x>=8 && x<(show_threshold?184:104) && y>=8 && y<22 && column<5 && row_bits[4-column];
 wire panel=x>=4 && x<(show_threshold?188:108) && y>=4 && y<26;
 wire diagnostic_panel=!PRESENTATION_MODE && DIAGNOSTICS_ENABLE && mode==6 && x>=508 && x<804 && y>=4 && y<122;
 wire [34:0] diagnostic_bitmap=glyph(diagnostic_ch);
 wire [2:0] diagnostic_row=diagnostic_y[3:1],diagnostic_column=diagnostic_x[3:1];
 wire [4:0] diagnostic_row_bits=diagnostic_bitmap[34-diagnostic_row*5-:5];
 wire diagnostic_ink=x>=512 && x<800 && y>=8 && y<118 && diagnostic_y[4:0]<14 && diagnostic_column<5 && diagnostic_row_bits[4-diagnostic_column];
 wire legend=DIRECTION_LEGEND_ENABLE && mode==5 && x>=8 && x<136 && y>=28 && y<42;
 wire [6:0] lx=x-12'd8;wire [3:0] ly=y-12'd28;
 reg legend_ink;reg [15:0] legend_color;
 always @* begin
  legend_ink=0;legend_color=0;
  case(lx[6:5])
   0:begin legend_color=16'hf800;legend_ink=lx[4:0]>=15 && lx[4:0]<=16 && ly>=2 && ly<=11;end
   1:begin legend_color=16'h07ff;legend_ink=ly>=6 && ly<=7 && lx[4:0]>=10 && lx[4:0]<=21;end
   2:begin legend_color=16'hffe0;legend_ink=ly>=2 && ly<=11 && ({1'b0,lx[4:0]}+ly>=22) && ({1'b0,lx[4:0]}+ly<=23);end
   3:begin legend_color=16'hf81f;legend_ink=ly>=2 && ly<=11 && lx[4:0]>=ly+10 && lx[4:0]<=ly+11;end
  endcase
  if(STRENGTH_COLOR_ENABLE && !color_direction)begin
   legend_color=(lx[6:5]==0)?16'h07ff:(lx[6:5]==1)?16'h07e0:(lx[6:5]==2)?16'hffe0:16'hf800;
   legend_ink=lx[4:0]>=6 && lx[4:0]<=25 && ly<=12 &&
       ((lx[6:5]==0 && ly>=10)||(lx[6:5]==1 && ly>=7)||(lx[6:5]==2 && ly>=4)||(lx[6:5]==3 && ly>=1));
  end
 end
 assign rgb_out=legend?(legend_ink?legend_color:16'h1082):diagnostic_panel?(diagnostic_ink?16'hffff:16'h1082):panel ? (ink ? 16'hffff : 16'h1082) : rgb_in;
endmodule
