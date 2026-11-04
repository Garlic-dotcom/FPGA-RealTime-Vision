// Physical camera VSYNC timing, independently measured against clk_sys.
// VS counts active-high VSYNC falling edges, not complete RGB565 packets.
// P is the most recent falling-edge interval in milliseconds, not latency.
// Three synchronizer stages add bounded timestamp quantization; no camera or
// clock configuration is changed. The first edge only initializes the period.
module camera_timing_overlay #(parameter CLOCK_HZ=96000000)(
 input sys_clk,sys_rst_n,pixel_clk,pixel_rst_n,camera_vsync,
 input video_de,video_vsync,
 input [23:0] video_rgb,
 output [23:0] output_rgb
);
 reg [2:0] vs_pipe;
 reg vs_d,first_edge;
 wire edge_done=vs_d && !vs_pipe[2];
 reg [31:0] gate_clock,period_clock;
 reg [7:0] gate_frames,physical_fps;
 reg converting;
 reg [31:0] period_remaining;
 reg [13:0] period_work,period_ms;
 always @(posedge sys_clk or negedge sys_rst_n) begin
   if(!sys_rst_n) begin
     vs_pipe<=0;vs_d<=0;first_edge<=0;
     gate_clock<=0;period_clock<=0;gate_frames<=0;physical_fps<=0;
     converting<=0;period_remaining<=0;period_work<=0;period_ms<=0;
   end else begin
     vs_pipe<={vs_pipe[1:0],camera_vsync};vs_d<=vs_pipe[2];
     if(gate_clock==CLOCK_HZ-1) begin
       gate_clock<=0;gate_frames<=0;
       physical_fps<=gate_frames==255?8'd255:gate_frames+edge_done;
     end else begin
       gate_clock<=gate_clock+1'b1;
       if(edge_done && gate_frames!=255) gate_frames<=gate_frames+1'b1;
     end
     if(first_edge && period_clock!=32'hffffffff) period_clock<=period_clock+1'b1;
     if(edge_done) begin
       first_edge<=1;period_clock<=0;
       if(first_edge) begin
         converting<=1;period_remaining<=period_clock==32'hffffffff ? period_clock : period_clock+1'b1;
         period_work<=0;
       end
     end else if(converting) begin
       if(period_remaining>=CLOCK_HZ/1000 && period_work<9999) begin
         period_remaining<=period_remaining-CLOCK_HZ/1000;period_work<=period_work+1'b1;
       end else begin converting<=0;period_ms<=period_work;end
     end
   end
 end
 function [15:0] bcd;
   input [13:0] v;integer i,j;reg [15:0] r;
   begin
     r=0;
     for(i=13;i>=0;i=i-1) begin
       for(j=0;j<4;j=j+1) if(r[j*4+:4]>=5) r[j*4+:4]=r[j*4+:4]+4'd3;
       r={r[14:0],v[i]};
     end
     bcd=r;
   end
 endfunction
 wire [31:0] camera_info;
 history_status_cdc #(.WIDTH(32)) timing_cdc(
 .source_clk(sys_clk),.source_rst_n(sys_rst_n),.data_in({bcd({6'd0,physical_fps}),bcd(period_ms)}),
 .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(camera_info));
 reg [31:0] frame_info;
 reg [10:0] x,y;reg de_d;
 always @(posedge pixel_clk or negedge pixel_rst_n) begin
   if(!pixel_rst_n) begin x<=0;y<=0;de_d<=0;frame_info<=0;end
   else begin
     de_d<=video_de;
     if(!video_vsync) begin x<=0;y<=0;frame_info<=camera_info;end
     else begin if(video_de) x<=x+1'b1;else x<=0;if(de_d&&!video_de) y<=y+1'b1;end
   end
 end
 wire [10:0] tx=x-8,ty=y-112;
 wire [3:0] ci=tx[7:4];wire [2:0] col=tx[3:1],row=ty[3:1];
 reg [3:0] digit;
 reg [34:0] glyph;
 always @* begin
   digit=15;
   case(ci)
     3:digit=frame_info[27:24];4:digit=frame_info[23:20];5:digit=frame_info[19:16];
     8:digit=frame_info[15:12];9:digit=frame_info[11:8];10:digit=frame_info[7:4];11:digit=frame_info[3:0];
   endcase
   case(digit)
     0:glyph={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
     1:glyph={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
     2:glyph={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
     3:glyph={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
     4:glyph={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
     5:glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
     6:glyph={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
     7:glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
     8:glyph={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
     9:glyph={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
     default:glyph=0;
   endcase
   if(ci==0) glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
   if(ci==1) glyph={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
   if(ci==2) glyph={5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000};
   if(ci==7) glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
 end
 wire [4:0] row_bits=glyph[34-row*5-:5];
 wire panel=video_de && x>=4 && x<204 && y>=108 && y<130;
 wire ink=x>=8 && x<200 && y>=112 && y<126 && col<5 && row_bits[4-col];
 assign output_rgb=panel?(ink?24'hffffff:24'h101010):video_rgb;
endmodule
