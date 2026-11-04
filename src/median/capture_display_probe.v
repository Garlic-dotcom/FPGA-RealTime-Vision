// Board diagnostic only. No changes to the normal stream when DISPLAY_TEST=0.
// Four top tiles (left to right): row byte count, row count, frame byte count,
// gray pixel count. White=matches configured geometry, dark=fails/not ready.
module capture_display_probe #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,cmos_vsync,cmos_href,gray_valid,gray_sop,
 output [7:0] pattern_data
);
 reg vs_d,href_d;
 reg [23:0] bytes_in_frame,gray_in_frame;
 reg [12:0] bytes_in_row;
 reg [11:0] rows_in_frame;
 reg bad_row;
 reg [3:0] status;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     vs_d<=0;href_d<=0;bytes_in_frame<=0;gray_in_frame<=0;
     bytes_in_row<=0;rows_in_frame<=0;bad_row<=0;status<=0;
   end else begin
     vs_d<=cmos_vsync;href_d<=cmos_href;
     if(cmos_vsync && !vs_d) begin
       status<={!bad_row && rows_in_frame!=0,rows_in_frame==HEIGHT,
                bytes_in_frame==WIDTH*HEIGHT*2,gray_in_frame==WIDTH*HEIGHT};
       bytes_in_frame<=0;gray_in_frame<=0;bytes_in_row<=0;
       rows_in_frame<=0;bad_row<=0;
     end else begin
       if(cmos_href) begin
         bytes_in_frame<=bytes_in_frame+1'b1;
         bytes_in_row<=bytes_in_row+1'b1;
       end
       if(href_d && !cmos_href) begin
         if(bytes_in_row!=WIDTH*2) bad_row<=1;
         rows_in_frame<=rows_in_frame+1'b1;bytes_in_row<=0;
       end
       if(gray_valid) gray_in_frame<=gray_in_frame+1'b1;
     end
   end
 end
 reg [11:0] x,y;
 wire [11:0] px=gray_sop ? 12'd0 : x;
 wire [11:0] py=gray_sop ? 12'd0 : y;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin x<=0;y<=0;end
   else if(gray_valid) begin
     if(px==WIDTH-1) begin x<=0;y<=(py==HEIGHT-1)?0:py+1'b1;end
     else begin x<=px+1'b1;y<=py;end
   end
 end
 wire tile_ok=(px<WIDTH/4)?status[3]:(px<WIDTH/2)?status[2]:
              (px<3*WIDTH/4)?status[1]:status[0];
 wire grid=px==0 || px==WIDTH-1 || px==WIDTH/4 || px==WIDTH/2 ||
           px==3*WIDTH/4 || py==HEIGHT-1 || py==HEIGHT/4 ||
           py==HEIGHT/2 || py==3*HEIGHT/4;
 assign pattern_data=(py<12) ? (tile_ok?8'd255:8'd32) :
                     grid ? 8'd255 : {2'b01,px[7:2]};
endmodule
