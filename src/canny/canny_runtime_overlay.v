// Clock-derived measurements of completed Canny jobs (not HDMI refresh rate).
// FPS is the count in the previous one-second gate; MS is last busy duration,
// truncated to milliseconds. Data crosses through the established mailboxes.
module canny_runtime_overlay #(parameter CLOCK_HZ=96000000)(
 input sys_clk,sys_rst_n,pixel_clk,pixel_rst_n,
 input enabled,busy,failed,result_valid,
 input [31:0] completed_frames,
 input [31:0] input_cycles,hysteresis_cycles,
 input [7:0] displayed_fps,
 input [15:0] captured_frames,
 input video_de,video_vsync,
 input [23:0] video_rgb,
 output [23:0] output_rgb
);
 reg [31:0] second_count,ms_count,previous_completed;
 reg [7:0] gate_frames,fps;
 reg [13:0] elapsed_ms,last_ms;
 reg busy_d,busy_completed;
 reg enabled_d;
 reg [15:0] previous_capture;
 reg [7:0] capture_gate,capture_fps;
 wire [15:0] capture_delta=captured_frames-previous_capture;
 wire [16:0] capture_sum={9'd0,capture_gate}+{1'b0,capture_delta};
 wire completed=completed_frames!=previous_completed;
 // Small sequential converters avoid adding a 32-bit divider to the video
 // timing path. They latch one successful job, saturate at 9999 milliseconds,
 // and publish only complete values. Input time overlaps frontend processing;
 // hysteresis includes seed scan, global propagation and result streaming.
 reg converting;
 reg [31:0] input_remaining,hys_remaining;
 reg [13:0] input_ms_work,hys_ms_work,input_ms,hys_ms;
 always @(posedge sys_clk or negedge sys_rst_n) begin
   if(!sys_rst_n) begin
     converting<=0;input_remaining<=0;hys_remaining<=0;
     input_ms_work<=0;hys_ms_work<=0;input_ms<=0;hys_ms<=0;
   end else if(completed) begin
     converting<=1;input_remaining<=input_cycles;hys_remaining<=hysteresis_cycles;
     input_ms_work<=0;hys_ms_work<=0;
   end else if(converting) begin
     if(input_remaining>=CLOCK_HZ/1000 && input_ms_work<9999) begin
       input_remaining<=input_remaining-CLOCK_HZ/1000;input_ms_work<=input_ms_work+1'b1;
     end
     if(hys_remaining>=CLOCK_HZ/1000 && hys_ms_work<9999) begin
       hys_remaining<=hys_remaining-CLOCK_HZ/1000;hys_ms_work<=hys_ms_work+1'b1;
     end
     if((input_remaining<CLOCK_HZ/1000 || input_ms_work==9999) &&
        (hys_remaining<CLOCK_HZ/1000 || hys_ms_work==9999)) begin
       converting<=0;input_ms<=input_ms_work;hys_ms<=hys_ms_work;
     end
   end
 end
 always @(posedge sys_clk or negedge sys_rst_n) begin
   if(!sys_rst_n) begin
     second_count<=0;ms_count<=0;previous_completed<=0;gate_frames<=0;fps<=0;
     elapsed_ms<=0;last_ms<=0;busy_d<=0;busy_completed<=0;
     enabled_d<=0;previous_capture<=0;capture_gate<=0;capture_fps<=0;
   end else begin
     previous_completed<=completed_frames;busy_d<=busy;
     enabled_d<=enabled;previous_capture<=captured_frames;
     if(second_count==CLOCK_HZ-1) begin
       capture_fps<=(capture_sum>255)?8'd255:capture_sum[7:0];capture_gate<=0;
     end else capture_gate<=(capture_sum>255)?8'd255:capture_sum[7:0];
     if(second_count==CLOCK_HZ-1) begin
       second_count<=0;fps<=(gate_frames==255)?255:gate_frames+completed;gate_frames<=0;
     end else begin second_count<=second_count+1'b1;if(completed && gate_frames!=255) gate_frames<=gate_frames+1'b1;end
     if(!enabled || enabled!=enabled_d) begin gate_frames<=0;fps<=0;end
     // LIVE duplicate reservations also pulse port busy, but never complete
     // a job. Retain M from the last successful busy interval across skips.
     if(busy && !busy_d) begin ms_count<=0;elapsed_ms<=0;busy_completed<=completed;end
     else if(busy) begin
       if(completed) busy_completed<=1;
       if(ms_count==CLOCK_HZ/1000-1) begin ms_count<=0;if(elapsed_ms<9999) elapsed_ms<=elapsed_ms+1'b1;end
       else ms_count<=ms_count+1'b1;
     end
     if(busy_d && !busy && !failed && (busy_completed || completed)) last_ms<=elapsed_ms;
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
 wire [15:0] fps_bcd=bcd({6'd0,fps});
 wire [15:0] status,latency;
 wire [15:0] capture_latency;
 wire [15:0] display_bcd=bcd({6'd0,displayed_fps});
 history_status_cdc capture_cdc(.source_clk(sys_clk),.source_rst_n(sys_rst_n),.data_in(bcd({6'd0,capture_fps})),
 .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(capture_latency));
 wire [15:0] input_latency,hys_latency;
 history_status_cdc input_cdc(.source_clk(sys_clk),.source_rst_n(sys_rst_n),.data_in(bcd(input_ms)),
 .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(input_latency));
 history_status_cdc hys_cdc(.source_clk(sys_clk),.source_rst_n(sys_rst_n),.data_in(bcd(hys_ms)),
 .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(hys_latency));
 history_status_cdc status_cdc(.source_clk(sys_clk),.source_rst_n(sys_rst_n),
 .data_in({enabled,failed,result_valid,busy,fps_bcd[11:0]}),
 .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(status));
 history_status_cdc latency_cdc(.source_clk(sys_clk),.source_rst_n(sys_rst_n),.data_in(bcd(last_ms)),
 .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(latency));
 reg [10:0] x,y;reg de_d;
 always @(posedge pixel_clk or negedge pixel_rst_n) begin
   if(!pixel_rst_n) begin x<=0;y<=0;de_d<=0;end
   else begin
     de_d<=video_de;
     if(!video_vsync) begin x<=0;y<=0;end
     else begin if(video_de) x<=x+1'b1;else x<=0;if(de_d&&!video_de) y<=y+1'b1;end
   end
 end
 wire box=video_de && x>=1136 && x<1168 && y>=8 && y<24;
 wire [23:0] color=status[14]?24'hff2020:status[13]?24'h20ff40:24'hffff20;
 wire extra_row=y>=88;
 wire [10:0] tx=x-8,ty=extra_row?y-88:y-64;
 wire [4:0] ci=tx[8:4];wire [2:0] col=tx[3:1],row=ty[3:1];
 reg [3:0] digit;
 reg [34:0] glyph;
 always @* begin
   digit=15;
   case(ci)
     4:digit=display_bcd[11:8];5:digit=display_bcd[7:4];6:digit=display_bcd[3:0];
     9:digit=latency[15:12];10:digit=latency[11:8];11:digit=latency[7:4];12:digit=latency[3:0];
     15:digit=input_latency[15:12];16:digit=input_latency[11:8];17:digit=input_latency[7:4];18:digit=input_latency[3:0];
     21:digit=hys_latency[15:12];22:digit=hys_latency[11:8];23:digit=hys_latency[7:4];24:digit=hys_latency[3:0];
   endcase
   if(!status[15] && ci>=8) digit=15;
   if(extra_row) begin
     digit=15;
     case(ci)
       4:digit=capture_latency[11:8];5:digit=capture_latency[7:4];6:digit=capture_latency[3:0];
       12:digit=status[15]?status[11:8]:4'd15;
       13:digit=status[15]?status[7:4]:4'd15;
       14:digit=status[15]?status[3:0]:4'd15;
     endcase
   end
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
   if(!extra_row) begin
   // Display updates and retained Canny timing.
   if(ci==1) glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
   if(ci==2) glyph={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
   if(ci==3) glyph={5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000};
   if(ci==0) glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
   if(ci==8) glyph={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
   if(ci==14) glyph={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
   if(ci==20) glyph={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
   if(!extra_row && !status[15] && ci>=8) glyph=0;
   end
   if(extra_row) begin
     // CAP:xxx ALG:xxx. ALG is Canny task completion, independent of FPS.
     if(ci==0) glyph={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
     if(ci==1 || ci==8) glyph={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
     if(ci==2) glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
     if(ci==3 || ci==11) glyph={5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000};
     if(ci==9) glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
     if(ci==10) glyph={5'b01111,5'b10000,5'b10000,5'b10111,5'b10001,5'b10001,5'b01111};
     if(ci==7 || ci>=15) glyph=0;
     if(!status[15] && ci>=8) glyph=0;
   end
 end
 wire [4:0] row_bits=glyph[34-row*5-:5];
 wire panel=video_de && x>=4 && x<412 && ((y>=60 && y<82) || (y>=84 && y<106));
 wire ink=x>=8 && x<408 && ((y>=64 && y<78) || (y>=88 && y<102)) && col<5 && row_bits[4-col];
 assign output_rgb=box?color:panel?(ink?24'hffffff:24'h101010):video_rgb;
endmodule

// Supplemental observations, not FPS or camera-to-HDMI latency. U is the
// free-running sys one-second window (hex); J is the last successful job's
// source generation and D the generation adopted for a DDR display read (hex).
// A/R/W are floor milliseconds of overlapping address/read/write bus waits
// from that same successful job. Repeated generations remain visible in HOLD.
module canny_task_observer_overlay #(parameter CLOCK_HZ=96000000)(
 input sys_clk,sys_rst_n,pixel_clk,pixel_rst_n,enabled,
 input [31:0] completed_frames,ar_wait_cycles,read_wait_cycles,write_wait_cycles,
 input [15:0] job_generation,adopted_generation,
 input video_de,video_vsync,input [23:0] video_rgb,output [23:0] output_rgb
);
 reg [31:0] window_clock,previous_completed,ar_remaining,read_remaining,write_remaining;
 reg [15:0] window_id,observed_job,pending_job;
 reg [13:0] ar_work,read_work,write_work;
 reg [47:0] wait_bcd;
 reg converting;
 wire completed=completed_frames!=previous_completed;
 // Decimal accumulators follow the existing one-millisecond work steps.
 // The unchanged work<9999 guards prevent incrementing beyond decimal9999.
 reg [15:0] ar_decimal,read_decimal,write_decimal;
 function [15:0] increment_decimal;
  input [15:0] v;
  begin
   increment_decimal=v;
   if(v[3:0]==4'd9) begin
    increment_decimal[3:0]=0;
    if(v[7:4]==4'd9) begin
     increment_decimal[7:4]=0;
     if(v[11:8]==4'd9) begin
      increment_decimal[11:8]=0;
      increment_decimal[15:12]=v[15:12]+4'd1;
     end else increment_decimal[11:8]=v[11:8]+4'd1;
    end else increment_decimal[7:4]=v[7:4]+4'd1;
   end else increment_decimal[3:0]=v[3:0]+4'd1;
  end
 endfunction
 always @(posedge sys_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin
   window_clock<=0;window_id<=0;previous_completed<=0;observed_job<=0;pending_job<=0;
   ar_remaining<=0;read_remaining<=0;write_remaining<=0;
   ar_work<=0;read_work<=0;write_work<=0;wait_bcd<=0;converting<=0;
   ar_decimal<=0;read_decimal<=0;write_decimal<=0;
  end else begin
   previous_completed<=completed_frames;
   if(window_clock==CLOCK_HZ-1) begin window_clock<=0;window_id<=window_id+1'b1;end
   else window_clock<=window_clock+1'b1;
   if(completed) begin
    pending_job<=job_generation;
    converting<=1;ar_remaining<=ar_wait_cycles;read_remaining<=read_wait_cycles;write_remaining<=write_wait_cycles;
    ar_work<=0;read_work<=0;write_work<=0;
    ar_decimal<=0;read_decimal<=0;write_decimal<=0;
   end else if(converting) begin
    if(ar_remaining>=CLOCK_HZ/1000 && ar_work<9999) begin ar_remaining<=ar_remaining-CLOCK_HZ/1000;ar_work<=ar_work+1'b1;ar_decimal<=increment_decimal(ar_decimal);end
    if(read_remaining>=CLOCK_HZ/1000 && read_work<9999) begin read_remaining<=read_remaining-CLOCK_HZ/1000;read_work<=read_work+1'b1;read_decimal<=increment_decimal(read_decimal);end
    if(write_remaining>=CLOCK_HZ/1000 && write_work<9999) begin write_remaining<=write_remaining-CLOCK_HZ/1000;write_work<=write_work+1'b1;write_decimal<=increment_decimal(write_decimal);end
    if((ar_remaining<CLOCK_HZ/1000 || ar_work==9999) &&
       (read_remaining<CLOCK_HZ/1000 || read_work==9999) &&
       (write_remaining<CLOCK_HZ/1000 || write_work==9999)) begin
     converting<=0;wait_bcd<={ar_decimal,read_decimal,write_decimal};
     observed_job<=pending_job;
    end
   end
  end
 end
 wire [96:0] info;
 history_status_cdc #(.WIDTH(97)) observer_cdc(
  .source_clk(sys_clk),.source_rst_n(sys_rst_n),.data_in({enabled,window_id,observed_job,adopted_generation,wait_bcd}),
  .dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(info));
 reg [96:0] frame_info;
 reg [10:0] x,y;reg de_d;
 always @(posedge pixel_clk or negedge pixel_rst_n) begin
  if(!pixel_rst_n) begin x<=0;y<=0;de_d<=0;frame_info<=0;end
  else begin
   de_d<=video_de;
   if(!video_vsync) begin x<=0;y<=0;frame_info<=info;end
   else begin if(video_de) x<=x+1'b1;else x<=0;if(de_d&&!video_de) y<=y+1'b1;end
  end
 end
 wire wait_row=y>=160;
 wire [10:0] tx=x-8,ty=wait_row?y-160:y-136;
 wire [4:0] ci=tx[8:4];wire [2:0] col=tx[3:1],row=ty[3:1];
 reg [3:0] digit;reg number;reg [34:0] glyph;
 always @* begin
  digit=0;number=0;
  if(ci>=1 && ci<=4) begin number=1;digit=wait_row?(frame_info[47:32]>>(4*(4-ci))):(frame_info[95:80]>>(4*(4-ci)));end
  if(ci>=7 && ci<=10) begin number=1;digit=wait_row?(frame_info[31:16]>>(4*(10-ci))):(frame_info[79:64]>>(4*(10-ci)));end
  if(ci>=13 && ci<=16) begin number=1;digit=wait_row?(frame_info[15:0]>>(4*(16-ci))):(frame_info[63:48]>>(4*(16-ci)));end
  glyph=0;
  if(number) case(digit)
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
   10:glyph={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
   11:glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
   12:glyph={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
   13:glyph={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
   14:glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
   15:glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
  endcase
  if(ci==0) glyph=wait_row?{5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001}:
                                {5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
  if(ci==6) glyph=wait_row?{5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001}:
                                {5'b00111,5'b00010,5'b00010,5'b00010,5'b00010,5'b10010,5'b01100};
  if(ci==12) glyph=wait_row?{5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010}:
                                 {5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
 end
 wire [4:0] row_bits=glyph[34-row*5-:5];
 wire panel=frame_info[96] && video_de && x>=4 && x<284 && ((y>=132 && y<154)||(y>=156 && y<178));
 wire ink=x>=8 && x<280 && ((y>=136 && y<150)||(y>=160 && y<174)) && col<5 && row_bits[4-col];
 assign output_rgb=panel?(ink?24'hffffff:24'h101010):video_rgb;
endmodule
