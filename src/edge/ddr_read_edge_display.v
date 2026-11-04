// Read RAW RGB565 from the existing FWFT DDR display FIFO, then process it.
// The output raster starts four lines later, allowing the filter, gradient
// and optional one-row NMS to fill. The pixel FIFO absorbs padding/tail bursts.
// DDR read timing keeps the original lcd_driver phase and VSYNC polarity.
module ddr_read_edge_display #(
 parameter HISTORY_OSD=0,WIDTH=1280,HEIGHT=720,
 parameter H_FRONT=110,H_SYNC=40,H_BACK=220,H_TOTAL=1650,
 parameter V_FRONT=5,V_SYNC=5,V_BACK=20,V_TOTAL=750,
 parameter DELAY_LINES=4,FIFO_DEPTH=4096,MEDIAN_ENABLE=1,SHAPE_ENABLE=1,OSD_ENABLE=1,FILTER_PREVIEW_ENABLE=0,EXTERNAL_SHAPES=0,STRENGTH_COLOR_ENABLE=0,RGB_STROKE_ENABLE=0
)(
 input clk,rst_n,
 input [15:0] raw_rgb565,
 input raw_empty,
 input [15:0] history_status,
 input [10:0] threshold,
 input [2:0] mode,
 input filter_gaussian,
 input color_direction,
 input [404:0] contour_shapes,
 input [31:0] frame_metadata,
 output [7:0] display_fps,
 output raw_request,raw_vsync,
 output video_hsync,video_vsync,video_de,
 output [23:0] video_rgb,
 output reg fault
);
 localparam N=WIDTH*HEIGHT,FDEPTH=RGB_STROKE_ENABLE?3072:FIFO_DEPTH,AW=$clog2(FDEPTH);
 localparam DELAY_CYCLES=(DELAY_LINES+(RGB_STROKE_ENABLE?1:0))*H_TOTAL,DW=$clog2(DELAY_CYCLES+1);
 wire read_request_i,read_vs_i;
 lcd_driver #(.H_FRONT(H_FRONT),.H_SYNC(H_SYNC),.H_BACK(H_BACK),.H_DISP(WIDTH),.H_TOTAL(H_TOTAL),
   .V_FRONT(V_FRONT),.V_SYNC(V_SYNC),.V_BACK(V_BACK),.V_DISP(HEIGHT),.V_TOTAL(V_TOTAL),.DATA_WIDTH(24)) read_timing(
   .clk(clk),.rst_n(rst_n),.lcd_request(read_request_i),.lcd_vs(read_vs_i),.lcd_data(24'd0));
 assign raw_request=rst_n && read_request_i;
 assign raw_vsync=rst_n ? read_vs_i : 1'b1;

 reg [DW-1:0] delay_count;
 reg output_reset_n;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin delay_count<=0;output_reset_n<=0;end
   else if(!output_reset_n) begin
     if(delay_count==DELAY_CYCLES-1) output_reset_n<=1;
     else delay_count<=delay_count+1'b1;
   end
 end
 wire output_request_i,hs,vs,de;
 lcd_driver #(.H_FRONT(H_FRONT),.H_SYNC(H_SYNC),.H_BACK(H_BACK),.H_DISP(WIDTH),.H_TOTAL(H_TOTAL),
   .V_FRONT(V_FRONT),.V_SYNC(V_SYNC),.V_BACK(V_BACK),.V_DISP(HEIGHT),.V_TOTAL(V_TOTAL),.DATA_WIDTH(24)) output_timing(
   .clk(clk),.rst_n(output_reset_n),.lcd_request(output_request_i),.lcd_hs(hs),.lcd_vs(vs),.lcd_en(de),.lcd_data(24'd0));
 wire pop=output_reset_n && output_request_i;
 assign video_hsync=output_reset_n ? hs : 1'b1;
 assign video_vsync=output_reset_n ? vs : 1'b1;
 assign video_de=output_reset_n && de;

 reg [31:0] input_count;
 reg [31:0] input_metadata,output_metadata;
 reg input_data_good;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin input_count<=0;input_metadata<=0;input_data_good<=0;end
   else if(raw_request) begin
     input_count<=(input_count==N-1)?0:input_count+1'b1;
     if(input_count==0) begin input_metadata<=frame_metadata;input_data_good<=!raw_empty;end
     else if(raw_empty) input_data_good<=0;
   end
 end
 wire gv,gs,ge;
 wire [7:0] gray;
 wire [15:0] aligned_rgb;
 vip_rgb2gray_proc grayscale(
   .clk(clk),.rst_n(rst_n),.sink_valid(raw_request),.sink_sop(input_count==0),.sink_eop(input_count==N-1),
   .sink_data({raw_rgb565[15:11],3'b111,raw_rgb565[10:5],2'b11,raw_rgb565[4:0],3'b111}),
   .source_valid(gv),.source_sop(gs),.source_eop(ge),.source_data(gray));
 vip_rgb2gray_shift_regs #(.DWIDTH(16),.DELAY_DUTY(7)) rgb_delay(
   .clk(clk),.rst_n(rst_n),.idata(raw_rgb565),.odata(aligned_rgb));
 wire pv,ps,pe;
 wire [15:0] pixel;
 wire [8:0] compare_data;
 wire [2:0] pixel_mode;
 wire [15:0] pixel_threshold_bcd;
 edge_video_pipeline #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MEDIAN_ENABLE(MEDIAN_ENABLE),
   .SHAPE_ENABLE(SHAPE_ENABLE),.OSD_ENABLE(OSD_ENABLE),.FILTER_PREVIEW_ENABLE(FILTER_PREVIEW_ENABLE),.EXTERNAL_SHAPES(EXTERNAL_SHAPES),.STRENGTH_COLOR_ENABLE(STRENGTH_COLOR_ENABLE),.RGB_STROKE_ENABLE(RGB_STROKE_ENABLE)) processing(
   .clk(clk),.rst_n(rst_n),.sink_valid(gv),.sink_sop(gs),.sink_eop(ge),.sink_gray(gray),.sink_rgb565(aligned_rgb),
   .threshold(threshold),.mode(mode),.filter_gaussian(filter_gaussian),.color_direction(color_direction),.contour_shapes(contour_shapes),.source_valid(pv),.source_sop(ps),.source_eop(pe),.source_rgb565(pixel),
   .source_compare_data(compare_data),.source_mode(pixel_mode),.source_threshold_bcd(pixel_threshold_bcd));

 // Separate synchronous read/write ports infer BRAM. The read register is
 // updated on the same edge as output_timing.lcd_en for that requested pixel.
 wire [15:0] output_word;
 reg read_packed_q,read_lane_q;
 wire [7:0] gray_byte=read_lane_q?output_word[15:8]:output_word[7:0];
 wire [15:0] output_pixel=read_packed_q?{gray_byte[5:1],gray_byte[5:0],gray_byte[5:1]}:output_word;
 reg [11:0] output_x,output_y,display_x,display_y;
 reg [AW:0] wp,rp;reg [AW-1:0] row_read_base;
 reg [2:0] prepared_mode,output_mode;
 reg [15:0] prepared_threshold,output_threshold;
 wire output_start=pop && output_x==0 && output_y==0;
 wire split_read=(WIDTH%2==0) && ((output_start?prepared_mode:output_mode)==0);
 wire [11:0] split_x=(output_x<WIDTH/2)?output_x:output_x-WIDTH/2;
 wire [AW:0] split_address={1'b0,row_read_base}+split_x;
 wire read_packed=RGB_STROKE_ENABLE && (output_start?prepared_mode:output_mode)>=1 && (output_start?prepared_mode:output_mode)<=4;
 wire [AW-1:0] read_address=split_read?
   ((split_address>=FDEPTH)?split_address-FDEPTH:split_address):(read_packed?rp[AW:1]:rp[AW-1:0]);
 reg [AW+1:0] occupancy;
 reg [31:0] produced_count;
 wire write_packed=RGB_STROKE_ENABLE && pixel_mode>=1 && pixel_mode<=4;
 wire new_frame=RGB_STROKE_ENABLE && pv && ps;
 wire [AW:0] wa=new_frame?{(AW+1){1'b0}}:wp;
 wire [AW+1:0] capacity=write_packed?2*FDEPTH:FDEPTH;
 wire push=pv && occupancy<capacity;
 wire take=pop && occupancy!=0;
 wire [AW-1:0] write_address=write_packed?wa[AW:1]:wa[AW-1:0];
 reg [5:0] gray_hold;
 wire memory_write=push && (!write_packed || wa[0] || pe);
 wire [15:0] write_word=write_packed?
   (wa[0]?{2'd0,pixel[10:5],2'd0,gray_hold}:{8'd0,2'd0,pixel[10:5]}):
   ((pixel_mode==0 && WIDTH%2==0)?{7'd0,compare_data}:pixel);
 always @(posedge clk)begin
   if(rst_n && push && write_packed && !wa[0])gray_hold<=pixel[10:5];
 end
 generate if(RGB_STROKE_ENABLE)begin: packed_storage
   reg [15:0] bank0[0:1023],bank1[0:1023],bank2[0:1023];
   reg [15:0] q0,q1,q2;reg [1:0] selected_bank;
   always @(posedge clk)begin
     if(rst_n && memory_write)case(write_address[11:10])
       0:bank0[write_address[9:0]]<=write_word;
       1:bank1[write_address[9:0]]<=write_word;
       2:bank2[write_address[9:0]]<=write_word;
     endcase
     if(rst_n && take)begin
       q0<=bank0[read_address[9:0]];q1<=bank1[read_address[9:0]];q2<=bank2[read_address[9:0]];selected_bank<=read_address[11:10];
     end
   end
   assign output_word=selected_bank==0?q0:selected_bank==1?q1:q2;
 end else begin: original_storage
   reg [15:0] storage[0:FDEPTH-1],q;
   always @(posedge clk)begin
     if(rst_n && memory_write)storage[write_address]<=write_word;
     if(rst_n && take)q<=storage[read_address];
   end
   assign output_word=q;
 end endgenerate
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin read_packed_q<=0;read_lane_q<=0;end
   else if(take)begin read_packed_q<=read_packed;read_lane_q<=rp[0];end
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin wp<=0;rp<=0;occupancy<=0;fault<=0;produced_count<=0;end
   else begin
     if(new_frame)begin
       if(occupancy!=0 || pop)fault<=1;
       wp<=1;rp<=0;occupancy<=push?1:0;
     end else begin
       if(push)wp<=(wa==capacity-1)?0:wa+1'b1;
       if(take)rp<=(rp==(read_packed?2*FDEPTH:FDEPTH)-1)?0:rp+1'b1;
       case({push,take})2'b10:occupancy<=occupancy+1'b1;2'b01:occupancy<=occupancy-1'b1;default:;endcase
     end
     if((pv && !push) || (pop && !take))fault<=1;
     if(pv)begin
       if(ps!=(produced_count==0) || pe!=(produced_count==N-1))fault<=1;
       produced_count<=(produced_count==N-1)?0:produced_count+1'b1;
     end
   end
 end
 // Split reads the same native-resolution left half twice at x=0..WIDTH/2-1. The
 // existing three-line prefill retains the row until its right half is read.
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     row_read_base<=0;prepared_mode<=0;output_mode<=0;
     prepared_threshold<=0;output_threshold<=0;
   end else begin
     if(pv && ps) begin if(RGB_STROKE_ENABLE)row_read_base<=0;prepared_mode<=pixel_mode;prepared_threshold<=pixel_threshold_bcd;end
     if(output_start) begin output_mode<=prepared_mode;output_threshold<=prepared_threshold;end
     if(pop && output_x==WIDTH-1)
       row_read_base<=((row_read_base+WIDTH)>=FDEPTH)?row_read_base+WIDTH-FDEPTH:row_read_base+WIDTH;
   end
 end
 wire [7:0] split_gray=output_pixel[8:1];
 wire [15:0] split_pixel=(display_x==WIDTH/2)?16'h07e0:
   (display_x<WIDTH/2)?{split_gray[7:3],split_gray[7:2],split_gray[7:3]}:{16{output_pixel[0]}};
 wire [15:0] split_with_osd;
 video_status_osd split_osd(.x(display_x),.y(display_y),.mode(3'd0),.shape(2'd0),
   .threshold_bcd(output_threshold),.rgb_in(split_pixel),.rgb_out(split_with_osd));
 wire [15:0] display_pixel=(output_mode==0 && WIDTH%2==0)?
   (OSD_ENABLE?split_with_osd:split_pixel):output_pixel;
 // Frame metadata and coordinates follow the synchronous FIFO read register.

 reg [15:0] status_frame;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin output_x<=0;output_y<=0;display_x<=0;display_y<=0;status_frame<=0;output_metadata<=0;end
   else if(pop) begin
     display_x<=output_x;display_y<=output_y;
     if(output_x==0 && output_y==0) begin status_frame<=history_status;output_metadata<=input_metadata;end
     if(output_x==WIDTH-1) begin output_x<=0;output_y<=(output_y==HEIGHT-1)?0:output_y+1'b1;end
     else output_x<=output_x+1'b1;
   end
 end
 display_frame_rate rate(
   .clk(clk),.rst_n(rst_n),.metadata(output_metadata),.fault(fault || !input_data_good),
   .frame_start(pop && output_x==0 && output_y==0),
   .frame_end(video_de && display_x==WIDTH-1 && display_y==HEIGHT-1),.fps(display_fps));
 wire [15:0] history_pixel;
 generate if(HISTORY_OSD) begin: g_history_osd
   frame_history_osd #(.HEIGHT(HEIGHT)) overlay(.x(display_x),.y(display_y),.status(status_frame),
     .rgb_in(display_pixel),.rgb_out(history_pixel));
 end else begin: g_no_history_osd
   assign history_pixel=display_pixel;
 end endgenerate
 // A visible fault prevents a shifted/partial frame being mistaken for good
 // output. Only reset clears it. This does not diagnose the old DDR controller.
 assign video_rgb=!video_de ? 24'd0 : fault ? 24'hff00ff :
   {history_pixel[15:11],history_pixel[15:13],history_pixel[10:5],history_pixel[10:9],history_pixel[4:0],history_pixel[4:2]};
endmodule

// Count complete, distinct content frames in the preceding one-second gate.
// A 16-bit content token may wrap; adjacent inequality remains correct because
// fewer than 65536 updates can occur between output frames. Static frames and
// repeated Canny computations of the same input/configuration do not count.
module display_frame_rate #(parameter CLOCK_HZ=74400000)(
 input clk,rst_n,frame_start,frame_end,fault,
 input [31:0] metadata,
 output reg [7:0] fps
);
 reg [31:0] second_count;
 reg [7:0] gate_frames;
 reg [15:0] previous_token;
 reg [4:0] previous_context;
 reg seen;
 wire context_changed=metadata[21:17]!=previous_context;
 wire new_frame=frame_end && metadata[16] && !fault && (!seen || metadata[15:0]!=previous_token);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin second_count<=0;gate_frames<=0;fps<=0;previous_token<=0;previous_context<=0;seen<=0;end
  else begin
   // output_metadata is registered at SOP; inspect it one cycle later. It
   // remains stable for the entire frame, including the EOF counted below.
   if(context_changed) begin
    previous_context<=metadata[21:17];second_count<=0;gate_frames<=0;fps<=0;
   end else begin
    if(second_count==CLOCK_HZ-1) begin
     second_count<=0;fps<=(gate_frames==255)?8'd255:gate_frames+new_frame;gate_frames<=0;
    end else begin
     second_count<=second_count+1'b1;
     if(new_frame && gate_frames!=255) gate_frames<=gate_frames+1'b1;
    end
    if(frame_end && metadata[16] && !fault) begin previous_token<=metadata[15:0];seen<=1;end
   end
  end
 end
endmodule
