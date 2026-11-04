// LIVE-only adaptive grayscale history. Each DDR pixel stores a 10-bit Q2
// accumulator in a padded 16-bit lane, with the ordinary DMA lane ordering.
// Differences > 8 gray levels replace history immediately; small differences
// use rounded (3*history + 4*current)/4. Fractional state avoids +/-1 lock-in.
// Caller owns the disjoint history region until done, including the final B.
// A failed or interrupted job invalidates history; reset the shared bus on fault.
module canny_temporal_gray #(
 parameter WIDTH=1280,HEIGHT=720,HISTORY_BASE=32'h09c00000
)(
 input clk,rst_n,start,enable,history_valid,
 output reg busy,done,fault,
 input sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 output sink_ready,
 output source_valid,source_sop,source_eop,
 output [7:0] source_data,
 input source_ready,
 output [31:0] axi_awaddr,
 output [7:0] axi_awlen,
 output [2:0] axi_awsize,
 output [1:0] axi_awburst,
 output axi_awvalid,
 input axi_awready,
 output [127:0] axi_wdata,
 output [15:0] axi_wstrb,
 output axi_wlast,axi_wvalid,
 input axi_wready,
 input [1:0] axi_bresp,
 input axi_bvalid,
 output axi_bready,
 output [31:0] axi_araddr,
 output [7:0] axi_arlen,
 output [2:0] axi_arsize,
 output [1:0] axi_arburst,
 output axi_arvalid,
 input axi_arready,
 input [127:0] axi_rdata,
 input [1:0] axi_rresp,
 input axi_rlast,axi_rvalid,
 output axi_rready
);
 localparam PIXELS=WIDTH*HEIGHT;
 wire launch=start && !busy && !fault;
 // Named-port legacy testbenches may omit the new job controls. Only an
 // explicit asserted enable can access history; all other cases bypass it.
 wire requested_enable=(enable===1'b1);
 wire requested_history=(history_valid===1'b1);
 reg active_enable,using_history,saw_last,saw_reader,saw_writer;
 reg [31:0] pixel_index;
 wire hv,hs,he,hr,rdone,rfault,wr,wdone,wfault;
 wire [15:0] history_data;
 wire [10:0] rounded_history={1'b0,history_data[9:0]}+11'd2;
 wire [7:0] previous_gray=rounded_history[9:2];
 wire [8:0] difference=(sink_data>=previous_gray)?
     {1'b0,sink_data}-{1'b0,previous_gray}:
     {1'b0,previous_gray}-{1'b0,sink_data};
 wire [9:0] current_q2={sink_data,2'b00};
 wire [11:0] weighted_q2={2'b00,history_data[9:0]}+
     ({2'b00,history_data[9:0]}<<1)+{2'b00,current_q2}+12'd2;
 wire [9:0] next_q2=(using_history && difference<=9'd8)?weighted_q2[11:2]:current_q2;
 wire [10:0] rounded_next={1'b0,next_q2}+11'd2;
 wire last_pixel=(pixel_index==PIXELS-1);
 wire packet_ok=(sink_sop==(pixel_index==0)) && (sink_eop==last_pixel);
 wire history_ok=!using_history ||
     ((hs==(pixel_index==0)) && (he==last_pixel) && history_data[15:10]==0);
 wire available=rst_n && busy && !fault && !saw_last &&
     (!using_history || hv) && (!active_enable || wr);
 assign sink_ready=available && source_ready;
 assign source_valid=available && sink_valid && packet_ok && history_ok;
 assign source_sop=sink_sop;
 assign source_eop=sink_eop;
 assign source_data=active_enable?rounded_next[9:2]:sink_data;
 wire transfer=source_valid && source_ready;
 wire last_accept=transfer && last_pixel;
 assign hr=transfer && using_history;
 axi_frame_reader #(.PIXELS(PIXELS)) history_reader(
 .clk(clk),.rst_n(rst_n),.start(launch && requested_enable && requested_history),
 .frame_base(HISTORY_BASE),.split_frame(1'b0),.second_base(32'd0),
 .busy(),.done(rdone),.fault(rfault),
 .source_valid(hv),.source_sop(hs),.source_eop(he),.source_data(history_data),.source_ready(hr),
 .axi_araddr(axi_araddr),.axi_arlen(axi_arlen),.axi_arsize(axi_arsize),
 .axi_arburst(axi_arburst),.axi_arvalid(axi_arvalid),.axi_arready(axi_arready),
 .axi_rdata(axi_rdata),.axi_rresp(axi_rresp),.axi_rlast(axi_rlast),
 .axi_rvalid(axi_rvalid),.axi_rready(axi_rready));
 canny_temporal_writer #(.PIXELS(PIXELS)) history_writer(
 .clk(clk),.rst_n(rst_n),.start(launch && requested_enable),.frame_base(HISTORY_BASE),
 .busy(),.done(wdone),.fault(wfault),
 .sink_valid(transfer && active_enable),.sink_sop(sink_sop),.sink_eop(sink_eop),
 .sink_data({6'd0,next_q2}),.sink_ready(wr),
 .axi_awaddr(axi_awaddr),.axi_awlen(axi_awlen),.axi_awsize(axi_awsize),
 .axi_awburst(axi_awburst),.axi_awvalid(axi_awvalid),.axi_awready(axi_awready),
 .axi_wdata(axi_wdata),.axi_wstrb(axi_wstrb),.axi_wlast(axi_wlast),
 .axi_wvalid(axi_wvalid),.axi_wready(axi_wready),
 .axi_bresp(axi_bresp),.axi_bvalid(axi_bvalid),.axi_bready(axi_bready));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     busy<=0;done<=0;fault<=0;active_enable<=0;using_history<=0;
     saw_last<=0;saw_reader<=0;saw_writer<=0;pixel_index<=0;
   end else begin
     done<=0;
     if(launch) begin
       busy<=1;active_enable<=requested_enable;
       using_history<=requested_enable && requested_history;
       saw_last<=0;saw_reader<=0;saw_writer<=0;pixel_index<=0;
     end
     if(transfer) begin
       pixel_index<=pixel_index+1'b1;
       if(last_pixel) saw_last<=1;
     end
     if(rdone) saw_reader<=1;
     if(wdone) saw_writer<=1;
     if(busy && (saw_last || last_accept) &&
         (!using_history || saw_reader || rdone) &&
         (!active_enable || saw_writer || wdone)) begin
       busy<=0;done<=1;
     end
     if(rfault || wfault || (sink_valid && sink_ready && (!packet_ok || !history_ok))) begin
       busy<=0;done<=0;fault<=1;
     end
   end
 end
endmodule

// Two pixel lanes per RAM word reduce serial packing during AXI ownership.
// Bank capacity, burst addressing, tail strobes and final-B ownership are unchanged.
module canny_temporal_writer #(parameter PIXELS=1280*720,MAX_BURST=128)(
 input clk,rst_n,start,
 input [31:0] frame_base,
 output reg busy,done,fault,
 input sink_valid,sink_sop,sink_eop,
 input [15:0] sink_data,
 output sink_ready,
 output [31:0] axi_awaddr,
 output [7:0] axi_awlen,
 output [2:0] axi_awsize,
 output [1:0] axi_awburst,
 output axi_awvalid,
 input axi_awready,
 output [127:0] axi_wdata,
 output [15:0] axi_wstrb,
 output axi_wlast,axi_wvalid,
 input axi_wready,
 input [1:0] axi_bresp,
 input axi_bvalid,
 output axi_bready
);
 localparam BEATS=(PIXELS+7)/8;
 localparam WORDS=MAX_BURST*4;
 localparam WORD_ADDR_BITS=(WORDS>1)?$clog2(WORDS):1;
 localparam TAIL_PIXELS=PIXELS%2;
 localparam S_IDLE=0,S_FETCH=1,S_PACK=2,S_SEND=3,S_RESPONSE=4,S_FAILED=5;
 reg [2:0] send_state,lane;
 reg [1:0] pack_lane;
 reg [31:0] collect_address,beats_left,pixel_index;
 reg [8:0] collected,send_index;
 reg [15:0] collect_prefix;
 reg [127:0] pack_data;
 reg collect_bank,send_bank,collecting_done,aw_pending;
 reg [1:0] bank_full;
 reg [31:0] bank_address[0:1];
 reg [8:0] bank_beats[0:1];
 reg [15:0] bank_strobe[0:1];
 reg bank_final[0:1];
 reg [31:0] bank0_mem[0:WORDS-1];
 reg [31:0] bank1_mem[0:WORDS-1];
 reg [31:0] bank0_dout,bank1_dout;
 wire accept=sink_valid && sink_ready;
 wire last_pixel=(pixel_index==PIXELS-1);
 wire finish_word=(lane==7 || last_pixel);
 wire packet_ok=(sink_sop==(pixel_index==0)) && (sink_eop==last_pixel);
 wire packet_error_now=accept && !packet_ok;
 wire [8:0] page_beats=9'd256-{1'b0,collect_address[11:4]};
 wire [31:0] limited_beats=(beats_left>MAX_BURST)?MAX_BURST:beats_left;
 wire [8:0] target_beats=(limited_beats>page_beats)?page_beats:limited_beats[8:0];
 wire write_pixel=accept && packet_ok;
 wire write_word=write_pixel && (lane[0] || last_pixel);
 wire [WORD_ADDR_BITS-1:0] write_index={collected,lane[2:1]};
 // The only partial group is the final group. Its length is a constant parameter,
 // so ordinary 720p collection has no dynamic shift in the gray-to-RAM path.
 wire [31:0] write_data=(last_pixel && TAIL_PIXELS==1)?{sink_data,16'd0}:
     {collect_prefix,sink_data};
 wire read_word=send_state==S_FETCH || (send_state==S_PACK && pack_lane<3) ||
     (send_state==S_SEND && axi_wready && !axi_wlast);
 wire [8:0] next_send_index=send_index+1'b1;
 wire [WORD_ADDR_BITS-1:0] read_index=(send_state==S_FETCH)?{send_index,2'b00}:
     (send_state==S_SEND)?{next_send_index,2'b00}:{send_index,2'b00}+pack_lane+1'b1;
 wire [31:0] read_data=send_bank?bank1_dout:bank0_dout;
 wire tail_beat=bank_final[send_bank] && send_index==bank_beats[send_bank]-1'b1;
 wire [3:0] word_strobe=bank_strobe[send_bank][15-4*pack_lane-:4];
 // Unused RAM words may be uninitialized. Mask every pixel lane,
 // including that word, rather than relying on WSTRB to hide unknown WDATA.
 wire [31:0] packed_word={
     ((!tail_beat || word_strobe[2])?read_data[31:16]:16'd0),
     ((!tail_beat || word_strobe[0])?read_data[15:0]:16'd0)};
 assign sink_ready=rst_n && busy && !fault && !collecting_done && !bank_full[collect_bank];
 assign axi_awaddr=bank_address[send_bank];
 assign axi_awlen=bank_beats[send_bank]-1'b1;
 assign axi_awsize=3'd4;
 assign axi_awburst=2'b01;
 assign axi_awvalid=rst_n && aw_pending;
 assign axi_wvalid=rst_n && send_state==S_SEND;
 assign axi_wlast=(send_index==bank_beats[send_bank]-1'b1);
 assign axi_wstrb=(bank_final[send_bank] && axi_wlast)?bank_strobe[send_bank]:16'hffff;
 assign axi_wdata=pack_data;
 assign axi_bready=rst_n && send_state==S_RESPONSE && !aw_pending;
 always @(posedge clk) begin
   if(write_word && !collect_bank) bank0_mem[write_index]<=write_data;
   if(read_word && !send_bank) bank0_dout<=bank0_mem[read_index];
 end
 always @(posedge clk) begin
   if(write_word && collect_bank) bank1_mem[write_index]<=write_data;
   if(read_word && send_bank) bank1_dout<=bank1_mem[read_index];
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     send_state<=S_IDLE;busy<=0;done<=0;fault<=0;lane<=0;pack_lane<=0;collect_prefix<=0;
     collect_address<=0;beats_left<=0;pixel_index<=0;collected<=0;send_index<=0;
     pack_data<=0;collect_bank<=0;send_bank<=0;collecting_done<=0;aw_pending<=0;bank_full<=0;
     bank_address[0]<=0;bank_address[1]<=0;bank_beats[0]<=0;bank_beats[1]<=0;
     bank_strobe[0]<=16'hffff;bank_strobe[1]<=16'hffff;bank_final[0]<=0;bank_final[1]<=0;
   end else begin
     done<=0;
     if(axi_awvalid && axi_awready) aw_pending<=0;
     if(!busy && !fault && start && send_state==S_IDLE) begin
       if(frame_base[3:0]!=0) begin fault<=1;send_state<=S_FAILED;end
       else begin
         busy<=1;collect_address<=frame_base;beats_left<=BEATS;pixel_index<=0;
         lane<=0;collected<=0;send_index<=0;pack_lane<=0;collect_prefix<=0;pack_data<=0;
         collect_bank<=0;send_bank<=0;collecting_done<=0;bank_full<=0;aw_pending<=0;
       end
     end else begin
       if(packet_error_now) begin fault<=1;busy<=0;end
       if(accept && packet_ok) begin
         collect_prefix<=sink_data;
         pixel_index<=pixel_index+1'b1;lane<=lane+1'b1;
         if(finish_word) begin
           lane<=0;collected<=collected+1'b1;
           if(collected==target_beats-1'b1) begin
             bank_full[collect_bank]<=1;bank_address[collect_bank]<=collect_address;
             bank_beats[collect_bank]<=target_beats;
             bank_strobe[collect_bank]<=last_pixel?(16'hffff<<(2*(7-lane))):16'hffff;
             bank_final[collect_bank]<=last_pixel;
             collect_address<=collect_address+{19'd0,target_beats,4'd0};
             beats_left<=beats_left-target_beats;collected<=0;collect_bank<=!collect_bank;
             if(last_pixel) collecting_done<=1;
           end
         end
       end
       case(send_state)
         S_IDLE:begin
           if(fault || packet_error_now) send_state<=S_FAILED;
           else if(bank_full[send_bank]) begin
             send_index<=0;pack_lane<=0;pack_data<=0;aw_pending<=1;send_state<=S_FETCH;
           end
         end
         S_FETCH:send_state<=S_PACK;
         S_PACK:begin
           pack_data<={pack_data[95:0],packed_word};pack_lane<=pack_lane+1'b1;
           if(pack_lane==3) send_state<=S_SEND;
         end
         S_SEND:if(axi_wready) begin
           if(axi_wlast) send_state<=S_RESPONSE;
           else begin
             send_index<=send_index+1'b1;pack_lane<=0;pack_data<=0;send_state<=S_PACK;
           end
         end
         S_RESPONSE:if(axi_bvalid && axi_bready) begin
           if(axi_bresp!=0) begin fault<=1;busy<=0;send_state<=S_FAILED;end
           else begin
             bank_full[send_bank]<=0;
             if(fault || packet_error_now) begin busy<=0;send_state<=S_FAILED;end
             else if(bank_final[send_bank]) begin busy<=0;done<=1;send_state<=S_IDLE;end
             else begin send_bank<=!send_bank;send_state<=S_IDLE;end
           end
         end
         S_FAILED:busy<=0;
         default:begin fault<=1;busy<=0;send_state<=S_FAILED;end
       endcase
     end
   end
 end
endmodule
