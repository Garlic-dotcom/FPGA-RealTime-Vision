// Ordered scratch read shim. Only complete, aligned label blocks are prefetched;
// queue reads and label tails retain the original single-beat transaction.
// Four clean 64-byte blocks reduce repeated DDR read startup. Successful scratch
// AW handshakes invalidate the matching block. New reads wait for same-block
// earlier B/offered AW; unrelated writes retain the original safe overlap.
// The first label read of each task is single-beat. Its measured AR-handshake
// to first-RVALID elapsed cycles select prefetch only for startup >=8 cycles.
// Short-startup peers retain the original transparent single-beat R path.
module canny_label_read_prefetch #(
 parameter LABEL_BYTES=1280*720, CACHE_BLOCKS=4
)(
 input clk,rst_n,cache_clear,
 input [31:0] label_base,
 output reg fault,
 input [31:0] write_awaddr,
 input write_awvalid,write_awready,write_bvalid,write_bready,
 input [1:0] write_bresp,
 input [31:0] upstream_araddr,
 input [7:0] upstream_arlen,
 input [2:0] upstream_arsize,
 input [1:0] upstream_arburst,
 input upstream_arvalid,
 output upstream_arready,
 output [127:0] upstream_rdata,
 output [1:0] upstream_rresp,
 output upstream_rlast,upstream_rvalid,
 input upstream_rready,
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
 localparam BIW=(CACHE_BLOCKS>1)?$clog2(CACHE_BLOCKS):1;
 localparam IDLE=0,ADDRESS=1,DATA=2,REPLY=3,FAILED=4,PASS_DATA=5;
 localparam CALIB=0,PASS=1,PREFETCH=2;
 reg [2:0] state;
 reg [1:0] read_mode;
 reg calibrating_read,calibration_active;
 reg [7:0] calibration_elapsed;
 reg configured;
 reg [31:0] labels;
 reg [32:0] labels_end;
 reg [CACHE_BLOCKS-1:0] valid;
 reg [25:0] tags[0:CACHE_BLOCKS-1];
 reg [127:0] lines[0:CACHE_BLOCKS*4-1];
 reg [BIW-1:0] replacement,fill_index,hit_index;
 reg hit;
 reg [31:0] read_address;
 reg [1:0] requested_lane,beat;
 reg burst,fill_invalidated;
 reg [127:0] response;
 reg [1:0] writes_pending;
 reg [25:0] pending_write_block;
 integer lookup,invalidate;
 wire aw_fire=write_awvalid && write_awready;
 wire b_fire=write_bvalid && write_bready;
 wire b_error_now=b_fire && write_bresp!=0;
 wire [31:0] block_address={upstream_araddr[31:6],6'b0};
 wire label_request=configured && upstream_araddr>=labels &&
     {1'b0,upstream_araddr}<labels_end;
 wire complete_block=label_request && block_address>=labels &&
     ({1'b0,block_address}+33'd64)<=labels_end;
 wire bad_request=upstream_araddr[3:0]!=0 || upstream_arlen!=0 ||
     upstream_arsize!=4 || upstream_arburst!=1;
 wire fill_write_collision=aw_fire && write_awaddr[31:6]==read_address[31:6];
 always @* begin
   hit=0;hit_index=0;
   for(lookup=0;lookup<CACHE_BLOCKS;lookup=lookup+1)
     if(valid[lookup] && tags[lookup]==upstream_araddr[31:6]) begin
       hit=1;hit_index=lookup;
     end
 end
 // Compare the whole 64-byte region, including an adjacent dirty victim. A
 // different block cannot modify any beat returned by this prefetch, so its
 // write/B may proceed concurrently as in the original scratch bridge.
 wire pending_write_conflict=writes_pending!=0 &&
     pending_write_block==upstream_araddr[31:6];
 wire offered_write_conflict=write_awvalid &&
     write_awaddr[31:6]==upstream_araddr[31:6];
 wire read_admission=rst_n && configured && !fault && !cache_clear &&
     !b_error_now && state==IDLE && !pending_write_conflict && !offered_write_conflict;
 wire transparent_read=read_mode==PASS || (read_mode==CALIB && !label_request);
 wire use_prefetch=read_mode==PREFETCH && complete_block;
 assign upstream_arready=read_admission &&
     (bad_request || !transparent_read || axi_arready);
 wire idle_read_miss=upstream_arvalid && read_admission && !bad_request &&
     !(use_prefetch && hit);
 wire pass_response=state==PASS_DATA;
 wire bad_pass_response=axi_rvalid && (axi_rresp!=0 || !axi_rlast);
 assign upstream_rdata=pass_response?axi_rdata:response;
 assign upstream_rresp=pass_response?axi_rresp:2'b0;
 assign upstream_rlast=pass_response?axi_rlast:1'b1;
 assign upstream_rvalid=rst_n && !fault && !b_error_now &&
     (state==REPLY || (pass_response && axi_rvalid && !bad_pass_response));
 // Pass an idle legal miss directly to AR. If DDR stalls, the acceptance
 // edge latches this exact address/length and ADDRESS keeps them stable.
 assign axi_araddr=idle_read_miss?(use_prefetch?block_address:upstream_araddr):read_address;
 assign axi_arlen=idle_read_miss?(use_prefetch?8'd3:8'd0):(burst?8'd3:8'd0);
 assign axi_arsize=4;
 assign axi_arburst=1;
 assign axi_arvalid=rst_n && !fault && (state==ADDRESS || idle_read_miss);
 assign axi_rready=rst_n && !fault &&
     (state==DATA || (pass_response && upstream_rready && !b_error_now));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=IDLE;configured<=0;fault<=0;labels<=0;labels_end<=0;
     read_mode<=CALIB;calibrating_read<=0;calibration_active<=0;calibration_elapsed<=0;
     valid<=0;replacement<=0;fill_index<=0;
     read_address<=0;requested_lane<=0;beat<=0;burst<=0;
     fill_invalidated<=0;response<=0;writes_pending<=0;pending_write_block<=0;
   end else begin
     if(cache_clear) begin
       if(state!=IDLE || upstream_arvalid || writes_pending!=0 || write_awvalid ||
          label_base[3:0]!=0 || LABEL_BYTES<1 || CACHE_BLOCKS<1 ||
          ({1'b0,label_base}+LABEL_BYTES)>33'h100000000)
         begin fault<=1;state<=FAILED;end
       else begin
         configured<=1;labels<=label_base;
         labels_end<={1'b0,label_base}+LABEL_BYTES;
         valid<=0;replacement<=0;
         read_mode<=CALIB;calibrating_read<=0;calibration_active<=0;calibration_elapsed<=0;
       end
     end else if(!fault) begin
       case({aw_fire,b_fire})
         2'b10:begin
           if(writes_pending!=0) begin fault<=1;state<=FAILED;end
           else writes_pending<=1;
         end
         2'b01:begin
           if(writes_pending==0) begin fault<=1;state<=FAILED;end
           else writes_pending<=0;
         end
       endcase
       if(aw_fire) pending_write_block<=write_awaddr[31:6];
       if(b_error_now) begin fault<=1;state<=FAILED;end
       // Count elapsed clock edges after the external AR handshake, excluding
       // AR backpressure and independent of upstream RREADY. A peer offering
       // RVALID on the AR edge measures zero; on the next edge it measures one.
       if(calibration_active) begin
         if(axi_rvalid) begin
           read_mode<=calibration_elapsed>=7?PREFETCH:PASS;
           calibration_active<=0;
         end else if(calibration_elapsed!=255)
           calibration_elapsed<=calibration_elapsed+1'b1;
       end
       case(state)
         IDLE:if(upstream_arvalid && upstream_arready) begin
           if(bad_request) begin fault<=1;state<=FAILED;end
           else if(use_prefetch && hit) begin
             response<=lines[(hit_index*4)+upstream_araddr[5:4]];
             state<=REPLY;
           end else begin
             burst<=use_prefetch;
             read_address<=use_prefetch?block_address:upstream_araddr;
             requested_lane<=use_prefetch?upstream_araddr[5:4]:2'd0;
             calibrating_read<=read_mode==CALIB && label_request;
             beat<=0;fill_invalidated<=0;fill_index<=replacement;
             if(use_prefetch) valid[replacement]<=0;
             if(transparent_read) state<=PASS_DATA;
             else state<=axi_arready?DATA:ADDRESS;
             if(axi_arready && read_mode==CALIB && label_request) begin
               calibration_elapsed<=0;
               calibration_active<=!axi_rvalid;
               if(axi_rvalid) read_mode<=PASS;
             end
           end
         end
         ADDRESS:if(axi_arready) begin
           state<=DATA;
           if(calibrating_read) begin
             calibration_elapsed<=0;calibration_active<=!axi_rvalid;
             if(axi_rvalid) read_mode<=PASS;
           end
         end
         DATA:if(axi_rvalid) begin
           // A missing/early last or any failed beat invalidates the whole
           // speculative block. No demand response is published before last.
           if(axi_rresp!=0 || axi_rlast!=(burst?(beat==3):1'b1))
             begin fault<=1;valid[fill_index]<=0;state<=FAILED;end
           else begin
             if(beat==requested_lane) response<=axi_rdata;
             if(burst) lines[(fill_index*4)+beat]<=axi_rdata;
             if(!burst || beat==3) begin
               if(burst) begin
                 tags[fill_index]<=read_address[31:6];
                 valid[fill_index]<=!fill_invalidated && !fill_write_collision;
                 replacement<=(replacement==CACHE_BLOCKS-1)?0:replacement+1'b1;
               end
               state<=REPLY;
             end else beat<=beat+1'b1;
           end
         end
         REPLY:if(upstream_rready) state<=IDLE;
         PASS_DATA:if(axi_rvalid) begin
           if(bad_pass_response) begin fault<=1;valid<=0;state<=FAILED;end
           else if(upstream_rready) state<=IDLE;
         end
         FAILED:state<=FAILED;
         default:begin fault<=1;state<=FAILED;end
       endcase
       if(aw_fire) begin
         for(invalidate=0;invalidate<CACHE_BLOCKS;invalidate=invalidate+1)
           if(valid[invalidate] && tags[invalidate]==write_awaddr[31:6])
             valid[invalidate]<=0;
         if(state==ADDRESS || state==DATA)
           if(fill_write_collision) fill_invalidated<=1;
       end
       // Fault has precedence over a simultaneous successful read/hit.
       if(b_error_now || (aw_fire && b_fire && writes_pending==0) ||
          (aw_fire && !b_fire && writes_pending!=0) ||
          (b_fire && !aw_fire && writes_pending==0)) begin
         fault<=1;state<=FAILED;valid<=0;
       end
     end
   end
 end
endmodule
