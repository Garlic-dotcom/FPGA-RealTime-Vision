// One frame of packed RGB565 from 128-bit AXI. Pixel zero occupies bits127:112,
// matching the existing W0_FIFO ENDIANESS=0 packing. Base is 16-byte aligned.
// Arbitrary frame pixel count supported; unused lanes in the last beat ignored.
// Bursts never cross 4 KiB. Consumer may stall at any pixel; all outputs hold.
// A complete burst reserves buffer space BEFORE AR. This decouples R transfers
// from serial pixel output, so a slow display does not monopolize the read bus.
// BUFFER_BEATS is a power of two, at least MAX_BURST (prefer >=2*MAX_BURST).
// Controller and consumer share clk. Caller owns the frame until done/fault.
module axi_frame_reader #(parameter PIXELS=1280*720,MAX_BURST=128,BUFFER_BEATS=256,SPLIT_ENABLE=0)(
 input clk,rst_n,start,
 input [31:0] frame_base,
 input split_frame,
 input [31:0] second_base,
 output reg busy,done,fault,
 output [31:0] axi_araddr,
 output [7:0] axi_arlen,
 output [2:0] axi_arsize,
 output [1:0] axi_arburst,
 output axi_arvalid,
 input axi_arready,
 input [127:0] axi_rdata,
 input [1:0] axi_rresp,
 input axi_rlast,axi_rvalid,
 output axi_rready,
 output source_valid,source_sop,source_eop,
 output [15:0] source_data,
 input source_ready
);
 localparam BEATS=(PIXELS+7)/8;
 localparam IDLE=0,ADDRESS=1,RECEIVE=2,DRAIN=3,FAILED=4;
 reg [2:0] state;
 reg [31:0] next_address,beats_left,pixel_index;
 reg active_split;
 reg [31:0] active_second;
 reg [8:0] burst_size,received;
 reg [127:0] beat_data;
 reg beat_valid;
 reg [2:0] lane;
 reg [$clog2(BUFFER_BEATS):0] buffered_beats;
 wire consume=source_valid && source_ready;
 wire last_pixel=(pixel_index==PIXELS-1);
 wire release_beat=consume && (lane==7 || last_pixel);
 wire [8:0] page_beats=9'd256-{1'b0,next_address[11:4]};
 wire [31:0] segment_left=(active_split && beats_left>BEATS/2)?beats_left-BEATS/2:beats_left;
 wire [31:0] limited_beats=(segment_left>MAX_BURST)?MAX_BURST:segment_left;
 wire [8:0] issue_beats=(limited_beats>page_beats)?page_beats:limited_beats[8:0];
 wire [31:0] occupancy=buffered_beats;
 wire space_for_burst=occupancy+issue_beats<=BUFFER_BEATS;
 wire fifo_ready,fifo_valid;
 wire [127:0] fifo_data;
 wire accept_read=axi_rvalid && axi_rready;
 wire good_read=accept_read && axi_rresp==0 && axi_rlast==(received==burst_size-1'b1);
 wire take_beat=fifo_valid && (!beat_valid || release_beat) && !fault;
 stream_fifo #(.DATA_WIDTH(128),.DEPTH(BUFFER_BEATS)) burst_buffer(
 .clk(clk),.rst_n(rst_n),.sink_valid(good_read),.sink_data(axi_rdata),.sink_ready(fifo_ready),
 .source_valid(fifo_valid),.source_data(fifo_data),.source_ready((!beat_valid || release_beat)&&!fault));
 assign axi_araddr=next_address;
 assign axi_arlen=issue_beats-1'b1;
 assign axi_arsize=3'd4;
 assign axi_arburst=2'b01;
 assign axi_arvalid=rst_n && state==ADDRESS && space_for_burst;
 assign axi_rready=rst_n && state==RECEIVE && fifo_ready;
 assign source_valid=rst_n && beat_valid && !fault;
 assign source_sop=source_valid && pixel_index==0;
 assign source_eop=source_valid && last_pixel;
 assign source_data=beat_data[127:112];
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=IDLE;busy<=0;done<=0;fault<=0;next_address<=0;beats_left<=0;
     pixel_index<=0;burst_size<=0;received<=0;beat_data<=0;beat_valid<=0;lane<=0;buffered_beats<=0;
     active_split<=0;active_second<=0;
   end else begin
     done<=0;
     if(consume) begin
       pixel_index<=pixel_index+1'b1;beat_data<={beat_data[111:0],16'd0};lane<=lane+1'b1;
       if(release_beat) begin beat_valid<=0;lane<=0;end
       if(last_pixel) begin busy<=0;done<=1;state<=IDLE;end
     end
     if(take_beat) begin beat_data<=fifo_data;beat_valid<=1;lane<=0;end
     case({good_read,release_beat})
       2'b10:buffered_beats<=buffered_beats+1'b1;
       2'b01:buffered_beats<=buffered_beats-1'b1;
     endcase
     case(state)
       IDLE:if(start) begin
         if(frame_base[3:0]!=0 || (SPLIT_ENABLE && split_frame && (second_base[3:0]!=0 || PIXELS%16!=0))) begin fault<=1;state<=FAILED;end
         else begin
           busy<=1;next_address<=frame_base;beats_left<=BEATS;pixel_index<=0;
           active_split<=SPLIT_ENABLE && split_frame;active_second<=second_base;
           beat_valid<=0;lane<=0;buffered_beats<=0;state<=ADDRESS;
         end
       end
       ADDRESS:if(axi_arvalid && axi_arready) begin
         burst_size<=issue_beats;received<=0;state<=RECEIVE;
       end
       RECEIVE:if(axi_rvalid && axi_rready) begin
         if(axi_rresp!=0 || axi_rlast!=(received==burst_size-1'b1)) begin
           fault<=1;busy<=0;beat_valid<=0;state<=FAILED;
         end else begin
           received<=received+1'b1;
           if(received==burst_size-1'b1) begin
             next_address<=(active_split && beats_left>BEATS/2 && beats_left-burst_size==BEATS/2)?
                 active_second+PIXELS:next_address+{19'd0,burst_size,4'd0};
             beats_left<=beats_left-burst_size;
             state<=(beats_left==burst_size)?DRAIN:ADDRESS;
           end
         end
       end
       DRAIN:begin end // Last beat drains via source_ready before done.
       FAILED:begin busy<=0;beat_valid<=0;end // Shared bus must reset on fault.
       default:begin fault<=1;busy<=0;state<=FAILED;end
     endcase
   end
 end
endmodule
