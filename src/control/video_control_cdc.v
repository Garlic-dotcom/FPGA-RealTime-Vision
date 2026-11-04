// Bundled-data request/acknowledge mailbox. payload_hold stays unchanged while
// the request crosses two destination flops and until acknowledgement returns.
// An additional three destination cycles allow bundled-data routing to settle.
// Arbitrary simultaneous changes of threshold/mode therefore arrive atomically.
// The destination then samples controls at each algorithm/output frame SOP.
// Both resets must originate from the same system reset (release may differ).
module video_control_cdc #(
 parameter INITIAL_THRESHOLD=160
)(
 input source_clk,source_rst_n,
 input [10:0] threshold_in,
 input [2:0] mode_in,
 input dest_clk,dest_rst_n,
 output reg [10:0] threshold_out,
 output reg [2:0] mode_out
);
 localparam [10:0] INITIAL_VALUE=INITIAL_THRESHOLD;
 reg [13:0] payload_hold;
 reg request,ack_meta,ack_sync;
 reg acknowledge,request_meta,request_sync;
 reg [1:0] settle;
 wire [13:0] wanted={mode_in,threshold_in};
 always @(posedge source_clk or negedge source_rst_n) begin
   if(!source_rst_n) begin
     payload_hold<={3'd0,INITIAL_VALUE};
     request<=0;ack_meta<=0;ack_sync<=0;
   end else begin
     ack_meta<=acknowledge;ack_sync<=ack_meta;
     if(request==ack_sync && payload_hold!=wanted) begin
       payload_hold<=wanted;request<=!request;
     end
   end
 end
 always @(posedge dest_clk or negedge dest_rst_n) begin
   if(!dest_rst_n) begin
     acknowledge<=0;request_meta<=0;request_sync<=0;settle<=0;
     threshold_out<=INITIAL_THRESHOLD;mode_out<=0;
   end else begin
     request_meta<=request;request_sync<=request_meta;
     if(request_sync!=acknowledge) begin
       if(settle==0) settle<=3;
       else if(settle==1) begin
         {mode_out,threshold_out}<=payload_hold;
         acknowledge<=request_sync;settle<=0;
       end else settle<=settle-1'b1;
     end
   end
 end
endmodule
