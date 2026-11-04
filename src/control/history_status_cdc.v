// Bundled-data request/acknowledge mailbox. payload_hold stays unchanged while
// the request crosses two destination flops and until acknowledgement returns.
// An additional three destination cycles allow bundled-data routing to settle.
// Arbitrary simultaneous changes of threshold/mode therefore arrive atomically.
// The destination then samples controls at each algorithm/output frame SOP.
// Both resets must originate from the same system reset (release may differ).
module history_status_cdc #(parameter WIDTH=16)(
 input source_clk,source_rst_n,
 input [WIDTH-1:0] data_in,
 input dest_clk,dest_rst_n,
 output reg [WIDTH-1:0] data_out
);
 reg [WIDTH-1:0] payload_hold;
 reg request,ack_meta,ack_sync;
 reg acknowledge,request_meta,request_sync;
 reg [1:0] settle;
 wire [WIDTH-1:0] wanted=data_in;
 always @(posedge source_clk or negedge source_rst_n) begin
   if(!source_rst_n) begin
     payload_hold<=16'd0;
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
     data_out<=0;
   end else begin
     request_meta<=request;request_sync<=request_meta;
     if(request_sync!=acknowledge) begin
       if(settle==0) settle<=3;
       else if(settle==1) begin
         data_out<=payload_hold;
         acknowledge<=request_sync;settle<=0;
       end else settle<=settle-1'b1;
     end
   end
 end
endmodule
