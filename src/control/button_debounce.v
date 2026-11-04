// Synchronize an active-low board button; require a continuous stable interval.
module button_debounce #(parameter STABLE_CYCLES=1920000)(
 input clk,rst_n,button_n,
 output reg pressed
);
 localparam CW=(STABLE_CYCLES<2)?1:$clog2(STABLE_CYCLES);
 reg button_meta,button_sync;
 reg [CW-1:0] count;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin button_meta<=1;button_sync<=1;pressed<=0;count<=0;end
   else begin
     button_meta<=button_n;button_sync<=button_meta;
     if((!button_sync)==pressed) count<=0;
     else if(count==STABLE_CYCLES-1) begin pressed<=!button_sync;count<=0;end
     else count<=count+1'b1;
   end
 end
endmodule
