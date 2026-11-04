// One action when a debounced gesture is released. Holding both keys forms a
// chord and switches mode without also changing the threshold.
module video_buttons #(
 parameter DEBOUNCE_CYCLES=1920000,INITIAL_THRESHOLD=160,STEP=16,MODE_COUNT=6,MAX_THRESHOLD=800
)(
 input clk,rst_n,
 input [1:0] key_n,
 output reg [10:0] threshold,
 output reg [2:0] mode
);
 wire [1:0] pressed;
 reg [1:0] gesture;
 button_debounce #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) key_plus(
 .clk(clk),.rst_n(rst_n),.button_n(key_n[0]),.pressed(pressed[0]));
 button_debounce #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) key_minus(
 .clk(clk),.rst_n(rst_n),.button_n(key_n[1]),.pressed(pressed[1]));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin gesture<=0;threshold<=INITIAL_THRESHOLD;mode<=0;end
   else if(pressed!=0) gesture<=gesture|pressed;
   else if(gesture!=0) begin
     gesture<=0;
     case(gesture)
       2'b01: threshold<=(threshold>MAX_THRESHOLD-STEP)?MAX_THRESHOLD:threshold+STEP;
       2'b10: threshold<=(threshold<STEP)?11'd0:threshold-STEP;
       2'b11: mode<=(mode==MODE_COUNT-1)?3'd0:mode+1'b1;
     endcase
   end
 end
endmodule
