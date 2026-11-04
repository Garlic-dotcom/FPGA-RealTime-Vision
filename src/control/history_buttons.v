// Release-triggered gestures. Core-board K3 is CRESET_N, NOT a GPIO.
module history_buttons #(
 parameter DEBOUNCE_CYCLES=1920000,LONG_CYCLES=76800000,MODE_COUNT=7,
 parameter FILTER_PREVIEW_ENABLE=0,COLOR_STYLE_ENABLE=0,
 parameter INITIAL_THRESHOLD=160,STEP=16,MAX_THRESHOLD=800
)(input clk,rst_n,input [1:0] key_n,
 output reg [10:0] threshold,output reg [2:0] mode,
 output reg filter_gaussian,
 output reg color_direction,
 output reg [1:0] history_mode,output reg older,newer);
 wire [1:0] pressed;
 reg [1:0] gesture; reg [$clog2(LONG_CYCLES+1)-1:0] held;
 button_debounce #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) plus(.clk(clk),.rst_n(rst_n),.button_n(key_n[0]),.pressed(pressed[0]));
 button_debounce #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) minus(.clk(clk),.rst_n(rst_n),.button_n(key_n[1]),.pressed(pressed[1]));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin threshold<=INITIAL_THRESHOLD;filter_gaussian<=0;color_direction<=0;mode<=0;history_mode<=0;gesture<=0;held<=0;older<=0;newer<=0;end
   else begin
     older<=0;newer<=0;
     if(pressed!=0) begin
       gesture<=gesture|pressed;
       // Time the final gesture, not the first key in a staggered chord.
       if((gesture|pressed)!=gesture) held<=0;
       else if(held<LONG_CYCLES) held<=held+1'b1;
     end else if(gesture!=0) begin
       gesture<=0;held<=0;
       if(gesture==3) begin
         if(held==LONG_CYCLES) history_mode<=history_mode+1'b1;
         else mode<=(mode==MODE_COUNT-1)?0:mode+1'b1;
       end else if(FILTER_PREVIEW_ENABLE && mode==2 && held==LONG_CYCLES) begin
         // Idempotent filter selection; never moves the frozen history frame.
         filter_gaussian<=gesture==1;
       end else if(COLOR_STYLE_ENABLE && mode==5 && held==LONG_CYCLES) begin
         color_direction<=gesture==2;
       end else if(history_mode!=0 && held<LONG_CYCLES) begin
         newer<=gesture==1;older<=gesture==2;
       end else if(gesture==1) threshold<=(threshold>MAX_THRESHOLD-STEP)?MAX_THRESHOLD:threshold+STEP;
       else threshold<=(threshold<STEP)?11'd0:threshold-STEP;
     end
   end
 end
endmodule
