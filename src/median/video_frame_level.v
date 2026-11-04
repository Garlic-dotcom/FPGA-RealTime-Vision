// Convert processed SOP/EOP to the sustained level expected by axi4_ctrl.
// The writer detects its falling edge. Delay that edge by eight PCLK cycles
// to let the last FIFO write pointer propagate through the CDC synchronizers.
module video_frame_level(
 input clk,rst_n,pixel_valid,pixel_sop,pixel_eop,
 output reg frame_level
);
 reg [3:0] eof_wait;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin frame_level<=0;eof_wait<=0;end
   else begin
     if(eof_wait!=0) begin
       eof_wait<=eof_wait-1'b1;
       if(eof_wait==1) frame_level<=0;
     end
     if(pixel_valid && pixel_sop) begin frame_level<=1;eof_wait<=0;end
     if(pixel_valid && pixel_eop) eof_wait<=8;
   end
 end
endmodule
