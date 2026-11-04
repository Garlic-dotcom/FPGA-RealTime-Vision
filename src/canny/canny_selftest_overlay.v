// Status-only overlay for the unit diagnostic. Video timing passes unchanged.
module canny_selftest_overlay #(parameter HIDE_SUCCESS=0)(
 input sys_clk,sys_rst_n,pixel_clk,pixel_rst_n,
 input ddr_done,ddr_failed,
 input video_de,video_vsync,
 input [23:0] video_rgb,
 output [23:0] output_rgb
);
 wire done,failed;
 wire [2:0] test_index;
 wire [31:0] total_cycles;
 wire [15:0] status;
 wire [15:0] ddr_status;
 history_status_cdc ddr_mailbox(.source_clk(sys_clk),.source_rst_n(sys_rst_n),
   .data_in({14'd0,ddr_failed,ddr_done}),.dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(ddr_status));
 canny_hysteresis_selftest selftest(.clk(sys_clk),.rst_n(sys_rst_n),
   .done(done),.failed(failed),.test_index(test_index),.total_cycles(total_cycles));
 history_status_cdc mailbox(.source_clk(sys_clk),.source_rst_n(sys_rst_n),
   .data_in({11'd0,test_index,failed,done}),.dest_clk(pixel_clk),.dest_rst_n(pixel_rst_n),.data_out(status));
 reg [10:0] x,y;
 reg de_d;
 always @(posedge pixel_clk or negedge pixel_rst_n) begin
   if(!pixel_rst_n) begin x<=0;y<=0;de_d<=0;end
   else begin
     de_d<=video_de;
     if(!video_vsync) begin x<=0;y<=0;end
     else begin
       if(video_de) x<=x+1'b1;else x<=0;
       if(de_d && !video_de) y<=y+1'b1;
     end
   end
 end
 wire box=video_de && x>=1216 && x<1248 && y>=8 && y<24;
 wire [23:0] color=status[1]?24'hff2020:status[0]?24'h20ff40:24'hffff20;
 wire ddr_box=video_de && x>=1176 && x<1208 && y>=8 && y<24;
 wire [23:0] ddr_color=ddr_status[1]?24'hff2020:ddr_status[0]?24'h20ff40:24'hffff20;
 assign output_rgb=(box && (!HIDE_SUCCESS || !status[0] || status[1]))?color:
                   (ddr_box && (!HIDE_SUCCESS || !ddr_status[0] || ddr_status[1]))?ddr_color:video_rgb;
endmodule
