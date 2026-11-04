// Canny label DMA: one byte per pixel, first pixel in LOW byte.
// Only labels 0/1/2 are produced. 16 pixels per AXI beat, partial final WSTRB.
// Ready/valid input; complete-frame done waits for every successful B response.
// Two ordered burst banks overlap label collection with AXI W and B.
// The next burst may collect while the previous burst awaits successful B.
// All ports use clk; reset bus after sticky fault.
module axi_label_writer #(parameter PIXELS=1280*720,MAX_BURST=128)(
 input clk,rst_n,start,
 input [31:0] frame_base,
 output reg busy,done,fault,
 input sink_valid,sink_sop,sink_eop,
 input [1:0] sink_data,
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
 localparam BEATS=(PIXELS+15)/16;
 localparam S_IDLE=0,S_FETCH=1,S_SEND=2,S_RESPONSE=3,S_FAILED=4;
 reg [2:0] send_state;
 reg [3:0] lane;
 reg [31:0] collect_address,beats_left,pixel_index;
 reg [8:0] collected,send_index;
 reg [127:0] pack_data;
 reg collect_bank,send_bank,collecting_done,aw_pending;
 reg [1:0] bank_full;
 reg [31:0] bank_address[0:1];
 reg [8:0] bank_beats[0:1];
 reg [15:0] bank_strobe[0:1];
 reg bank_final[0:1];
 reg [127:0] bank0_mem[0:MAX_BURST-1];
 reg [127:0] bank1_mem[0:MAX_BURST-1];
 reg [127:0] bank0_dout,bank1_dout;
 wire accept=sink_valid && sink_ready;
 wire last_pixel=(pixel_index==PIXELS-1);
 wire finish_word=(lane==15 || last_pixel);
 wire [8:0] page_beats=9'd256-{1'b0,collect_address[11:4]};
 wire [31:0] limited_beats=(beats_left>MAX_BURST)?MAX_BURST:beats_left;
 wire [8:0] target_beats=(limited_beats>page_beats)?page_beats:limited_beats[8:0];
 wire packet_ok=(sink_sop==(pixel_index==0)) && (sink_eop==last_pixel);
 wire packet_error_now=accept && !packet_ok;
 wire [127:0] complete_word=pack_data | ({126'd0,sink_data}<<(8*lane));
 wire write_word=accept && packet_ok && finish_word;
 wire read_word=send_state==S_FETCH ||
                (send_state==S_SEND && axi_wready && !axi_wlast);
 wire [8:0] read_index=(send_state==S_FETCH)?send_index:send_index+1'b1;
 assign sink_ready=rst_n && busy && !fault && !collecting_done && !bank_full[collect_bank];
 assign axi_awaddr=bank_address[send_bank];
 assign axi_awlen=bank_beats[send_bank]-1'b1;
 assign axi_awsize=3'd4;
 assign axi_awburst=2'b01;
 assign axi_awvalid=rst_n && aw_pending;
 assign axi_wvalid=rst_n && send_state==S_SEND;
 assign axi_wlast=(send_index==bank_beats[send_bank]-1'b1);
 assign axi_wstrb=(bank_final[send_bank] && axi_wlast)?bank_strobe[send_bank]:16'hffff;
 assign axi_bready=rst_n && send_state==S_RESPONSE && !aw_pending;
 assign axi_wdata=send_bank ? bank1_dout : bank0_dout;
 // Separate synchronous RAMs. The selected read register does not advance
 // during WREADY stalls, so WDATA/WSTRB/WLAST remain stable.
 always @(posedge clk) begin
   if(write_word && !collect_bank) bank0_mem[collected]<=complete_word;
   if(read_word && !send_bank) bank0_dout<=bank0_mem[read_index];
 end
 always @(posedge clk) begin
   if(write_word && collect_bank) bank1_mem[collected]<=complete_word;
   if(read_word && send_bank) bank1_dout<=bank1_mem[read_index];
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     send_state<=S_IDLE;busy<=0;done<=0;fault<=0;lane<=0;
     collect_address<=0;beats_left<=0;pixel_index<=0;collected<=0;send_index<=0;
     pack_data<=0;collect_bank<=0;send_bank<=0;collecting_done<=0;aw_pending<=0;
     bank_full<=0;
     bank_address[0]<=0;bank_address[1]<=0;
     bank_beats[0]<=0;bank_beats[1]<=0;
     bank_strobe[0]<=16'hffff;bank_strobe[1]<=16'hffff;
     bank_final[0]<=0;bank_final[1]<=0;
   end else begin
     done<=0;
     if(axi_awvalid && axi_awready) aw_pending<=0;

     if(!busy && !fault && start && send_state==S_IDLE) begin
       if(frame_base[3:0]!=0) begin fault<=1;send_state<=S_FAILED;end
       else begin
         busy<=1;collect_address<=frame_base;beats_left<=BEATS;pixel_index<=0;
         lane<=0;collected<=0;send_index<=0;pack_data<=0;
         collect_bank<=0;send_bank<=0;collecting_done<=0;bank_full<=0;
         aw_pending<=0;
       end
     end else begin
       if(packet_error_now) begin fault<=1;busy<=0;end
       if(accept && packet_ok) begin
         pixel_index<=pixel_index+1'b1;
         pack_data<=complete_word;lane<=lane+1'b1;
         if(finish_word) begin
           lane<=0;pack_data<=0;collected<=collected+1'b1;
           if(collected==target_beats-1'b1) begin
             bank_full[collect_bank]<=1;
             bank_address[collect_bank]<=collect_address;
             bank_beats[collect_bank]<=target_beats;
             bank_strobe[collect_bank]<=last_pixel?(16'hffff>>(15-lane)):16'hffff;
             bank_final[collect_bank]<=last_pixel;
             collect_address<=collect_address+{19'd0,target_beats,4'd0};
             beats_left<=beats_left-target_beats;
             collected<=0;collect_bank<=!collect_bank;
             if(last_pixel) collecting_done<=1;
           end
         end
       end

       case(send_state)
         S_IDLE:begin
           if(fault || packet_error_now) send_state<=S_FAILED;
           else if(bank_full[send_bank]) begin
             send_index<=0;aw_pending<=1;send_state<=S_FETCH;
           end
         end
         S_FETCH:send_state<=S_SEND;
         S_SEND:if(axi_wready) begin
           if(axi_wlast) send_state<=S_RESPONSE;
           else send_index<=send_index+1'b1;
         end
         S_RESPONSE:if(axi_bvalid && axi_bready) begin
           if(axi_bresp!=0) begin
             fault<=1;busy<=0;send_state<=S_FAILED;
           end else begin
             bank_full[send_bank]<=0;
             if(fault || packet_error_now) begin
               busy<=0;send_state<=S_FAILED;
             end else if(bank_final[send_bank]) begin
               busy<=0;done<=1;send_state<=S_IDLE;
             end else begin
               send_bank<=!send_bank;send_state<=S_IDLE;
             end
           end
         end
         S_FAILED:busy<=0;
         default:begin fault<=1;busy<=0;send_state<=S_FAILED;end
       endcase
     end
   end
 end
endmodule
