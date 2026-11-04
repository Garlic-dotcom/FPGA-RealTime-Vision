// Same-clock ready/valid FIFO with synchronous BRAM read and an output register.
// DEPTH must be a power of two. Capacity is DEPTH RAM entries + one output word.
module stream_fifo #(parameter DATA_WIDTH=18,DEPTH=4096)(
 input clk,rst_n,
 input sink_valid,
 input [DATA_WIDTH-1:0] sink_data,
 output sink_ready,
 output reg source_valid,
 output reg [DATA_WIDTH-1:0] source_data,
 input source_ready
);
 localparam AW=$clog2(DEPTH);
 reg [DATA_WIDTH-1:0] memory[0:DEPTH-1];
 reg [AW-1:0] wp,rp;
 reg [AW:0] count;
 wire push=sink_valid && sink_ready;
 wire fetch=(count!=0) && (!source_valid || source_ready);
 assign sink_ready=(count<DEPTH);
 always @(posedge clk) begin
   if(push) memory[wp]<=sink_data;
   if(fetch) source_data<=memory[rp];
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin wp<=0;rp<=0;count<=0;source_valid<=0;end
   else begin
     if(push) wp<=wp+1'b1;
     if(fetch) rp<=rp+1'b1;
     case({push,fetch}) 2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;endcase
     if(fetch) source_valid<=1;
     else if(source_ready) source_valid<=0;
   end
 end
endmodule
