// Align original pixel context with a later, ordered full-frame algorithm.
// Both streams must have one SOP and exactly WIDTH*HEIGHT items per good frame.
// Caller must drain an old frame before the next input SOP and keep the maximum
// in-flight pixel count below DEPTH (4096 > 2*1280 + arithmetic pipelines).
module pixel_context_fifo #(parameter DATA_WIDTH=24,DEPTH=4096)(
 input clk,rst_n,write_valid,write_sop,
 input [DATA_WIDTH-1:0] write_data,
 input read_valid,read_sop,read_eop,
 output reg source_valid,source_sop,source_eop,
 output reg [DATA_WIDTH-1:0] source_data
);
 localparam AW=$clog2(DEPTH);
 
 reg [AW-1:0] write_pointer,read_pointer;
 wire [AW-1:0] wa=write_sop ? {AW{1'b0}} : write_pointer;
 wire [AW-1:0] ra=read_sop ? {AW{1'b0}} : read_pointer;
 generate if(DEPTH==6144)begin: banked
   // Three exact 2048-word banks avoid rounding a 6144-word RAM to 8192.
   reg [DATA_WIDTH-1:0] bank0[0:2047],bank1[0:2047],bank2[0:2047];
   reg [DATA_WIDTH-1:0] q0,q1,q2;
   reg [1:0] read_bank;
   always @(posedge clk)begin
     if(rst_n && write_valid)case(wa[12:11])
       0:bank0[wa[10:0]]<=write_data;
       1:bank1[wa[10:0]]<=write_data;
       2:bank2[wa[10:0]]<=write_data;
     endcase
     if(rst_n && read_valid)begin
       q0<=bank0[ra[10:0]];q1<=bank1[ra[10:0]];q2<=bank2[ra[10:0]];read_bank<=ra[12:11];
     end
   end
   always @*case(read_bank)0:source_data=q0;1:source_data=q1;default:source_data=q2;endcase
 end else begin: original
   reg [DATA_WIDTH-1:0] storage[0:DEPTH-1];
   always @(posedge clk)begin
     if(rst_n && write_valid)storage[wa]<=write_data;
     if(rst_n && read_valid)source_data<=storage[ra];
   end
 end endgenerate
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     write_pointer<=0;read_pointer<=0;
     source_valid<=0;source_sop<=0;source_eop<=0;
   end else begin
     if(write_valid) write_pointer<=(wa==DEPTH-1)?0:wa+1'b1;
     if(read_valid) read_pointer<=(ra==DEPTH-1)?0:ra+1'b1;
     source_valid<=read_valid;
     source_sop<=read_valid && read_sop;
     source_eop<=read_valid && read_eop;
   end
 end
endmodule
