// Ordered, single-outstanding 32-bit memory requests over a 128-bit AXI4 port.
// Byte addresses must be 4-byte aligned. Each AXI transaction is one full
// 16-byte beat; strobes preserve the other words. Completion of a write is
// fenced by B before the next local request is accepted. All ports share clk.
// fault is sticky until reset; a failed read never produces mem_rvalid.
module word32_axi128_bridge #(parameter EARLY_READ=0)(
 input clk,rst_n,
 input read_early,
 input mem_valid,mem_write,
 input [31:0] mem_address,mem_wdata,
 input [3:0] mem_wstrb,
 output mem_ready,
 output reg mem_rvalid,
 output reg [31:0] mem_rdata,
 output reg fault,
 output [31:0] axi_awaddr,axi_araddr,
 output [7:0] axi_awlen,axi_arlen,
 output [2:0] axi_awsize,axi_arsize,
 output [1:0] axi_awburst,axi_arburst,
 output axi_awvalid,axi_arvalid,
 input axi_awready,axi_arready,
 output [127:0] axi_wdata,
 output [15:0] axi_wstrb,
 output axi_wlast,axi_wvalid,
 input axi_wready,
 input [1:0] axi_bresp,
 input axi_bvalid,
 output axi_bready,
 input [127:0] axi_rdata,
 input [1:0] axi_rresp,
 input axi_rlast,axi_rvalid,
 output axi_rready
);
 localparam IDLE=0,WRITE=1,WRITE_RESP=2,READ_ADDR=3,READ_DATA=4,FAILED=5;
 reg [2:0] state;
 reg [31:0] address,data;
 reg [3:0] strobes;
 reg [1:0] lane;
 reg aw_pending,w_pending;
 assign mem_ready=rst_n && state==IDLE && !fault;
 assign axi_awaddr=address;
 // The owner can veto early AR while capturing a new dirty victim.
 // EARLY_READ=0 preserves the original one-cycle address stage.
 wire read_offer=EARLY_READ && read_early && mem_ready && mem_valid &&
     !mem_write && mem_address[1:0]==0;
 assign axi_araddr=read_offer?{mem_address[31:4],4'd0}:address;
 assign axi_awlen=0;
 assign axi_arlen=0;
 assign axi_awsize=3'd4;
 assign axi_arsize=3'd4;
 assign axi_awburst=2'b01;
 assign axi_arburst=2'b01;
 assign axi_awvalid=rst_n && state==WRITE && aw_pending;
 assign axi_wvalid=rst_n && state==WRITE && w_pending;
 assign axi_wdata={96'd0,data}<<(32*lane);
 assign axi_wstrb={12'd0,strobes}<<(4*lane);
 assign axi_wlast=1'b1;
 assign axi_bready=rst_n && state==WRITE_RESP;
 assign axi_arvalid=read_offer || (rst_n && state==READ_ADDR);
 assign axi_rready=rst_n && state==READ_DATA;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=IDLE;address<=0;data<=0;strobes<=0;lane<=0;
     aw_pending<=0;w_pending<=0;mem_rvalid<=0;mem_rdata<=0;fault<=0;
   end else begin
     mem_rvalid<=0;
     case(state)
       IDLE:if(mem_valid) begin
         if(mem_address[1:0]!=0) begin fault<=1;state<=FAILED;end
         else begin
           address<={mem_address[31:4],4'd0};lane<=mem_address[3:2];
           data<=mem_wdata;strobes<=mem_wstrb;
           aw_pending<=1;w_pending<=1;
           state<=mem_write?WRITE:((read_offer && axi_arready)?READ_DATA:READ_ADDR);
         end
       end
       WRITE:begin
         if(axi_awready) aw_pending<=0;
         if(axi_wready) w_pending<=0;
         if((!aw_pending || axi_awready) && (!w_pending || axi_wready)) state<=WRITE_RESP;
       end
       WRITE_RESP:if(axi_bvalid) begin
         if(axi_bresp!=0) begin fault<=1;state<=FAILED;end
         else state<=IDLE;
       end
       READ_ADDR:if(axi_arready) state<=READ_DATA;
       READ_DATA:if(axi_rvalid) begin
         if(axi_rresp!=0 || !axi_rlast) begin fault<=1;state<=FAILED;end
         else begin mem_rdata<=axi_rdata>>(32*lane);mem_rvalid<=1;state<=IDLE;end
       end
       FAILED:state<=FAILED;
       default:begin fault<=1;state<=FAILED;end
     endcase
   end
 end
endmodule
