// Physical-DDR diagnostic, not camera Canny. Scratch is exclusively owned here:
// [0x08000000,0x08001000), above all 32 historical 4-MiB frame slots.
// Six 37x19 maps, independent output ROM, byte guards and successful-B fences.
module canny_ddr_selftest #(parameter START_DELAY=96000000)(
 input clk,rst_n,enable,
 output reg done,failed,
 output reg [2:0] test_index,
 output reg [31:0] total_cycles,
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
 localparam W=37,H=19,N=W*H,BASE=32'h08000000,LBASE=BASE+16,QBASE=BASE+1024,WORDS=1024,LWORDS=(N+3)/4;
 localparam WAIT_ENABLE=0,PREP=1,INIT=2,INIT_FLUSH=3,INIT_FENCE=4,CLEAR=5,
   START=6,RUN=7,DRAIN=8,FENCE=9,GUARD_REQ=10,GUARD_WAIT=11,FINISH=12;
 reg [3:0] state;
 reg [26:0] delay_count,watchdog;
 reg [9:0] init_index,output_index,guard_index;
 reg [7:0] pace;
 wire hb,hdone,hv,hw,hr,rv,ov,os,oe,bridge_fault,fenced;
 wire [31:0] ha,hd,rd,cycles,accepted,qwrites;
 wire [3:0] hs;
 wire [7:0] od;
 wire output_ready=pace[1:0]!=0;
 function [7:0] label_at;
   input [2:0] t;input integer p;integer x,y;
   begin
     x=p%W;y=p/W;label_at=0;
     if(p>=N) label_at=8'ha5;
     else case(t)
       0:label_at=0;
       1:label_at=1;
       2:label_at=2;
       3:label_at=(p==N-1)?2:1;
       4:begin
         if(x==y) label_at=(x==H-1)?2:1;
         if(y==H-2 && x>=W-5) label_at=1;
       end
       5:begin
         if(y==2 && x>=1 && x<W-1) label_at=(x==W-2)?2:1;
         if(y==H-2 && x>=1 && x<W/2) label_at=1;
       end
     endcase
   end
 endfunction
 function expected_edge;
   input [2:0] t;input integer p;integer x,y;
   begin
     x=p%W;y=p/W;expected_edge=0;
     case(t)
       2,3:expected_edge=1;
       4:expected_edge=(x==y);
       5:expected_edge=(y==2 && x>=1 && x<W-1);
     endcase
   end
 endfunction
 function [31:0] expected_count;
   input [2:0] t;
   begin case(t)
     2,3:expected_count=N;
     4:expected_count=H;
     5:expected_count=W-2;
     default:expected_count=0;
   endcase end
 endfunction
 function [31:0] expected_queue_count;
   input [2:0] t;
   // Strong seeds are visited directly. Only accepted weak pixels enter
   // the queue; these counts follow the independent fixed label maps.
   begin case(t)
     3:expected_queue_count=N-1;
     4:expected_queue_count=H-1;
     5:expected_queue_count=W-3;
     default:expected_queue_count=0;
   endcase end
 endfunction
 // All six fixed maps use exactly sixteen full 32-bit label words.
 // Encode at elaboration time; decode combinationally after the synchronous
 // four-bit ROM read. No extra cycle and no change to the A5 guard bytes.
 function [3:0] label_code;
   input [31:0] word;
   begin case(word)
       32'h00000000:label_code=4'h0;
       32'h00000001:label_code=4'h1;
       32'h00000002:label_code=4'h2;
       32'h00000101:label_code=4'h3;
       32'h00000201:label_code=4'h4;
       32'h00010000:label_code=4'h5;
       32'h00010101:label_code=4'h6;
       32'h01000000:label_code=4'h7;
       32'h01010000:label_code=4'h8;
       32'h01010100:label_code=4'h9;
       32'h01010101:label_code=4'ha;
       32'h02020202:label_code=4'hb;
       32'ha5000000:label_code=4'hc;
       32'ha5010101:label_code=4'hd;
       32'ha5020101:label_code=4'he;
       32'ha5020202:label_code=4'hf;
       default:label_code=4'bxxxx;
     endcase end
 endfunction
 function [31:0] label_word;
   input [3:0] code;
   begin case(code)
       4'h0:label_word=32'h00000000;
       4'h1:label_word=32'h00000001;
       4'h2:label_word=32'h00000002;
       4'h3:label_word=32'h00000101;
       4'h4:label_word=32'h00000201;
       4'h5:label_word=32'h00010000;
       4'h6:label_word=32'h00010101;
       4'h7:label_word=32'h01000000;
       4'h8:label_word=32'h01010000;
       4'h9:label_word=32'h01010100;
       4'ha:label_word=32'h01010101;
       4'hb:label_word=32'h02020202;
       4'hc:label_word=32'ha5000000;
       4'hd:label_word=32'ha5010101;
       4'he:label_word=32'ha5020101;
       4'hf:label_word=32'ha5020202;
       default:label_word=32'hxxxxxxxx;
     endcase end
 endfunction
 reg [3:0] label_code_rom[0:6*LWORDS-1];
 reg edge_rom[0:6*N-1];
 integer t,k;
 initial begin
   for(t=0;t<6;t=t+1) begin
     for(k=0;k<LWORDS;k=k+1)
       label_code_rom[t*LWORDS+k]=label_code({label_at(t,k*4+3),label_at(t,k*4+2),label_at(t,k*4+1),label_at(t,k*4)});
     for(k=0;k<N;k=k+1) edge_rom[t*N+k]=expected_edge(t,k);
   end
 end
 reg [3:0] label_code_q;
 reg label_in_range_q;
 wire [31:0] init_data=label_in_range_q ? label_word(label_code_q) : 32'ha5a5a5a5;
 reg expected;
 wire [9:0] next_init=(state==INIT && hr && init_index<WORDS-1)?init_index+1'b1:init_index;
 wire [9:0] next_output=(ov && output_ready && output_index<N-1)?output_index+1'b1:output_index;
 always @(posedge clk) begin
   label_code_q <= label_code_rom[test_index*LWORDS+next_init-4];
   label_in_range_q <= next_init>=4 && next_init<4+LWORDS;
   expected <= edge_rom[test_index*N+next_output];
 end
 canny_hysteresis_engine #(.WIDTH(W),.HEIGHT(H)) engine(
 .clk(clk),.rst_n(rst_n),.start(state==START),.label_base(LBASE),.queue_base(QBASE),.busy(hb),.done(hdone),
 .mem_valid(hv),.mem_write(hw),.mem_address(ha),.mem_wdata(hd),.mem_wstrb(hs),.mem_ready(hr&&state==RUN),.mem_rvalid(rv&&state==RUN),.mem_rdata(rd),
 .source_valid(ov),.source_sop(os),.source_eop(oe),.source_data(od),.source_ready(output_ready),.cycle_count(cycles),.accepted_count(accepted));
 // Register selftest AR before the shared mux. This cuts the long
 // neighbor/cache/idle-AR/RREADY path without changing production image jobs.
 canny_queue_axi128_bridge #(.QUEUE_WORDS(N),.EARLY_READ(0)) bridge(
 .clk(clk),.rst_n(rst_n),.cache_clear(state==PREP || state==CLEAR),.queue_base(QBASE),
 .flush(state==INIT_FLUSH || state==DRAIN),.flush_done(fenced),.queue_write_beats(qwrites),
 .mem_valid((state==RUN&&hv) || state==INIT || state==GUARD_REQ),
 .mem_write(state==INIT || (state==RUN&&hw)),
 .mem_address(state==INIT ? BASE+{20'd0,init_index,2'b0} : state==GUARD_REQ ? BASE+{20'd0,guard_index,2'b0} : ha),
 .mem_wdata(state==INIT?init_data:hd),.mem_wstrb(state==INIT?4'hf:hs),
 .mem_ready(hr),.mem_rvalid(rv),.mem_rdata(rd),.fault(bridge_fault),
 .axi_awaddr(axi_awaddr),.axi_araddr(axi_araddr),.axi_awlen(axi_awlen),.axi_arlen(axi_arlen),.axi_awsize(axi_awsize),.axi_arsize(axi_arsize),
 .axi_awburst(axi_awburst),.axi_arburst(axi_arburst),.axi_awvalid(axi_awvalid),.axi_arvalid(axi_arvalid),.axi_awready(axi_awready),.axi_arready(axi_arready),
 .axi_wdata(axi_wdata),.axi_wstrb(axi_wstrb),.axi_wlast(axi_wlast),.axi_wvalid(axi_wvalid),.axi_wready(axi_wready),
 .axi_bresp(axi_bresp),.axi_bvalid(axi_bvalid),.axi_bready(axi_bready),.axi_rdata(axi_rdata),.axi_rresp(axi_rresp),.axi_rlast(axi_rlast),.axi_rvalid(axi_rvalid),.axi_rready(axi_rready));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=WAIT_ENABLE;done<=0;failed<=0;test_index<=0;total_cycles<=0;
     init_index<=0;output_index<=0;guard_index<=0;delay_count<=0;watchdog<=0;pace<=0;
   end else begin
     pace<=pace+1'b1;
     if(state!=WAIT_ENABLE && state!=FINISH) watchdog<=watchdog+1'b1;
     case(state)
       WAIT_ENABLE:begin
         if(!enable) delay_count<=0;
         else if(delay_count==START_DELAY-1) state<=PREP;
         else delay_count<=delay_count+1'b1;
       end
       PREP:begin init_index<=0;output_index<=0;watchdog<=0;state<=INIT;end
       INIT:if(hr) begin
         if(init_index==WORDS-1) state<=INIT_FLUSH;
         else init_index<=init_index+1'b1;
       end
       INIT_FLUSH:state<=INIT_FENCE;
       INIT_FENCE:if(fenced) state<=CLEAR;
       CLEAR:state<=START;
       START:state<=RUN;
       RUN:begin
         if(hv && hr && (ha[1:0]!=0 || ha<BASE || ha>=BASE+WORDS*4 ||
             (hw && !(ha>=LBASE && ha<LBASE+LWORDS*4 || ha>=QBASE && ha<QBASE+N*4)))) failed<=1;
         if(ov && output_ready) begin
           if(od!={8{expected}} || os!=(output_index==0) || oe!=(output_index==N-1) || output_index>=N) failed<=1;
           output_index<=output_index+1'b1;
         end
         if(hdone) begin
           if(output_index!=N || accepted!=expected_count(test_index)) failed<=1;
           total_cycles<=total_cycles+cycles;state<=DRAIN;
         end
       end
       DRAIN:state<=FENCE;
       FENCE:if(fenced) begin
         if(qwrites!=(expected_queue_count(test_index)+3)/4) failed<=1;
         guard_index<=0;state<=GUARD_REQ;
       end
       GUARD_REQ:if(hr) state<=GUARD_WAIT;
       GUARD_WAIT:if(rv) begin
         if(guard_index==4+LWORDS-1) begin if(rd[31:24]!=8'ha5) failed<=1;end
         else if(rd!=32'ha5a5a5a5) failed<=1;
         if(guard_index==3) guard_index<=4+LWORDS-1;
         else if(guard_index==255) guard_index<=256+expected_queue_count(test_index);
         else guard_index<=guard_index+1'b1;
         if(guard_index==WORDS-1) begin
           if(test_index==5) begin done<=1;state<=FINISH;end
           else begin test_index<=test_index+1'b1;state<=PREP;end
         end else state<=GUARD_REQ;
       end
       FINISH:state<=FINISH;
       default:begin failed<=1;done<=1;state<=FINISH;end
     endcase
     if(bridge_fault || (&watchdog && state!=FINISH)) begin failed<=1;done<=1;state<=FINISH;end
   end
 end
endmodule
