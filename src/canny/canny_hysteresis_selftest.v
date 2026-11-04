// Board diagnostic for the exact hysteresis engine, NOT the live Canny path.
// Local synchronous byte-lane RAM substitutes for DDR; no external bus access.
// Six independently specified label maps exercise full queue, late seeds,
// diagonal connectivity, a disconnected weak island and output backpressure.
module canny_hysteresis_selftest(
 input clk,rst_n,
 output reg done,failed,
 output reg [2:0] test_index,
 output reg [31:0] total_cycles
);
 localparam W=37,H=19,N=W*H,QBASE=1024,WORDS=1024;
 localparam INIT=0,START=1,RUN=2,GUARD_REQ=3,GUARD_WAIT=4,CHECK=5,FINISH=6;
 reg [2:0] state;
 reg [9:0] init_index,guard_index;
 reg [9:0] output_index;
 reg [23:0] watchdog;
 reg [7:0] pace;
 wire busy,engine_done,mv,mw,ov,os,oe;
 wire [31:0] ma,md,cycles,accepted;
 wire [3:0] ms;
 wire [7:0] od;
 wire mem_ready=state==RUN && pace[2:0]!=0;
 wire output_ready=pace[1:0]!=0;
 reg mem_rvalid;
 reg [7:0] bank0[0:WORDS-1],bank1[0:WORDS-1],bank2[0:WORDS-1],bank3[0:WORDS-1];
 reg [7:0] rd0,rd1,rd2,rd3;
 wire [31:0] rd={rd3,rd2,rd1,rd0};

 function [7:0] label_at;
   input [2:0] t;
   input integer p;
   integer x,y;
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
   input [2:0] t;
   input integer p;
   integer x,y;
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

 // Keep test vectors/expected values in ROMs. Index arithmetic and division
 // above are elaboration-time only, never a pixel-rate divider in hardware.
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
 reg [3:0] label_code_rom[0:6*((N+3)/4)-1];
 reg edge_rom[0:6*N-1];
 localparam LABEL_WORDS=(N+3)/4;
 integer t,k;
 initial begin
   for(t=0;t<6;t=t+1) begin
     for(k=0;k<LABEL_WORDS;k=k+1)
       label_code_rom[t*LABEL_WORDS+k]=label_code({label_at(t,k*4+3),label_at(t,k*4+2),label_at(t,k*4+1),label_at(t,k*4)});
     for(k=0;k<N;k=k+1) edge_rom[t*N+k]=expected_edge(t,k);
   end
 end
 reg [3:0] label_code_q;
 reg label_in_range_q;
 wire [31:0] init_data=label_in_range_q ? label_word(label_code_q) : 32'ha5a5a5a5;
 reg expected;
 wire [9:0] ram_address=(state==INIT)?init_index:
                            (state==GUARD_REQ)?guard_index:ma[11:2];
 // Prefetch initialization data and output golden bits one cycle ahead.
 wire [9:0] next_init=(state==INIT && init_index<WORDS-1)?init_index+1'b1:10'd0;
 wire [9:0] next_output=(ov && output_ready && output_index<N-1)?output_index+1'b1:output_index;
 always @(posedge clk) begin
   label_code_q <= label_code_rom[test_index*LABEL_WORDS+next_init];
   label_in_range_q <= next_init<LABEL_WORDS;
   expected <= edge_rom[test_index*N+next_output];
   if(state==INIT) begin
     bank0[ram_address]<=init_data[7:0];bank1[ram_address]<=init_data[15:8];
     bank2[ram_address]<=init_data[23:16];bank3[ram_address]<=init_data[31:24];
   end else if(mv && mem_ready && mw) begin
     if(ms[0]) bank0[ram_address]<=md[7:0];
     if(ms[1]) bank1[ram_address]<=md[15:8];
     if(ms[2]) bank2[ram_address]<=md[23:16];
     if(ms[3]) bank3[ram_address]<=md[31:24];
   end
   rd0<=bank0[ram_address];rd1<=bank1[ram_address];
   rd2<=bank2[ram_address];rd3<=bank3[ram_address];
 end
 canny_hysteresis_engine #(.WIDTH(W),.HEIGHT(H)) engine(
 .clk(clk),.rst_n(rst_n),.start(state==START),.label_base(32'd0),.queue_base(QBASE),
 .busy(busy),.done(engine_done),.mem_valid(mv),.mem_write(mw),.mem_address(ma),.mem_wdata(md),.mem_wstrb(ms),
 .mem_ready(mem_ready),.mem_rvalid(mem_rvalid),.mem_rdata(rd),
 .source_valid(ov),.source_sop(os),.source_eop(oe),.source_data(od),.source_ready(output_ready),
 .cycle_count(cycles),.accepted_count(accepted));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=CHECK;done<=0;failed<=0;test_index<=0;total_cycles<=0;
     init_index<=0;output_index<=0;guard_index<=LABEL_WORDS-1;
     watchdog<=0;pace<=0;mem_rvalid<=0;
   end else begin
     pace<=pace+1'b1;
     mem_rvalid<=mv && mem_ready && !mw;
     case(state)
       CHECK:begin state<=INIT;init_index<=0;output_index<=0;watchdog<=0;end
       INIT:if(init_index==WORDS-1) state<=START;else init_index<=init_index+1'b1;
       START:state<=RUN;
       RUN:begin
         watchdog<=watchdog+1'b1;
         if(mv && mem_ready && (ma[1:0]!=0 || ma>=WORDS*4 ||
              (mw && !(ma<LABEL_WORDS*4 || (ma>=QBASE && ma<QBASE+N*4))))) failed<=1;
         if(ov && output_ready) begin
           if(od!={8{expected}} || os!=(output_index==0) || oe!=(output_index==N-1) || output_index>=N) failed<=1;
           output_index<=output_index+1'b1;
         end
         if(engine_done) begin
           if(output_index!=N || accepted!=expected_count(test_index)) failed<=1;
           total_cycles<=total_cycles+cycles;
           guard_index<=LABEL_WORDS-1;state<=GUARD_REQ;
         end else if(&watchdog) begin failed<=1;done<=1;state<=FINISH;end
       end
       GUARD_REQ:state<=GUARD_WAIT;
       GUARD_WAIT:begin
         if(guard_index==LABEL_WORDS-1) begin
           if(rd[31:24]!=8'ha5) failed<=1;
         end else if(rd!=32'ha5a5a5a5) failed<=1;
         if(guard_index==QBASE/4-1) guard_index<=QBASE/4+N;
         else guard_index<=guard_index+1'b1;
         if(guard_index==QBASE/4+N) begin
           if(test_index==5) begin done<=1;state<=FINISH;end
           else begin test_index<=test_index+1'b1;state<=CHECK;end
         end else state<=GUARD_REQ;
       end
       FINISH:state<=FINISH;
       default:begin failed<=1;done<=1;state<=FINISH;end
     endcase
   end
 end
endmodule
