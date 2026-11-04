// Canny scratch bridge: eight coherent read/write-back lines and a queue
// write combiner. Label hits accumulate byte strobes until eviction/flush.
// Local ordering is coherent; external visibility is guaranteed by flush_done.
// The caller exclusively owns both planes from cache_clear through flush_done.
// Pulse cache_clear while idle to latch queue_base, then flush at end of frame.
// flush_done fences ALL successful AXI B responses, including a partial tail.
// Fault is sticky; reset the bridge AND its AXI peer to recover/cancel traffic.
module canny_queue_axi128_bridge #(parameter QUEUE_WORDS=1280*720, CACHE_LINES=8, EARLY_READ=1)(
 input clk,rst_n,cache_clear,flush,
 input [31:0] queue_base,
 output reg flush_done,
 output reg [31:0] queue_write_beats,
 input mem_valid,mem_write,
 input [31:0] mem_address,mem_wdata,
 input [3:0] mem_wstrb,
 output mem_ready,mem_rvalid,
 output [31:0] mem_rdata,
 // Complete coherent 16-byte line for aligned non-queue label reads.
 // It updates with the corresponding mem_rvalid pulse and holds otherwise.
 output [127:0] mem_line_rdata,
 output fault,
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
 localparam [32:0] QUEUE_BYTES=33'd4*QUEUE_WORDS;
 localparam CIW=$clog2(CACHE_LINES);
 localparam Q_IDLE=0,Q_SEND=1,Q_RESP=2;
 reg [1:0] state;
 reg configured,local_fault,flush_pending;
 reg [31:0] qbase;
 reg [32:0] qend;
 reg [127:0] pending_data;
 reg [27:0] pending_tag;
 reg [3:0] pending_valid;
 reg aw_pending,w_pending;
 reg [127:0] data[0:CACHE_LINES-1];
 reg [15:0] dirty[0:CACHE_LINES-1];
 reg [127:0] send_data;
 reg [27:0] send_tag;
 reg [15:0] send_strobes;
 reg send_queue;
 reg [CIW-1:0] send_index,dirty_index;
 reg any_dirty;
 reg [27:0] tag[0:CACHE_LINES-1];
 reg [CACHE_LINES-1:0] valid;
 reg [CIW-1:0] replace_index;
 wire [CIW-1:0] hit_index;
 wire hit;
 reg hit_valid;
 // Logical hit_data keeps its last word outside the response edge.
 reg [27:0] miss_tag;
 reg [127:0] coherent_rdata;
 integer i,j,k,d;
 integer cache_data_line,cache_data_word;

 // I candidate: CACHE32 only, sixteen 8-bit true-dual-port native banks.
 // A serves a selected hit/victim read or byte writes; B supplies full fills.
 // Native QA is visible on the original response edge, then captured into
 // the existing public-line/send holding registers before another A read.
 // Unwritten native contents initialize to zero; old invalid FF data were X.
 // No claim of private uninitialized-array equality is made.
 reg ram_hit_forward,ram_hit_live;
 reg [1:0] ram_hit_lane;
 reg [31:0] ram_pending_word;
 reg [31:0] hit_data_hold;
 reg ram_line_live,ram_victim_live;
 reg [127:0] mem_line_hold;
 reg ram_a_read,ram_fill;
 reg [15:0] ram_a_byte_we;
 wire [127:0] ram_a_q;
 wire [31:0] ram_hit_word=ram_hit_forward?ram_pending_word:(ram_a_q>>(ram_hit_lane*32));
 wire [31:0] hit_data=(CACHE_LINES==32 && ram_hit_live)?ram_hit_word:hit_data_hold;
 wire [9:0] ram_a_address=ram_a_read?read_index:hit_index;
 wire [9:0] ram_b_address=replace_index;
 integer ram_line,ram_word,ram_byte;
 // Metadata changes only in the unmodified controller below. The native
 // write/read enables are known 0/1 from the same procedural-if conditions;
 // unknown index/lane/strobe comparisons therefore retain their old no-op.
 always @* begin
   ram_a_read=0;ram_fill=0;ram_a_byte_we=0;
   if(!fault && !cache_clear) begin
     if(mem_valid && mem_ready) begin
       if(bad_request) begin end
       else begin
         if(use_hit) ram_a_read=1;
         if(mem_write && hit)
           for(ram_byte=0;ram_byte<4;ram_byte=ram_byte+1)
             if(mem_wstrb[ram_byte])
               for(ram_line=0;ram_line<CACHE_LINES;ram_line=ram_line+1)
                 for(ram_word=0;ram_word<4;ram_word=ram_word+1)
                   if(hit_index==ram_line && mem_address[3:2]==ram_word)
                     ram_a_byte_we[ram_word*4+ram_byte]=1;
       end
     end
     if(axi_rvalid && axi_rready && axi_rresp==0 && axi_rlast)
       for(ram_line=0;ram_line<CACHE_LINES;ram_line=ram_line+1)
         if(replace_index==ram_line) ram_fill=1;
     if(state==Q_IDLE && drain && base_ready) begin
       if(pending_valid!=0 && !evict_needed) begin end
       else if(evict_needed || ((flush || flush_pending) && any_dirty)) ram_a_read=1;
     end
   end
   // The old full-fill assignment was the last NBA writer. Suppress the
   // same-address A write rather than relying on undefined cross-port writes.
   // Different-address A/B writes remain independent.
   if(ram_fill && hit_index==replace_index) ram_a_byte_we=0;
 end
 genvar ram_bank;
 generate if(CACHE_LINES==32) begin: native_cache32
   for(ram_bank=0;ram_bank<16;ram_bank=ram_bank+1) begin: byte_bank
     EFX_DPRAM10 #(.READ_WIDTH_A(8),.WRITE_WIDTH_A(8),
       .READ_WIDTH_B(8),.WRITE_WIDTH_B(8),.OUTPUT_REG_A(0),.OUTPUT_REG_B(0),
       .WRITE_MODE_A("NO_CHANGE"),.WRITE_MODE_B("NO_CHANGE")) bytes_ram(
       .CLKA(clk),.WEA(ram_a_byte_we[ram_bank]),
       .CLKEA(ram_a_read || ram_a_byte_we[ram_bank]),.RSTA(1'b0),.ADDRENA(1'b1),
       .WDATAA(mem_wdata[(ram_bank%4)*8+:8]),.ADDRA(ram_a_address),
       .RDATAA(ram_a_q[ram_bank*8+:8]),
       .CLKB(clk),.WEB(1'b1),.CLKEB(ram_fill),.RSTB(1'b0),.ADDRENB(1'b1),
       .WDATAB(coherent_rdata[ram_bank*8+:8]),.ADDRB(ram_b_address),.RDATAB());
   end
 end else begin: unchanged_ff_cache
   assign ram_a_q=128'b0;
 end endgenerate
 assign mem_line_rdata=(CACHE_LINES==32 && ram_line_live)?ram_a_q:mem_line_hold;
 wire base_ready,base_rvalid,base_fault;
 wire [31:0] base_rdata,bawaddr;
 wire bawvalid,bwvalid,bbready;
 wire [127:0] bwdata;
 wire [15:0] bwstrb;
 wire qactive=state!=Q_IDLE;
 wire is_queue=configured && mem_address>=qbase && {1'b0,mem_address}<qend;
 wire qwrite=is_queue && mem_write;
 wire bad_request=mem_address[1:0]!=0 || (qwrite && mem_wstrb!=4'hf);
 wire forward_hit=is_queue && pending_valid[mem_address[3:2]] && pending_tag==mem_address[31:4];
 wire use_hit=!mem_write && !bad_request && (forward_hit || hit);
 wire different_block=qwrite && pending_valid!=0 && pending_tag!=mem_address[31:4];
 wire label_hit_write=mem_write && !is_queue && hit && !bad_request;
 wire evict_needed=mem_valid && !mem_write && !use_hit && !bad_request && dirty[replace_index]!=0;
 wire drain=evict_needed || flush || flush_pending || pending_valid==4'hf || (mem_valid && different_block);
 // Capture the dirty victim into send_* and accept the unrelated read on the
 // same edge. AXI read/write channels may run concurrently. All other local
 // requests stay blocked until the victim's B, so a fill cannot be dirtied
 // before that B clears send_index. Same-address accesses still use coherent
 // forwarding; a flush never admits this overlap.
 wire overlap_evict_read=state==Q_IDLE && evict_needed && !flush && !flush_pending &&
     valid[replace_index] && tag[replace_index]!=mem_address[31:4];
 // A cached read needs no AXI transaction and cannot dirty an in-flight
 // victim. Let it pass during Q_SEND/Q_RESP after the base read completes;
 // Queue misses, unsafe writes, and every request during final flush retain
 // the B fence; clean label misses have the separate rule below.
 wire cached_read_bypass=qactive && use_hit && !flush && !flush_pending;
 // A label hit on a different cache line can be accumulated while the
 // captured victim (or queue word) waits for B. Never dirty send_index again:
 // its saved strobes/data are already in flight and B will clear that slot.
 wire cached_write_bypass=qactive && label_hit_write &&
     (send_queue || hit_index!=send_index) && !flush && !flush_pending;
 // A B error can coincide with a speculative cached hit or unrelated R.
 // The existing mem_rvalid is suppressed by the resulting sticky fault, so
 // the wide line register must also hold its prior value on that edge.
 wire b_error_now=qactive && state==Q_RESP && axi_bvalid && axi_bresp!=0;
 // A label-plane miss can use the independent AXI read channel while an
 // unrelated write waits for B. The replacement line must be clean and must
 // not be the captured dirty victim: its data/strobes are already in flight.
 // Queue misses cannot use this path because a pending queue write may retire
 // before an older DDR read response, leaving stale data to be filled.
 wire clean_label_miss_bypass=qactive && mem_valid && !mem_write && !is_queue &&
     !use_hit && !bad_request && dirty[replace_index]==0 &&
     (send_queue || replace_index!=send_index) &&
     (send_queue || !valid[replace_index] || tag[replace_index]!=send_tag) &&
     !flush && !flush_pending;
 // A cache hit cannot be accepted while evicting or flushing. Share the
 // wide cache read mux instead of synthesizing a second 128-bit read port.
 wire [CIW-1:0] read_index=evict_needed?replace_index:((flush || flush_pending)?dirty_index:hit_index);
 wire [127:0] read_line=data[read_index];
 assign fault=local_fault || base_fault;
 assign mem_ready=rst_n && configured && !fault && !cache_clear && base_ready &&
     ((!qactive && !drain) || overlap_evict_read || cached_read_bypass ||
      cached_write_bypass || clean_label_miss_bypass);
 assign mem_rvalid=!fault && (hit_valid || base_rvalid);
 assign mem_rdata=hit_valid?hit_data:base_rdata;
 // Parallel known match leaves preserve the original procedural-if rule:
 // an X/Z comparison does not count as a hit. The padded balanced tree
 // selects the greatest matching index, including duplicate tag entries.
 // It changes only combinational structure; no state or request cycle.
 localparam HIT_LEAVES=2**CIW;
 localparam HIT_IW=(CIW>0)?CIW:2;
 wire [HIT_IW:0] hit_tree[1:2*HIT_LEAVES-1];
 genvar hit_leaf,hit_node;
 generate
   for(hit_leaf=0;hit_leaf<HIT_LEAVES;hit_leaf=hit_leaf+1) begin: hit_match_leaf
     if(hit_leaf<CACHE_LINES) begin: real_line
       localparam [HIT_IW-1:0] LEAF_INDEX=hit_leaf;
       reg matched;
       always @* begin
         matched=0;
         if(valid[hit_leaf] && tag[hit_leaf]==mem_address[31:4]) matched=1;
       end
       assign hit_tree[HIT_LEAVES+hit_leaf]=matched?{1'b1,LEAF_INDEX}:{(HIT_IW+1){1'b0}};
     end else begin: padding
       assign hit_tree[HIT_LEAVES+hit_leaf]={(HIT_IW+1){1'b0}};
     end
   end
   for(hit_node=1;hit_node<HIT_LEAVES;hit_node=hit_node+1) begin: hit_priority_node
     assign hit_tree[hit_node]=hit_tree[2*hit_node+1][HIT_IW]?hit_tree[2*hit_node+1]:hit_tree[2*hit_node];
   end
 endgenerate
 assign hit=hit_tree[1][HIT_IW];
 assign hit_index=hit_tree[1][HIT_IW-1:0];
 always @* begin
   any_dirty=0;dirty_index=0;
   for(d=0;d<CACHE_LINES;d=d+1) if(dirty[d]!=0) begin any_dirty=1;dirty_index=d;end
   // A DDR read may include queue words still in the combiner. Merge those
   // words on a fill so a later cache hit never observes stale DDR contents.
   coherent_rdata=axi_rdata;
   for(k=0;k<4;k=k+1)
     if(pending_valid[k] && pending_tag==miss_tag) coherent_rdata[k*32+:32]=pending_data[k*32+:32];
 end
 // A captured dirty victim offers AW on the next edge. Retain the old
 // AR address stage for that overlap so the 64B shim sees the new AW first.
 word32_axi128_bridge #(.EARLY_READ(EARLY_READ)) base(
 .clk(clk),.rst_n(rst_n),.read_early(!overlap_evict_read),.mem_valid(mem_valid&&mem_ready&&!use_hit&&!qwrite&&!label_hit_write&&!bad_request),
 .mem_write(mem_write),.mem_address(mem_address),.mem_wdata(mem_wdata),.mem_wstrb(mem_wstrb),
 .mem_ready(base_ready),.mem_rvalid(base_rvalid),.mem_rdata(base_rdata),.fault(base_fault),
 .axi_awaddr(bawaddr),.axi_araddr(axi_araddr),.axi_awlen(),.axi_arlen(axi_arlen),
 .axi_awsize(),.axi_arsize(axi_arsize),.axi_awburst(),.axi_arburst(axi_arburst),
 .axi_awvalid(bawvalid),.axi_arvalid(axi_arvalid),.axi_awready(axi_awready&&!qactive),.axi_arready(axi_arready),
 .axi_wdata(bwdata),.axi_wstrb(bwstrb),.axi_wlast(),.axi_wvalid(bwvalid),.axi_wready(axi_wready&&!qactive),
 .axi_bresp(axi_bresp),.axi_bvalid(axi_bvalid&&!qactive),.axi_bready(bbready),
 .axi_rdata(coherent_rdata),.axi_rresp(axi_rresp),.axi_rlast(axi_rlast),.axi_rvalid(axi_rvalid),.axi_rready(axi_rready));
 assign axi_awaddr=qactive?{send_tag,4'b0}:bawaddr;
 assign axi_awlen=0;
 assign axi_awsize=4;
 assign axi_awburst=1;
 assign axi_awvalid=qactive?(rst_n && state==Q_SEND && aw_pending):bawvalid;
 assign axi_wvalid=qactive?(rst_n && state==Q_SEND && w_pending):bwvalid;
 assign axi_wdata=qactive?((CACHE_LINES==32 && ram_victim_live && !send_queue)?ram_a_q:send_data):bwdata;
 assign axi_wstrb=qactive?send_strobes:bwstrb;
 assign axi_wlast=1;
 assign axi_bready=qactive?(rst_n && state==Q_RESP):bbready;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=Q_IDLE;configured<=0;local_fault<=0;flush_pending<=0;flush_done<=0;
     qbase<=0;qend<=0;pending_data<=0;pending_tag<=0;pending_valid<=0;
     aw_pending<=0;w_pending<=0;valid<=0;replace_index<=0;
     hit_valid<=0;hit_data_hold<=0;mem_line_hold<=0;miss_tag<=0;queue_write_beats<=0;
     ram_hit_forward<=0;ram_hit_lane<=0;ram_hit_live<=0;ram_pending_word<=0;ram_line_live<=0;ram_victim_live<=0;
     send_data<=0;send_tag<=0;send_strobes<=0;send_queue<=0;send_index<=0;
     for(j=0;j<CACHE_LINES;j=j+1) dirty[j]<=0;
   end else begin
     hit_valid<=0;flush_done<=0;
     if(CACHE_LINES==32) begin
       // Read old QA before its next native edge update. Public held values
       // survive non-read cycles, B errors, and a new cached bypass read.
       mem_line_hold<=mem_line_rdata;
       if(ram_hit_live) hit_data_hold<=hit_data;
       if(ram_victim_live) send_data<=ram_a_q;
       ram_hit_live<=0;ram_line_live<=0;ram_victim_live<=0;
     end
     if(flush) flush_pending<=1;
     if(cache_clear) begin
       if(!base_ready || qactive || pending_valid!=0 || any_dirty || flush_pending || flush ||
          queue_base[3:0]!=0 || QUEUE_WORDS<1 || ({1'b0,queue_base}+QUEUE_BYTES)>33'h100000000)
         local_fault<=1;
       else begin
         configured<=1;qbase<=queue_base;qend<={1'b0,queue_base}+QUEUE_BYTES;
         valid<=0;replace_index<=0;queue_write_beats<=0;
       end
     end
     if(!fault && !cache_clear) begin
       if(mem_valid && mem_ready) begin
         if(bad_request) local_fault<=1;
         else begin
           if(use_hit) begin
             if(CACHE_LINES==32) begin
               // Keep the original ternary's four-state merge. An unknown
               // forward selector is not silently converted to either arm.
               ram_hit_live<=1;ram_hit_forward<=forward_hit;ram_hit_lane<=mem_address[3:2];
               ram_pending_word<=pending_data>>(mem_address[3:2]*32);
             end else
               hit_data_hold<=forward_hit?(pending_data>>(mem_address[3:2]*32)):(read_line>>(mem_address[3:2]*32));
             hit_valid<=1;
             // H requests a 16-byte-aligned label address. The cache line
             // includes accepted dirty byte writes that have not reached DDR.
             if(!is_queue && !b_error_now) begin
               if(CACHE_LINES==32) ram_line_live<=1;
               else mem_line_hold<=read_line;
             end
           end else if(!mem_write) miss_tag<=mem_address[31:4];
           if(qwrite) begin
             pending_tag<=mem_address[31:4];pending_valid[mem_address[3:2]]<=1;
             pending_data[mem_address[3:2]*32+:32]<=mem_wdata;
           end
           if(mem_write && hit)
             for(j=0;j<4;j=j+1) if(mem_wstrb[j]) begin
               if(CACHE_LINES!=32)
                 for(cache_data_line=0;cache_data_line<CACHE_LINES;cache_data_line=cache_data_line+1)
                   for(cache_data_word=0;cache_data_word<4;cache_data_word=cache_data_word+1)
                     if(hit_index==cache_data_line && mem_address[3:2]==cache_data_word)
                       data[cache_data_line][cache_data_word*32+j*8+:8]<=mem_wdata[j*8+:8];
               if(!is_queue) dirty[hit_index][mem_address[3:2]*4+j]<=1;
             end
         end
       end
       if(axi_rvalid && axi_rready && axi_rresp==0 && axi_rlast) begin
         // Base mem_rvalid is registered on this same R handshake.
         if(!b_error_now) begin mem_line_hold<=coherent_rdata;ram_line_live<=0;end
         if(CACHE_LINES!=32)
           for(cache_data_line=0;cache_data_line<CACHE_LINES;cache_data_line=cache_data_line+1)
             if(replace_index==cache_data_line) data[cache_data_line]<=coherent_rdata;
         tag[replace_index]<=miss_tag;valid[replace_index]<=1;
         dirty[replace_index]<=0;
         replace_index<=(replace_index==CACHE_LINES-1)?0:replace_index+1'b1;
       end
       case(state)
         Q_IDLE:if(drain && base_ready) begin
           if(pending_valid!=0 && !evict_needed) begin
             state<=Q_SEND;aw_pending<=1;w_pending<=1;send_queue<=1;
             send_tag<=pending_tag;send_data<=pending_data;
             send_strobes<={{4{pending_valid[3]}},{4{pending_valid[2]}},{4{pending_valid[1]}},{4{pending_valid[0]}}};
           end else if(evict_needed || ((flush || flush_pending) && any_dirty)) begin
             state<=Q_SEND;aw_pending<=1;w_pending<=1;send_queue<=0;
             send_index<=evict_needed?replace_index:dirty_index;
             send_tag<=tag[evict_needed?replace_index:dirty_index];
             if(CACHE_LINES==32) ram_victim_live<=1;
             else send_data<=read_line;
             send_strobes<=dirty[evict_needed?replace_index:dirty_index];
           end else if(flush || flush_pending) begin flush_pending<=0;flush_done<=1;end
         end
         Q_SEND:begin
           if(axi_awready) aw_pending<=0;
           if(axi_wready) w_pending<=0;
           if((!aw_pending || axi_awready) && (!w_pending || axi_wready)) state<=Q_RESP;
         end
         Q_RESP:if(axi_bvalid) begin
           if(axi_bresp!=0) local_fault<=1;
           else begin
             state<=Q_IDLE;
             if(send_queue) begin pending_valid<=0;queue_write_beats<=queue_write_beats+1'b1;end
             else dirty[send_index]<=0;
           end
         end
         default:local_fault<=1;
       endcase
     end
   end
 end
endmodule
