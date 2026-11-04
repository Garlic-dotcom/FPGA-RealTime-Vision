// Exact global 8-connected Canny hysteresis, using an external byte label plane
// and 32-bit queue entries. Labels: 0=background, 1=weak, 2=strong, 3=accepted.
// The front end must finish a COMPLETE new label plane before start. No other
// master may modify that plane/queue while busy. Each label occupies one byte.
//
// Memory contract: aligned 32-bit request, byte write strobes, one outstanding
// read, registered read response (at least one cycle after request acceptance).
// Optional wide scan also receives the complete coherent 16-byte line for its
// aligned label read, valid on the same cycle as mem_rvalid.
// The memory adapter MUST preserve request ordering, including write->read.
// A queue holds at most WIDTH*HEIGHT entries; only newly accepted pixels enter.
// Full-frame output follows convergence. Eight coherent label cache entries
// survive node changes; every accepted label write updates matching entries.
// No other master may modify the plane during a job. Reset validity on start.
module canny_hysteresis_engine #(parameter WIDTH=1280,HEIGHT=720,NO_WEAK_HINT_ENABLE=0,
    WIDE_SCAN_ENABLE=0,NEIGHBOR_LINE_CACHE_ENABLE=WIDE_SCAN_ENABLE,
    WIDE_EMIT_ENABLE=WIDE_SCAN_ENABLE)(
 input clk,rst_n,start,
 // With the hint enabled, caller proves the COMPLETE label plane contains no weak pixels.
 // It must wait for the final label write response before asserting start.
 input no_weak_hint,
 input [31:0] label_base,queue_base,
 output reg busy,done,
 output reg mem_valid,mem_write,
 output reg [31:0] mem_address,mem_wdata,
 output reg [3:0] mem_wstrb,
 input mem_ready,mem_rvalid,
 input [31:0] mem_rdata,
 input [127:0] mem_line_rdata,
 output source_valid,source_sop,source_eop,
 output [7:0] source_data,
 input source_ready,
 output reg [31:0] cycle_count,accepted_count
);
 localparam N=WIDTH*HEIGHT,IW=$clog2(N),XW=$clog2(WIDTH),QW=$clog2(N+1);
 localparam IDLE=0,SCAN_REQ=1,SCAN_WAIT=2,SCAN_EVAL=3,SEED_MARK=4,SEED_PUSH=5,SCAN_NEXT=6,
            POP_CHECK=7,POP_REQ=8,POP_WAIT=9,NEIGH_CHECK=10,NEIGH_REQ=11,NEIGH_WAIT=12,
            NEIGH_MARK=13,NEIGH_PUSH=14,NEIGH_NEXT=15,EMIT_REQ=16,EMIT_WAIT=17,EMIT=18,FINISH=19,EMIT_NEXT_WAIT=20;
 reg [4:0] state;
 reg seed_visit,no_weak_known;
 reg [31:0] labels_addr,queue_addr,scan_word;
 reg [15:0] emit_word;
 reg wide_emit_active;
 // Four future words absorb ordered read latency. This is an output pipeline,
 // separate from the coherent neighborhood cache; only one read is in flight.
 reg [15:0] emit_prefetch[0:3];
 reg [1:0] prefetch_head,prefetch_tail;
 reg [2:0] prefetch_count;
 reg [IW:0] next_emit_index;
 reg prefetch_requested;
 // With aligned wide scanning, each entry holds the bridge's complete
 // coherent 16-byte line. The old 32-bit cache behavior remains for jobs
 // that fall back from wide scanning and when WIDE_SCAN_ENABLE is zero.
 reg [127:0] neighbor_lines[0:7];
 reg [IW-1:0] neighbor_tags[0:7];
 reg [7:0] neighbor_valids;
 reg [2:0] neighbor_replace,neighbor_hit_index;
 reg neighbor_hit;
 integer cache_lookup,cache_update;
 reg [IW-1:0] scan_index,current_index,emit_index;
 reg [XW-1:0] scan_x,current_x;
 // A wide scan examines each aligned 16-label line once, then visits the
 // strong seeds in raster order. Weak-to-accepted writes never create seeds.
 reg wide_scan_active;
 reg [IW-1:0] scan_line_base;
 reg [XW-1:0] scan_line_x;
 reg [15:0] scan_strong_mask;
 wire line_cache_active=NEIGHBOR_LINE_CACHE_ENABLE && wide_scan_active;
 reg [QW-1:0] head,tail;
 reg [2:0] neighbor;
 // Capture all complete label addresses on the original node-load edge.
 // No request/response cycle is added; current_index and address table agree
 // before the first NEIGH_CHECK, including IW wrap and arbitrary label base.
 reg [31:0] neighbor_addresses[0:7];
 // Same node-load edges capture IW-truncated indices for coherent cache probes.
 // Reset matches the eight original displacements from current_index=0.
 reg [IW-1:0] neighbor_indices[0:7];
 integer address_slot;
 wire neighbor_address_load=(state==SEED_MARK && mem_ready && !no_weak_known) ||
     (state==POP_WAIT && mem_rvalid);
 wire [IW-1:0] neighbor_load_index=(state==POP_WAIT)?mem_rdata[IW-1:0]:scan_index;
 reg [IW-1:0] neighbor_index;
 reg [XW-1:0] neighbor_x;
 reg neighbor_valid;
 wire [31:0] scan_byte=scan_word>>(8*scan_index[1:0]);
 wire [31:0] neighbor_byte=mem_rdata>>(8*neighbor_index[1:0]);
 wire [IW-1:0] cache_probe_index=(state==SCAN_REQ || state==SCAN_WAIT)?
     (wide_scan_active?scan_line_base:scan_index):neighbor_index;
 wire [1:0] cache_word_slot=(cache_probe_index>>2)&2'b11;
 wire [127:0] neighbor_line=neighbor_lines[neighbor_hit_index];
 wire [31:0] neighbor_word=line_cache_active?
     (neighbor_line>>(32*cache_word_slot)):neighbor_line[31:0];
 wire [31:0] cached_neighbor_byte=neighbor_word>>(8*neighbor_index[1:0]);
 wire [3:0] scan_cache_byte_lane=line_cache_active?(scan_index&4'hf):
     {2'b0,scan_index[1:0]};
 wire [3:0] neighbor_cache_byte_lane=line_cache_active?(neighbor_index&4'hf):
     {2'b0,neighbor_index[1:0]};
 // Complete ordinary and wide tag comparisons before choosing scan mode.
 // The original probe remains above for exact X/Z mode fallback and diagnostics.
 wire [IW-1:0] narrow_cache_probe_index=(state==SCAN_REQ || state==SCAN_WAIT)?
     scan_index:neighbor_index;
 wire [IW-1:0] wide_cache_probe_index=(state==SCAN_REQ || state==SCAN_WAIT)?
     scan_line_base:neighbor_index;
 (* syn_keep = "true" *) wire [7:0] narrow_cache_match;
 (* syn_keep = "true" *) wire [7:0] wide_cache_match;
 genvar latewide_slot;
 generate
   for(latewide_slot=0;latewide_slot<8;latewide_slot=latewide_slot+1) begin: latewide_compare
     assign narrow_cache_match[latewide_slot]=
         (neighbor_tags[latewide_slot]>>2)==(narrow_cache_probe_index>>2);
     assign wide_cache_match[latewide_slot]=NEIGHBOR_LINE_CACHE_ENABLE?
         (neighbor_tags[latewide_slot]>>4)==(wide_cache_probe_index>>4):
         (neighbor_tags[latewide_slot]>>2)==(wide_cache_probe_index>>2);
   end
 endgenerate
 // Precompute bool hits for every already-loaded direction before selector.
 // These queries never depend on neighbor, scan state or wide_scan_active.
 (* syn_keep = "true" *) wire [7:0] direction_normal_hit;
 (* syn_keep = "true" *) wire [7:0] direction_wide_hit;
 genvar hit_direction,hit_slot;
 generate
   for(hit_direction=0;hit_direction<8;hit_direction=hit_direction+1) begin: direction_hit_query
     (* syn_keep = "true" *) wire [7:0] normal_match;
     (* syn_keep = "true" *) wire [7:0] wide_match;
     for(hit_slot=0;hit_slot<8;hit_slot=hit_slot+1) begin: slot_compare
       assign normal_match[hit_slot]=(neighbor_tags[hit_slot]>>2)==
           (neighbor_indices[hit_direction]>>2);
       assign wide_match[hit_slot]=NEIGHBOR_LINE_CACHE_ENABLE?
           (neighbor_tags[hit_slot]>>4)==(neighbor_indices[hit_direction]>>4):
           (neighbor_tags[hit_slot]>>2)==(neighbor_indices[hit_direction]>>2);
     end
     reg [7:0] normal_qualified_value,wide_qualified_value;
     (* syn_keep = "true" *) wire [7:0] normal_qualified=normal_qualified_value;
     (* syn_keep = "true" *) wire [7:0] wide_qualified=wide_qualified_value;
     integer normalize_slot;
     always @* begin
       normal_qualified_value=0;wide_qualified_value=0;
       for(normalize_slot=0;normalize_slot<8;normalize_slot=normalize_slot+1) begin
         if(neighbor_valids[normalize_slot] && normal_match[normalize_slot])
           normal_qualified_value[normalize_slot]=1'b1;
         if(neighbor_valids[normalize_slot] && wide_match[normalize_slot])
           wide_qualified_value[normalize_slot]=1'b1;
       end
     end
     // No priority pair nets feed this independent two-level OR4/OR2 intent.
     (* syn_keep = "true" *) wire [1:0] normal_hit_halves={
         |normal_qualified[7:4],|normal_qualified[3:0]};
     (* syn_keep = "true" *) wire [1:0] wide_hit_halves={
         |wide_qualified[7:4],|wide_qualified[3:0]};
     assign direction_normal_hit[hit_direction]=|normal_hit_halves;
     assign direction_wide_hit[hit_direction]=|wide_hit_halves;
   end
 endgenerate
 // Independent scan queries have no state, mode, or neighbor selector input.
 // Normal scan addresses scan_index; wide scan addresses scan_line_base.
 (* syn_keep = "true" *) wire [7:0] scan_normal_match;
 (* syn_keep = "true" *) wire [7:0] scan_wide_match;
 genvar scan_hit_slot;
 generate
   for(scan_hit_slot=0;scan_hit_slot<8;scan_hit_slot=scan_hit_slot+1) begin: scan_hit_query
     assign scan_normal_match[scan_hit_slot]=(neighbor_tags[scan_hit_slot]>>2)==
         (scan_index>>2);
     assign scan_wide_match[scan_hit_slot]=NEIGHBOR_LINE_CACHE_ENABLE?
         (neighbor_tags[scan_hit_slot]>>4)==(scan_line_base>>4):
         (neighbor_tags[scan_hit_slot]>>2)==(scan_line_base>>2);
   end
 endgenerate
 reg [7:0] scan_normal_qualified_value,scan_wide_qualified_value;
 (* syn_keep = "true" *) wire [7:0] scan_normal_qualified=scan_normal_qualified_value;
 (* syn_keep = "true" *) wire [7:0] scan_wide_qualified=scan_wide_qualified_value;
 integer scan_normalize_slot;
 always @* begin
   scan_normal_qualified_value=0;scan_wide_qualified_value=0;
   for(scan_normalize_slot=0;scan_normalize_slot<8;scan_normalize_slot=scan_normalize_slot+1) begin
     if(neighbor_valids[scan_normalize_slot] && scan_normal_match[scan_normalize_slot])
       scan_normal_qualified_value[scan_normalize_slot]=1'b1;
     if(neighbor_valids[scan_normalize_slot] && scan_wide_match[scan_normalize_slot])
       scan_wide_qualified_value[scan_normalize_slot]=1'b1;
   end
 end
 (* syn_keep = "true" *) wire [1:0] scan_normal_hit_halves={
     |scan_normal_qualified[7:4],|scan_normal_qualified[3:0]};
 (* syn_keep = "true" *) wire [1:0] scan_wide_hit_halves={
     |scan_wide_qualified[7:4],|scan_wide_qualified[3:0]};
 (* syn_keep = "true" *) wire scan_normal_hit=|scan_normal_hit_halves;
 (* syn_keep = "true" *) wire scan_wide_hit=|scan_wide_hit_halves;
 always @* begin
   neighbor_hit=0;neighbor_hit_index=0;
   for(cache_lookup=0;cache_lookup<8;cache_lookup=cache_lookup+1) begin
     // Ordinary case and if retain unknown-condition collapse and highest slot.
     case(wide_scan_active)
       1'b0:if(neighbor_valids[cache_lookup] && narrow_cache_match[cache_lookup]) begin
         neighbor_hit=1;neighbor_hit_index=cache_lookup;
       end
       1'b1:if(neighbor_valids[cache_lookup] && wide_cache_match[cache_lookup]) begin
         neighbor_hit=1;neighbor_hit_index=cache_lookup;
       end
       default:begin
       if(neighbor_valids[cache_lookup] &&
          (line_cache_active?
             (neighbor_tags[cache_lookup]>>4)==(cache_probe_index>>4):
             (neighbor_tags[cache_lookup]>>2)==(cache_probe_index>>2))) begin
         neighbor_hit=1;neighbor_hit_index=cache_lookup;
       end
       end
     endcase
   end
   // Every known state/mode/direction picks an independently precomputed hit.
   // The complete original priority index and all X/Z fallback remain above.
   case(state)
     5'd1,5'd2:begin
       case(wide_scan_active)
         1'b0:begin
           case(neighbor)
             3'd0,3'd1,3'd2,3'd3,3'd4,3'd5,3'd6,3'd7:neighbor_hit=scan_normal_hit;
           endcase
         end
         1'b1:begin
           case(neighbor)
             3'd0,3'd1,3'd2,3'd3,3'd4,3'd5,3'd6,3'd7:neighbor_hit=scan_wide_hit;
           endcase
         end
       endcase
     end
     5'd0,5'd3,5'd4,5'd5,5'd6,5'd7,5'd8,5'd9,5'd10,5'd11,
     5'd12,5'd13,5'd14,5'd15,5'd16,5'd17,5'd18,5'd19,5'd20,5'd21,
     5'd22,5'd23,5'd24,5'd25,5'd26,5'd27,5'd28,5'd29,5'd30,5'd31:begin
       case(wide_scan_active)
         1'b0:begin
           case(neighbor)
             0:neighbor_hit=direction_normal_hit[0];
             1:neighbor_hit=direction_normal_hit[1];
             2:neighbor_hit=direction_normal_hit[2];
             3:neighbor_hit=direction_normal_hit[3];
             4:neighbor_hit=direction_normal_hit[4];
             5:neighbor_hit=direction_normal_hit[5];
             6:neighbor_hit=direction_normal_hit[6];
             7:neighbor_hit=direction_normal_hit[7];
           endcase
         end
         1'b1:begin
           case(neighbor)
             0:neighbor_hit=direction_wide_hit[0];
             1:neighbor_hit=direction_wide_hit[1];
             2:neighbor_hit=direction_wide_hit[2];
             3:neighbor_hit=direction_wide_hit[3];
             4:neighbor_hit=direction_wide_hit[4];
             5:neighbor_hit=direction_wide_hit[5];
             6:neighbor_hit=direction_wide_hit[6];
             7:neighbor_hit=direction_wide_hit[7];
           endcase
         end
       endcase
     end
   endcase
 end
 // After convergence only accepted/not-accepted bits are needed for output.
 // Full line data is already returned by the existing coherent label adapter.
 // Narrow/unaligned integrations retain four pixels per memory request.
 wire [3:0] emit_lane=wide_emit_active?(emit_index & 4'hf):{2'b0,emit_index[1:0]};
 wire emit_last_lane=wide_emit_active?(emit_lane==4'd15):(emit_lane==4'd3);
 wire [IW:0] emit_step=wide_emit_active?16:4;
 wire [15:0] emit_response;
 genvar emit_slot;
 generate for(emit_slot=0;emit_slot<16;emit_slot=emit_slot+1) begin: emit_decode
   if(emit_slot<4) begin: low_word
     assign emit_response[emit_slot]=wide_emit_active?
       (mem_line_rdata[emit_slot*8+:2]==2'd3):(mem_rdata[emit_slot*8+:2]==2'd3);
   end else begin: upper_words
     assign emit_response[emit_slot]=wide_emit_active && (mem_line_rdata[emit_slot*8+:2]==2'd3);
   end
 end endgenerate
 wire needs_emit_word=(state==EMIT_NEXT_WAIT) ||
     (state==EMIT && source_ready && emit_index!=N-1 && emit_last_lane);
 wire prefetch_pop=needs_emit_word && prefetch_count!=0;
 wire prefetch_push=mem_rvalid && !(needs_emit_word && prefetch_count==0);
 // A word without strong seeds needs no per-byte scan. The X position still
 // wraps correctly for widths not divisible by four; final padding is ignored.
 wire skip_scan_word=WIDTH>=4 && scan_index[1:0]==0 &&
     scan_word[1:0]!=2 && scan_word[9:8]!=2 && scan_word[17:16]!=2 && scan_word[25:24]!=2;
 wire advance_scan=!wide_scan_active && ((state==SCAN_EVAL && scan_byte[1:0]!=2'd2) ||
     (state==SCAN_NEXT) || (state==SEED_MARK && no_weak_known && mem_ready));
 wire [3:0] strong_groups={|scan_strong_mask[15:12],|scan_strong_mask[11:8],
                            |scan_strong_mask[7:4],|scan_strong_mask[3:0]};
 wire [1:0] strong_group=strong_groups[0]?2'd0:strong_groups[1]?2'd1:
                         strong_groups[2]?2'd2:2'd3;
 wire [3:0] strong_nibble=scan_strong_mask>>(4*strong_group);
 wire [1:0] strong_position=strong_nibble[0]?2'd0:strong_nibble[1]?2'd1:
                            strong_nibble[2]?2'd2:2'd3;
 wire [3:0] strong_offset={strong_group,strong_position};
 wire advance_neighbor=(state==NEIGH_CHECK &&
     (!neighbor_valid || (neighbor_hit && cached_neighbor_byte[1:0]!=2'd1))) ||
     (state==NEIGH_WAIT && mem_rvalid && neighbor_byte[1:0]!=2'd1) ||
     (state==NEIGH_PUSH && mem_ready);
 // Only skip neighbors already proven non-weak in the SAME coherent word.
 // Horizontal border checks prevent any row-wrap or truncated neighborhood.
 wire [31:0] checked_word=(state==NEIGH_WAIT)?mem_rdata:neighbor_word;
 wire [31:0] checked_bytes=checked_word>>(8*neighbor_index[1:0]);
 wire group_nonweak=left_ok && right_ok && neighbor_index[1:0]<=1 &&
     checked_bytes[1:0]!=1 && checked_bytes[17:16]!=1 &&
     ((neighbor==3) || checked_bytes[9:8]!=1);
 wire skip_group=group_nonweak && (neighbor==0 || neighbor==3 || neighbor==5) &&
     ((state==NEIGH_CHECK && neighbor_valid && neighbor_hit) ||
      (state==NEIGH_WAIT && mem_rvalid));
 wire up_ok=(current_index>=WIDTH),down_ok=(current_index<N-WIDTH);
 wire left_ok=(current_x!=0),right_ok=(current_x!=WIDTH-1);
 always @* begin
   neighbor_index=current_index;neighbor_x=current_x;neighbor_valid=0;
   case(neighbor)
     0:begin neighbor_index=neighbor_indices[0];neighbor_x=current_x-1'b1;neighbor_valid=up_ok && left_ok;end
     1:begin neighbor_index=neighbor_indices[1];neighbor_valid=up_ok;end
     2:begin neighbor_index=neighbor_indices[2];neighbor_x=current_x+1'b1;neighbor_valid=up_ok && right_ok;end
     3:begin neighbor_index=neighbor_indices[3];neighbor_x=current_x-1'b1;neighbor_valid=left_ok;end
     4:begin neighbor_index=neighbor_indices[4];neighbor_x=current_x+1'b1;neighbor_valid=right_ok;end
     5:begin neighbor_index=neighbor_indices[5];neighbor_x=current_x-1'b1;neighbor_valid=down_ok && left_ok;end
     6:begin neighbor_index=neighbor_indices[6];neighbor_valid=down_ok;end
     7:begin neighbor_index=neighbor_indices[7];neighbor_x=current_x+1'b1;neighbor_valid=down_ok && right_ok;end
   endcase
 end
 function [31:0] label_word_address;
   input [IW-1:0] index;
   begin label_word_address=labels_addr+((32'b0+index)&32'hfffffffc);end
 endfunction
 function [IW-1:0] loaded_neighbor;
   input [IW-1:0] index;
   input [2:0] direction;
   begin
     case(direction)
       0:loaded_neighbor=index-(WIDTH+1);
       1:loaded_neighbor=index-WIDTH;
       2:loaded_neighbor=index-(WIDTH-1);
       3:loaded_neighbor=index-1'b1;
       4:loaded_neighbor=index+1'b1;
       5:loaded_neighbor=index+(WIDTH-1);
       6:loaded_neighbor=index+WIDTH;
       7:loaded_neighbor=index+(WIDTH+1);
       default:loaded_neighbor=index;
     endcase
   end
 endfunction
 // Ordinary case retains the original default when neighbor contains X/Z.
 // Every binary selector picks the address captured with current_index.
 reg [31:0] neighbor_label_address;
 always @* begin
   neighbor_label_address=label_word_address(neighbor_index);
   case(neighbor)
     0:neighbor_label_address=neighbor_addresses[0];
     1:neighbor_label_address=neighbor_addresses[1];
     2:neighbor_label_address=neighbor_addresses[2];
     3:neighbor_label_address=neighbor_addresses[3];
     4:neighbor_label_address=neighbor_addresses[4];
     5:neighbor_label_address=neighbor_addresses[5];
     6:neighbor_label_address=neighbor_addresses[6];
     7:neighbor_label_address=neighbor_addresses[7];
   endcase
 end
 function [31:0] queue_entry;
   input [IW-1:0] index;
   input [XW-1:0] xpos;
   begin queue_entry={{(32-IW-XW){1'b0}},xpos,index};end
 endfunction
 // Parallel address terms preserve the original output in every four-state case.
 // Only pair outputs are kept: each two masked terms fits one four-input LUT.
 // The original block remains the exact fallback for selected X/Z address bits.
 reg [31:0] original_mem_address;
 reg [31:0] scan_address_candidate,neighbor_address_candidate;
 reg [7:0] address_enable;
 (* syn_keep = "true" *) wire [31:0] ma_pair_0,ma_pair_1,ma_pair_2,ma_pair_3;
 wire [31:0] ma_parallel_address;
 always @* begin
   mem_valid=0;mem_write=0;original_mem_address=0;mem_wdata=0;mem_wstrb=0;
   case(state)
     SCAN_REQ:begin
       if(wide_scan_active || !neighbor_hit) begin
         mem_valid=1;
         original_mem_address=label_word_address(wide_scan_active?scan_line_base:scan_index);
       end
       // Known inactive hit exposes its harmless candidate address.
       // Ordinary case keeps every X/Z selector or mode on the original path.
       case(neighbor)
         3'd0,3'd1,3'd2,3'd3,3'd4,3'd5,3'd6,3'd7:begin
           case({wide_scan_active,neighbor_hit})
             2'b00,2'b01:original_mem_address=label_word_address(scan_index);
             2'b10,2'b11:original_mem_address=label_word_address(scan_line_base);
             default:begin end
           endcase
         end
         default:begin end
       endcase
     end
     SEED_MARK:begin
       mem_valid=1;mem_write=1;original_mem_address=label_word_address(scan_index);
       mem_wdata=32'd3<<(8*scan_index[1:0]);mem_wstrb=4'b0001<<scan_index[1:0];
     end
     // Issue on the decision cycle. Indices/state remain stable until ready.
     POP_CHECK:if(head!=tail) begin mem_valid=1;original_mem_address=queue_addr+(32'b0+head)*4;end
     NEIGH_CHECK:begin
       if(neighbor_valid && !neighbor_hit) begin mem_valid=1;original_mem_address=neighbor_label_address;end
       // Binary cases expose the candidate. Only mem_valid sends a request.
       // All X/Z control values retain the original conditional address.
       case(neighbor)
         3'd0,3'd1,3'd2,3'd3,3'd4,3'd5,3'd6,3'd7:begin
           case(wide_scan_active)
             1'b0,1'b1:begin
               case({neighbor_valid,neighbor_hit})
                 2'b00,2'b01,2'b10,2'b11:original_mem_address=neighbor_label_address;
                 default:begin end
               endcase
             end
             default:begin end
           endcase
         end
         default:begin end
       endcase
     end
     NEIGH_MARK:begin
       mem_valid=1;mem_write=1;original_mem_address=neighbor_label_address;
       mem_wdata=32'd3<<(8*neighbor_index[1:0]);mem_wstrb=4'b0001<<neighbor_index[1:0];
     end
     NEIGH_PUSH:begin
       mem_valid=1;mem_write=1;original_mem_address=queue_addr+(32'b0+tail)*4;
       mem_wdata=queue_entry(neighbor_index,neighbor_x);mem_wstrb=4'b1111;
     end
     EMIT_REQ:begin mem_valid=1;original_mem_address=label_word_address(emit_index);end
     EMIT,EMIT_NEXT_WAIT:if(!prefetch_requested && prefetch_count<4 && next_emit_index<N) begin
       mem_valid=1;original_mem_address=label_word_address(next_emit_index[IW-1:0]);
     end
   endcase
 end
 always @* begin
   scan_address_candidate=0;
       if(wide_scan_active || !neighbor_hit) begin
         scan_address_candidate=labels_addr+((32'b0+(wide_scan_active?scan_line_base:scan_index))&32'hfffffffc);
       end
       // Known inactive hit exposes its harmless candidate address.
       // Ordinary case keeps every X/Z selector or mode on the original path.
       case(neighbor)
         3'd0,3'd1,3'd2,3'd3,3'd4,3'd5,3'd6,3'd7:begin
           case({wide_scan_active,neighbor_hit})
             2'b00,2'b01:scan_address_candidate=labels_addr+((32'b0+scan_index)&32'hfffffffc);
             2'b10,2'b11:scan_address_candidate=labels_addr+((32'b0+scan_line_base)&32'hfffffffc);
             default:begin end
           endcase
         end
         default:begin end
       endcase
   neighbor_address_candidate=0;
       if(neighbor_valid && !neighbor_hit) begin neighbor_address_candidate=neighbor_label_address;end
       // Binary cases expose the candidate. Only mem_valid sends a request.
       // All X/Z control values retain the original conditional address.
       case(neighbor)
         3'd0,3'd1,3'd2,3'd3,3'd4,3'd5,3'd6,3'd7:begin
           case(wide_scan_active)
             1'b0,1'b1:begin
               case({neighbor_valid,neighbor_hit})
                 2'b00,2'b01,2'b10,2'b11:neighbor_address_candidate=neighbor_label_address;
                 default:begin end
               endcase
             end
             default:begin end
           endcase
         end
         default:begin end
       endcase
 end
 // Ordinary case emits known zero/one enables. A partially unknown state
 // selects no term, matching the original default address zero.
 always @* begin
   address_enable=0;
   case(state)
     SCAN_REQ:address_enable[0]=1;
     SEED_MARK:address_enable[1]=1;
     POP_CHECK:if(head!=tail) address_enable[2]=1;
     NEIGH_CHECK:address_enable[3]=1;
     NEIGH_MARK:address_enable[4]=1;
     NEIGH_PUSH:address_enable[5]=1;
     EMIT_REQ:address_enable[6]=1;
     EMIT,EMIT_NEXT_WAIT:
       if(!prefetch_requested && prefetch_count<4 && next_emit_index<N)
         address_enable[7]=1;
   endcase
 end
 assign ma_pair_0=({32{address_enable[0]}} & scan_address_candidate) |
     ({32{address_enable[1]}} & (labels_addr+((32'b0+scan_index)&32'hfffffffc)));
 assign ma_pair_1=({32{address_enable[2]}} & (queue_addr+(32'b0+head)*4)) |
     ({32{address_enable[3]}} & neighbor_address_candidate);
 assign ma_pair_2=({32{address_enable[4]}} & neighbor_label_address) |
     ({32{address_enable[5]}} & (queue_addr+(32'b0+tail)*4));
 assign ma_pair_3=({32{address_enable[6]}} & (labels_addr+((32'b0+emit_index)&32'hfffffffc))) |
     ({32{address_enable[7]}} & (labels_addr+((32'b0+next_emit_index[IW-1:0])&32'hfffffffc)));
 assign ma_parallel_address=ma_pair_0 | ma_pair_1 | ma_pair_2 | ma_pair_3;
 always @* begin
   mem_address=original_mem_address;
   // Masked OR changes a selected Z to X. Detect every nonbinary result
   // and return the original expression, preserving the direct Z exactly.
   case(^ma_parallel_address)
     1'b0,1'b1:mem_address=ma_parallel_address;
     default:begin end
   endcase
 end
 assign source_valid=(state==EMIT);
 assign source_sop=source_valid && emit_index==0;
 assign source_eop=source_valid && emit_index==N-1;
 assign source_data=emit_word[emit_lane]?8'hff:8'h00;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=IDLE;busy<=0;done<=0;cycle_count<=0;accepted_count<=0;seed_visit<=0;no_weak_known<=0;
     labels_addr<=0;queue_addr<=0;scan_word<=0;emit_word<=0;wide_emit_active<=0;
     prefetch_head<=0;prefetch_tail<=0;prefetch_count<=0;next_emit_index<=0;prefetch_requested<=0;
     neighbor_valids<=0;neighbor_replace<=0;
     scan_index<=0;scan_x<=0;current_index<=0;current_x<=0;emit_index<=0;
     wide_scan_active<=0;scan_line_base<=0;scan_line_x<=0;scan_strong_mask<=0;
     head<=0;tail<=0;neighbor<=0;
     for(address_slot=0;address_slot<8;address_slot=address_slot+1) begin
       neighbor_addresses[address_slot]<=0;
       neighbor_indices[address_slot]<=loaded_neighbor({IW{1'b0}},address_slot);
     end
   end else begin
     done<=0;
     if(busy) cycle_count<=cycle_count+1'b1;
     if(neighbor_address_load)
       for(address_slot=0;address_slot<8;address_slot=address_slot+1) begin
         neighbor_addresses[address_slot]<=label_word_address(loaded_neighbor(neighbor_load_index,address_slot));
         neighbor_indices[address_slot]<=loaded_neighbor(neighbor_load_index,address_slot);
       end
     // Fill on neighborhood misses. Wide scan reads can prewarm a line for
     // nearby seeds, but a line already present must not create duplicate
     // tags or displace a useful cache entry. Legacy scan fills stay 32-bit.
     if(mem_rvalid && (state==NEIGH_WAIT ||
        (state==SCAN_WAIT && (!wide_scan_active || (line_cache_active && !neighbor_hit))))) begin
       neighbor_lines[neighbor_replace]<=line_cache_active?mem_line_rdata:{96'd0,mem_rdata};
       neighbor_tags[neighbor_replace]<=(state==SCAN_WAIT)?
           (wide_scan_active?scan_line_base:scan_index):neighbor_index;
       neighbor_valids[neighbor_replace]<=1;
       neighbor_replace<=neighbor_replace+1'b1;
     end
     case(state)
       IDLE:if(start) begin
         busy<=1;cycle_count<=0;accepted_count<=0;head<=0;tail<=0;
         scan_index<=0;scan_x<=0;emit_index<=0;
         wide_scan_active<=WIDE_SCAN_ENABLE && WIDTH%16==0 && label_base[3:0]==0;
         scan_line_base<=0;scan_line_x<=0;scan_strong_mask<=0;
         seed_visit<=0;no_weak_known<=NO_WEAK_HINT_ENABLE && no_weak_hint;
         wide_emit_active<=WIDE_EMIT_ENABLE && WIDTH%16==0 && label_base[3:0]==0;
         next_emit_index<=(WIDE_EMIT_ENABLE && WIDTH%16==0 && label_base[3:0]==0)?16:4;prefetch_requested<=0;prefetch_head<=0;prefetch_tail<=0;prefetch_count<=0;
         neighbor_valids<=0;neighbor_replace<=0;
         labels_addr<=label_base;queue_addr<=queue_base;state<=SCAN_REQ;
       end
       SCAN_REQ:begin
         if(!wide_scan_active && neighbor_hit) begin scan_word<=neighbor_word;state<=SCAN_EVAL;end
         else if(mem_ready) state<=SCAN_WAIT;
       end
       SCAN_WAIT:if(mem_rvalid) begin
         if(wide_scan_active) begin
           for(cache_update=0;cache_update<16;cache_update=cache_update+1)
             scan_strong_mask[cache_update]<=
                 mem_line_rdata[cache_update*8+:2]==2'd2;
         end else scan_word<=mem_rdata;
         state<=SCAN_EVAL;
       end
       SCAN_EVAL:if(wide_scan_active) begin
         if(scan_strong_mask!=0) begin
           scan_strong_mask[strong_offset]<=0;
           scan_index<=scan_line_base+strong_offset;
           scan_x<=scan_line_x+strong_offset;
           state<=SEED_MARK;
         end else if(({1'b0,scan_line_base}+16)>=N) state<=POP_CHECK;
         else begin
           scan_line_base<=scan_line_base+16;
           scan_line_x<=(scan_line_x==WIDTH-16)?0:scan_line_x+16;
           state<=SCAN_REQ;
         end
       end else if(scan_byte[1:0]==2'd2) state<=SEED_MARK;
       SEED_MARK:if(mem_ready) begin
         // Strong pixels are visited directly in raster order. Only accepted
         // weak pixels enter the queue, still propagating to global closure.
         accepted_count<=accepted_count+1'b1;
         // Seed writes are coherent too: a prior node may have cached this word.
         for(cache_update=0;cache_update<8;cache_update=cache_update+1)
           if(neighbor_valids[cache_update] &&
              (line_cache_active?
                (neighbor_tags[cache_update]>>4)==(scan_index>>4):
                (neighbor_tags[cache_update]>>2)==(scan_index>>2)))
             neighbor_lines[cache_update][scan_cache_byte_lane*8+:8]<=8'd3;
         if(!no_weak_known) begin
           current_index<=scan_index;current_x<=scan_x;neighbor<=0;
           seed_visit<=1;state<=NEIGH_CHECK;
         end else if(wide_scan_active) state<=SCAN_EVAL;
       end
       SCAN_NEXT:begin seed_visit<=0;if(wide_scan_active) state<=SCAN_EVAL;end
       POP_CHECK:begin
         if(head==tail) state<=EMIT_REQ;
         else if(mem_ready) state<=POP_WAIT;
       end
       POP_WAIT:if(mem_rvalid) begin
         current_index<=mem_rdata[IW-1:0];current_x<=mem_rdata[IW+XW-1:IW];
         head<=head+1'b1;seed_visit<=0;neighbor<=0;state<=NEIGH_CHECK;
       end
       NEIGH_CHECK:if(neighbor_valid) begin
         if(!neighbor_hit) begin if(mem_ready) state<=NEIGH_WAIT;end
         else if(cached_neighbor_byte[1:0]==2'd1) state<=NEIGH_MARK;
       end
       NEIGH_WAIT:if(mem_rvalid) begin
         if(neighbor_byte[1:0]==2'd1) state<=NEIGH_MARK;
       end
       NEIGH_MARK:if(mem_ready) begin
         // Mark-before-enqueue, including our local copy. The ordered memory
         // adapter fences the write before accepting the following queue push.
         neighbor_lines[neighbor_hit_index][neighbor_cache_byte_lane*8+:8]<=8'd3;
         state<=NEIGH_PUSH;
       end
       NEIGH_PUSH:if(mem_ready) begin tail<=tail+1'b1;accepted_count<=accepted_count+1'b1;end
       EMIT_REQ:if(mem_ready) state<=EMIT_WAIT;
       EMIT_WAIT:if(mem_rvalid) begin emit_word<=emit_response;state<=EMIT;end
       EMIT:if(source_ready) begin
         if(emit_index==N-1) state<=FINISH;
         else begin
           emit_index<=emit_index+1'b1;
           if(emit_last_lane) begin
             if(prefetch_count!=0 || mem_rvalid) begin
               emit_word<=prefetch_count!=0?emit_prefetch[prefetch_head]:emit_response;
             end else state<=EMIT_NEXT_WAIT;
           end
         end
       end
       EMIT_NEXT_WAIT:if(prefetch_count!=0 || mem_rvalid) begin
         emit_word<=prefetch_count!=0?emit_prefetch[prefetch_head]:emit_response;
         state<=EMIT;
       end
       FINISH:begin busy<=0;done<=1;state<=IDLE;end
       default:state<=IDLE;
     endcase
     // Resume the raster scan after a direct strong-seed visit. The saved
     // scan word may contain old weak labels; only strong labels are examined.
     // Weak-to-accepted changes cannot create or hide a later strong seed.
     if(advance_scan) begin
       if(state==SCAN_EVAL && skip_scan_word) begin
         if(({1'b0,scan_index}+4)>=N) state<=POP_CHECK;
         else begin
           scan_index<=scan_index+4;
           scan_x<=(scan_x>=WIDTH-4)?scan_x-(WIDTH-4):scan_x+4;
           state<=SCAN_REQ;
         end
       end else if(scan_index==N-1) state<=POP_CHECK;
       else begin
         scan_index<=scan_index+1'b1;scan_x<=(scan_x==WIDTH-1)?0:scan_x+1'b1;
         state<=(scan_index[1:0]==2'b11)?SCAN_REQ:SCAN_EVAL;
       end
     end
     // Skip invalid/non-weak neighbors and advance after enqueue without a
     // dedicated NEXT cycle. The final enqueue updates tail before POP_CHECK.
     if(advance_neighbor) begin
       if(neighbor==7 || (skip_group && neighbor==5)) state<=seed_visit?SCAN_NEXT:POP_CHECK;
       else begin
         neighbor<=skip_group?((neighbor==0)?3'd3:3'd5):neighbor+1'b1;
         state<=NEIGH_CHECK;
       end
     end
     // Retire one outstanding request into a bounded FIFO, independently of
     // pixel consumption. Directly bypass a response only when the FIFO is
     // empty and a new output word is needed. A stalled emit_word stays fixed.
     if(state==EMIT || state==EMIT_NEXT_WAIT) begin
       if(mem_valid && mem_ready) begin
         prefetch_requested<=1;next_emit_index<=next_emit_index+emit_step;
       end
       if(mem_rvalid) prefetch_requested<=0;
       if(prefetch_pop) prefetch_head<=prefetch_head+1'b1;
       if(prefetch_push) begin
         emit_prefetch[prefetch_tail]<=emit_response;prefetch_tail<=prefetch_tail+1'b1;
       end
       case({prefetch_push,prefetch_pop})
         2'b10:prefetch_count<=prefetch_count+1'b1;
         2'b01:prefetch_count<=prefetch_count-1'b1;
       endcase
     end
   end
 end
endmodule
