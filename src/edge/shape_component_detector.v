// Bounded full-resolution 8-connected EDGE components, no input backpressure.
// Raw IDs and tagged row profile banks are distinct. Raw neighbor equivalence
// events are buffered; EOP min-root union and in-place flatten avoid a 4-port canonical
// crossbar/full parallel table rewrite in the input path. Two EOP passes share
// profile RAM. The explicit event/deadline bounds also fail closed.
// Aggregate up to MAX_LABELS final components before selecting 16 candidates.
// Small early components no longer exhaust the candidate metadata table.
// Geometry is evaluated before the output limit; overflow still fails closed.
// Packed slot0 is least significant; 1=axis rectangle,2=circle,3=triangle (including rotated three-sided profiles).
module shape_component_detector #(
 parameter WIDTH=1280,HEIGHT=720,
 parameter ROI_X0=(WIDTH>16)?4:0,ROI_X1=(WIDTH>16)?WIDTH-4:WIDTH,
 parameter ROI_Y0=(HEIGHT>16)?4:0,ROI_Y1=(HEIGHT>16)?HEIGHT-4:HEIGHT,
 parameter MAX_LABELS=128,PROFILE_BANKS=18,MAX_COMPONENTS=16,MAX_OBJECTS=8,
 parameter EVENT_DEPTH=2048,ANALYSIS_LIMIT=200000,SKIP_BUSY_INPUT=1,
 parameter MAX_EDGE_PER_SPAN=12
)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,sink_edge,
 output reg result_valid,
 output reg [3:0] candidate_count,
 output reg [2*MAX_OBJECTS-1:0] shapes,
 output reg [12*MAX_OBJECTS-1:0] boxes_x0,boxes_x1,boxes_y0,boxes_y1,
 output reg [32*MAX_OBJECTS-1:0] edge_counts,
 output busy,
 output reg overrun,overflow,malformed,
 output reg [31:0] skipped_frames,
 // Sample with result_valid. Read-only diagnostics; no feedback into recognition.
 output [159:0] diagnostic_data,
 output frame_accepted
);
 localparam COLLECT=0,READ_META=1,MERGE_META=2,SELECT=3,READ_ROW=4,MERGE_ROW=5,FETCH_CAND=6,CALC_CAND=7,EVAL_CAND=8,FINISH=9;
 localparam INIT=10,EVENT_READ=11,EVENT_PREP=12,PAIR=13,FIND_A_READ=14,FIND_A_USE=15,FIND_B_READ=16,FIND_B_USE=17,LINK_LOSER=18,LINK_WINNER=19,FLAT_START=20,FLAT_READ=21,FLAT_USE=22,FLAT_STORE=23;
 localparam META_SLOT=24,META_UPDATE=25,SELECT_USE=26,ROW_SLOT=27,FINISH_LOAD=28,LOAD_CAND=29,PARENT_META=30,PARENT_ROW=31;
 localparam RANK_LOAD=32,RANK_CALC=33,RANK_USE=34,PUBLISH_LOAD=35,PUBLISH_USE=36,ROT_PREP=37,ROT_MUL=38,TRI_PREP=39,TRI_MUL=40,TRI_EVAL=41;
 localparam LW=$clog2(MAX_LABELS+1),BW=$clog2(PROFILE_BANKS+1),RW=$clog2(MAX_LABELS+1);
 localparam XW=$clog2(WIDTH),YW=$clog2(HEIGHT),CW=$clog2(WIDTH+1),TW=$clog2(MAX_LABELS),PW=TW+CW+2*XW;
 localparam DW=XW+2,YDW=YW+2,SUMW=((XW>YW)?XW:YW)+1,GEW=XW+15;
 localparam EW=$clog2(EVENT_DEPTH);
 reg [5:0] state;
 reg [11:0] in_x,in_y,dx,dy,scan_y;
 reg dv,ds,de,dp,frame_active,frame_fault;
 reg [LW-1:0] north_q,northeast_q,west,northwest;
 reg [LW-1:0] previous_line[0:WIDTH-1];
 reg [4*LW-1:0] events[0:EVENT_DEPTH-1],event_q;
 reg [LW-1:0] parents[1:MAX_LABELS],parent_q;
 reg [EW:0] event_count,event_index;
 reg [LW-1:0] init_index,flat_index,node,base_id,pair_id,root_a,root_b;
 reg [1:0] pair_index;
 reg [17:0] analysis_ticks;
 reg capture_frame;
 wire allow_sop=(state==COLLECT && !(dv && (de || (dx==WIDTH-1 && dy==HEIGHT-1))));
 wire accept_input=!SKIP_BUSY_INPUT || (sink_sop?allow_sop:capture_frame);
 wire [LW-1:0] event_neighbor=event_q[pair_index*LW+:LW];
 wire duplicate_neighbor=(pair_index>=1 && event_neighbor==event_q[0+:LW]) ||
                         (pair_index>=2 && event_neighbor==event_q[LW+:LW]) ||
                         (pair_index>=3 && event_neighbor==event_q[2*LW+:LW]);
 // A single synchronous read is shared by union/flatten and both profile
 // passes. During flatten all parents point to lower IDs, so ascending ID
 // traversal writes the final root in place; no second canonical table.
 wire [LW-1:0] parent_query=(state==READ_META || state==MERGE_META || state==PARENT_META || state==META_SLOT || state==META_UPDATE || state==READ_ROW || state==MERGE_ROW || state==PARENT_ROW || state==ROW_SLOT)?((profile_tag==0)?1:profile_tag):node;
 wire [LW-1:0] parent_now=parent_q;
 reg [LW-1:0] event_min;
 integer kevent;
 always @* begin
   event_min=0;
   for(kevent=0;kevent<4;kevent=kevent+1)
     if(event_q[kevent*LW+:LW]!=0 && (event_min==0 || event_q[kevent*LW+:LW]<event_min))event_min=event_q[kevent*LW+:LW];
 end
 wire a_wins=root_a<root_b;
 wire [LW-1:0] winner=a_wins?root_a:root_b,loser=a_wins?root_b:root_a;
 reg [RW-1:0] root_slot[1:MAX_LABELS];
 reg [LW:0] allocated;
 wire [11:0] sx=sink_sop?12'd0:in_x,sy=sink_sop?12'd0:in_y;
 wire hit=dp && dx>=ROI_X0 && dx<ROI_X1 && dy>=ROI_Y0 && dy<ROI_Y1;
 reg [LW-1:0] rw,rnw,rn,rne,min_root,chosen_label;
 reg allocate_pixel;
 reg [BW-1:0] chosen_bank,free_bank;
 reg [LW-1:0] row_tag[1:PROFILE_BANKS];
 reg [CW-1:0] row_count[1:PROFILE_BANKS];
 reg [XW-1:0] row_min[1:PROFILE_BANKS],row_max[1:PROFILE_BANKS];
 wire [PW-1:0] profile_q[1:PROFILE_BANKS];
 integer i,krow,kmerge;
 always @* begin
   rw=(dx==0 || ds)?0:west;rnw=(dx==0 || dy==0 || ds)?0:northwest;
   rn=(dy==0 || ds)?0:north_q;rne=(dx==WIDTH-1 || dy==0 || ds)?0:northeast_q;
   min_root=0;
   if(rw!=0)min_root=rw;if(rnw!=0 && (min_root==0 || rnw<min_root))min_root=rnw;
   if(rn!=0 && (min_root==0 || rn<min_root))min_root=rn;if(rne!=0 && (min_root==0 || rne<min_root))min_root=rne;
   allocate_pixel=hit && min_root==0 && !frame_fault;chosen_label=0;
   if(hit && (ds || !frame_fault)) begin
     if(ds)chosen_label=1;else if(min_root!=0)chosen_label=min_root;else if(allocated<MAX_LABELS)chosen_label=allocated+1'b1;
   end
   chosen_bank=0;free_bank=0;
   for(krow=1;krow<=PROFILE_BANKS;krow=krow+1) begin
     if(row_tag[krow]==0 && free_bank==0)free_bank=krow;
     if(chosen_label!=0 && row_tag[krow]==chosen_label)chosen_bank=krow;
   end
   if(chosen_label!=0 && chosen_bank==0)chosen_bank=free_bank;if(ds && hit)chosen_bank=1;
 end
 wire conflict=hit && min_root!=0 && ((rw!=0 && rw!=min_root) || (rnw!=0 && rnw!=min_root) || (rn!=0 && rn!=min_root) || (rne!=0 && rne!=min_root));
 // One event word per conflicting pixel, preserving all four 8-neighbors.
 // A new primitive writes its identity through the same single RAM port.
 // Only allocated IDs are traversed at EOP, so uninitialized/stale IDs are
 // never used. Input capture never reads the parent table.
 wire new_parent=dv && state==COLLECT && chosen_label!=0 && (ds || allocate_pixel) && (ds || !frame_fault);
 wire parent_we=new_parent || (state==LINK_LOSER && root_a!=root_b) || (state==FLAT_STORE);
 wire [LW-1:0] parent_wa=new_parent?chosen_label:(state==LINK_LOSER)?loser:flat_index;
 wire [LW-1:0] parent_wd=new_parent?chosen_label:(state==LINK_LOSER)?winner:node;
 always @(posedge clk) begin
   if(dv && state==COLLECT && !ds && conflict && !frame_fault && event_count<EVENT_DEPTH)events[event_count[EW-1:0]]<={rw,rnw,rn,rne};
   if(state==EVENT_READ)event_q<=events[event_index[EW-1:0]];
   parent_q<=parents[parent_query];if(parent_we)parents[parent_wa]<=parent_wd;
 end
 // N and NE synchronous reads are delayed with the current input pixel.
 // The prior accepted N register supplies NW even across valid bubbles.
  generate if(WIDTH>=3) begin: line_ahead
    wire [11:0] ahead_address=(sx==WIDTH-1)?12'd0:sx+12'd1;
    always @(posedge clk) begin
      if(sink_valid) begin
        north_q<=northeast_q;
        northeast_q<=previous_line[ahead_address];
      end
    end
  end else begin: line_dual
    always @(posedge clk) begin
      if(sink_valid) begin north_q<=(sy==0)?0:previous_line[sx];northeast_q<=(sy==0 || sx==WIDTH-1)?0:previous_line[sx+1'b1];end
    end
  end endgenerate
  always @(posedge clk) begin
    if(dv && (state==COLLECT || ds))previous_line[dx]<=chosen_label;
  end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin in_x<=0;in_y<=0;dx<=0;dy<=0;dv<=0;ds<=0;de<=0;dp<=0;capture_frame<=0;skipped_frames<=0;end
   else begin
     dv<=sink_valid && accept_input;ds<=sink_sop;de<=sink_eop;dp<=sink_edge;
     if(sink_valid && sink_sop)begin capture_frame<=allow_sop;if(SKIP_BUSY_INPUT && !allow_sop)skipped_frames<=skipped_frames+1'b1;end
     if(sink_valid) begin dx<=sx;dy<=sy;if(sx==WIDTH-1)begin in_x<=0;in_y<=(sy==HEIGHT-1)?0:sy+1'b1;end else begin in_x<=sx+1'b1;in_y<=sy;end end
   end
 end
 genvar g;
 generate for(g=1;g<=PROFILE_BANKS;g=g+1) begin: profile_bank
   reg [PW-1:0] memory[0:HEIGHT-1];reg [PW-1:0] q;
   always @(posedge clk) begin
     if(dv && (state==COLLECT || ds) && dx==WIDTH-1) begin
       if(chosen_bank==g && chosen_label!=0)memory[dy]<={chosen_label[TW-1:0],row_count[g]+{{CW-1{1'b0}},1'b1},(row_count[g]==0)?dx[XW-1:0]:row_min[g],dx[XW-1:0]};
       else memory[dy]<={row_tag[g][TW-1:0],row_count[g],row_min[g],row_max[g]};
     end
     if(state==READ_META || state==READ_ROW)q<=memory[scan_y];
   end
   assign profile_q[g]=q;
 end endgenerate
 reg [BW-1:0] bank_index;
 wire [TW-1:0] stored_tag=profile_q[bank_index][PW-1:PW-TW];
 wire [CW-1:0] profile_count=profile_q[bank_index][2*XW+CW-1:2*XW];
 // The all-zero tag encodes primitive 128 when count is nonzero. Empty banks
 // use count zero, saving one bit per 720-row word without reducing capacity.
 wire [LW-1:0] profile_tag=(profile_count==0)?0:(stored_tag==0)?(1<<TW):stored_tag;
 wire [XW-1:0] profile_min=profile_q[bank_index][2*XW-1:XW],profile_max=profile_q[bank_index][XW-1:0];
 wire [LW-1:0] profile_root=(profile_tag!=0)?parent_now:0;
 reg [RW-1:0] slot_q;
 wire [RW-1:0] existing_slot=slot_q;
 reg [RW-1:0] component_count;
 wire [RW-1:0] metadata_slot=(existing_slot!=0)?existing_slot:component_count+1'b1;
 // Packed coordinates reclaim unused 12-bit padding for extreme-row intervals.
 localparam MW=20+6*XW+6*YW;
 reg [MW-1:0] metadata[1:MAX_LABELS],packed_meta_q;
 wire [19:0] packed_area;
 wire [XW-1:0] px0,px1,pt0,pt1,pb0,pb1;
 wire [YW-1:0] py0,py1,ly_first,ly_last,ry_first,ry_last;
 assign {packed_area,px0,px1,py0,py1,pt0,pt1,pb0,pb1,ly_first,ly_last,ry_first,ry_last}=packed_meta_q;
 wire [115:0] meta_q={packed_area,{{(12-XW){1'b0}},px0},{{(12-XW){1'b0}},px1},{{(12-YW){1'b0}},py0},{{(12-YW){1'b0}},py1},{{(12-XW){1'b0}},pt0},{{(12-XW){1'b0}},pt1},{{(12-XW){1'b0}},pb0},{{(12-XW){1'b0}},pb1}};
 reg [YW-1:0] next_lfirst,next_llast,next_rfirst,next_rlast;
 wire [YW:0] ly_sum={1'b0,ly_first}+{1'b0,ly_last},ry_sum={1'b0,ry_first}+{1'b0,ry_last};
 wire [YW-1:0] left_y=ly_sum[YW:1],right_y=ry_sum[YW:1];
 wire [19:0] ma=meta_q[115:96];
 wire [11:0] mx0=meta_q[95:84],mx1=meta_q[83:72],my0=meta_q[71:60],my1=meta_q[59:48];
 wire [11:0] mt0=meta_q[47:36],mt1=meta_q[35:24],mb0=meta_q[23:12],mb1=meta_q[11:0];
 reg [RW-1:0] metadata_slot_reg;
 reg [115:0] meta_updated;
 always @* begin
   meta_updated=meta_q;meta_updated[115:96]=ma+profile_count;
   next_lfirst=ly_first;next_llast=ly_last;next_rfirst=ry_first;next_rlast=ry_last;
   if(ma==0 || profile_min<mx0)begin next_lfirst=scan_y;next_llast=scan_y;end
   else if(profile_min==mx0)next_llast=scan_y;
   if(ma==0 || profile_max>mx1)begin next_rfirst=scan_y;next_rlast=scan_y;end
   else if(profile_max==mx1)next_rlast=scan_y;
   if(profile_min<mx0)meta_updated[95:84]=profile_min;if(profile_max>mx1)meta_updated[83:72]=profile_max;
   if(scan_y<my0)begin meta_updated[71:60]=scan_y;meta_updated[47:36]=profile_min;meta_updated[35:24]=profile_max;end
   else if(scan_y==my0)begin if(profile_min<mt0)meta_updated[47:36]=profile_min;if(profile_max>mt1)meta_updated[35:24]=profile_max;end
   if(ma==0 || scan_y>my1)begin meta_updated[59:48]=scan_y;meta_updated[23:12]=profile_min;meta_updated[11:0]=profile_max;end
   else if(scan_y==my1)begin if(profile_min<mb0)meta_updated[23:12]=profile_min;if(profile_max>mb1)meta_updated[11:0]=profile_max;end
 end
 reg selected[1:MAX_LABELS];
 reg [RW-1:0] candidates[0:MAX_COMPONENTS-1];
 // Counts include MAX_COMPONENTS (16); four bits would wrap the full list.
 reg [RW-1:0] n_candidates,select_slot,ci,publish_slot;
 reg [RW-1:0] select_index,select_best_root;reg [19:0] select_best_count;
 // A complete contour may cover almost the full image. Reject clipped
 // components at the ROI boundary rather than restricting all objects to
 // the old central 3/4 region. Eight-pixel spans admit narrow rectangles;
 // classification below rejects tiny near-square raster ambiguities.
 wire coarse_ok=ma!=0 && mx1-mx0>=8 && my1-my0>=8 &&
                mx0>ROI_X0 && mx1<ROI_X1-1 && my0>ROI_Y0 && my1<ROI_Y1-1 &&
                ma>=((mx1-mx0+my1-my0)>>1) && ma<=(mx1-mx0+my1-my0)*20;
 wire better_root=!selected[select_index] && coarse_ok && (select_best_root==0 || ma>select_best_count);
 wire [RW-1:0] pick_root=better_root?select_index:select_best_root;
 reg [RW-1:0] merge_candidate;
 always @* begin merge_candidate=MAX_COMPONENTS;for(kmerge=0;kmerge<MAX_COMPONENTS;kmerge=kmerge+1)if(kmerge<n_candidates && candidates[kmerge]==existing_slot && existing_slot!=0)merge_candidate=kmerge;end
 reg [CW-1:0] merged_count[0:MAX_COMPONENTS-1];reg [XW-1:0] merged_min[0:MAX_COMPONENTS-1],merged_max[0:MAX_COMPONENTS-1];
 reg [71:0] statistics[0:MAX_COMPONENTS-1],stat_q;
 wire [9:0] so=stat_q[71:62],sr=stat_q[61:52],sc=stat_q[51:42],su=stat_q[41:32],sd=stat_q[31:22];
 // End-row maximum comparisons need only these exact predicates, not 22 count bits.
 wire top_strong=stat_q[21],top_small=stat_q[20],bottom_strong=stat_q[19],bottom_small=stat_q[18];
 wire [9:0] rotated_rows=stat_q[17:8];
 wire rotation_discontinuous=stat_q[7];
 wire [RW-1:0] cr=(ci<n_candidates)?candidates[ci]:0;
 // Candidate geometry and published boxes share the immutable metadata RAM.
 // LOAD_CAND adds one cycle per candidate instead of caching two wide
 // register read ports; recognition cadence is independent of display cadence.
 wire [115:0] cm=meta_q,fm=meta_q;
 wire [XW-1:0] cx0=cm[95:84],cx1=cm[83:72],ct0=cm[47:36],ct1=cm[35:24],cb0=cm[23:12],cb1=cm[11:0];
 wire [YW-1:0] cy0=cm[71:60],cy1=cm[59:48];
 wire [XW:0] top_center={1'b0,ct0}+{1'b0,ct1},bottom_center={1'b0,cb0}+{1'b0,cb1};
 reg [XW-1:0] sw,tol;reg [YW-1:0] sh;reg [2*SUMW-1:0] radius_squared;
 wire [SUMW-1:0] mean_span=({1'b0,sw}+{1'b0,sh})>>1;
 reg signed [DW-1:0] lx_delta,rx_delta;reg signed [YDW-1:0] yy_delta;
 reg [2*DW-1:0] l_squared,r_squared;reg [2*YDW-1:0] y_squared;
 reg signed [GEW-1:0] up_error,down_error,center_error;
 wire [31:0] radial_left=l_squared+y_squared,radial_right=r_squared+y_squared;
 // Coordinates are doubled. A one-pixel radial raster/Sobel error adds
 // 4*diameter+4 to r^2. Sobel expands the exterior boundary, so only the
 // upper band needs this allowance. Keep the lower band to reject diamonds.
 wire [31:0] circle_raster_allowance=({{(32-SUMW){1'b0}},mean_span}<<4)+32'd16;
 wire circle_fit=radial_left*4>=radius_squared*3 &&
                 radial_left*4<=radius_squared*5+circle_raster_allowance &&
                 radial_right*4>=radius_squared*3 &&
                 radial_right*4<=radius_squared*5+circle_raster_allowance;
 wire [31:0] triangle_limit=tol*sh*2;
 wire center_fit=center_error>=-$signed(triangle_limit) && center_error<=$signed(triangle_limit);
 wire up_fit=center_fit && up_error>=-$signed(triangle_limit) && up_error<=$signed(triangle_limit);
 wire down_fit=center_fit && down_error>=-$signed(triangle_limit) && down_error<=$signed(triangle_limit);
 wire [RW-1:0] fr=(publish_slot<n_candidates)?candidates[publish_slot]:0;
 wire [XW-1:0] fw=(fr!=0)?fm[83:72]-fm[95:84]:0;
 wire [YW-1:0] fh=(fr!=0)?fm[59:48]-fm[71:60]:0;
 wire occupied_row=scan_y>=cy0 && scan_y<=cy1 && merged_count[ci]!=0;
 wire row_rect={1'b0,merged_min[ci]}<=({1'b0,cx0}+{1'b0,tol}) && ({1'b0,merged_max[ci]}+{1'b0,tol})>={1'b0,cx1};
 wire next_top_strong=top_strong || (occupied_row && scan_y<=cy0+2 && merged_count[ci]*2>=sw);
 wire next_top_small=top_small && !(occupied_row && scan_y<=cy0+2 && merged_count[ci]*3>sw);
 wire next_bottom_strong=bottom_strong || (occupied_row && scan_y+2>=cy1 && merged_count[ci]*2>=sw);
 wire next_bottom_small=bottom_small && !(occupied_row && scan_y+2>=cy1 && merged_count[ci]*3>sw);
 // Two piecewise lines, selected at left/right extreme-row midpoints.
 wire [XW-1:0] top_x=top_center[XW:1],bottom_x=bottom_center[XW:1];
 wire [XW-1:0] lax=(scan_y<=left_y)?top_x:cx0,lbx=(scan_y<=left_y)?cx0:bottom_x;
 wire [YW-1:0] lay=(scan_y<=left_y)?cy0:left_y,lby=(scan_y<=left_y)?left_y:cy1;
 wire [XW-1:0] rax=(scan_y<=right_y)?top_x:cx1,rbx=(scan_y<=right_y)?cx1:bottom_x;
 wire [YW-1:0] ray=(scan_y<=right_y)?cy0:right_y,rby=(scan_y<=right_y)?right_y:cy1;
 reg signed [12:0] rldx,rldy,rlpx,rlpy,rrdx,rrdy,rrpx,rrpy;
 reg signed [25:0] rlproduct_a,rlproduct_b,rrproduct_a,rrproduct_b;
 reg [14:0] rllimit,rrlimit;
 reg [2:0] rotation_tol;
 reg [2*XW-1:0] previous_extrema[0:MAX_COMPONENTS-1],previous_extrema_q;
 reg signed [12:0] step_l,step_r;
 reg signed [25:0] step_product_l,step_product_r;
 reg check_step_l,check_step_r;
 always @(posedge clk)begin
   previous_extrema_q<=previous_extrema[ci[SW-1:0]];
   if(state==EVAL_CAND && occupied_row)previous_extrema[ci[SW-1:0]]<={merged_min[ci],merged_max[ci]};
 end
 wire signed [26:0] step_error_l=$signed(step_product_l)-$signed(rldx),step_error_r=$signed(step_product_r)-$signed(rrdx);
 wire [14:0] step_limit_l=2*(rladx+rldy),step_limit_r=2*(rradx+rrdy);
 wire step_bad=(check_step_l && (step_error_l>$signed({1'b0,step_limit_l}) || step_error_l< -$signed({1'b0,step_limit_l}))) ||
               (check_step_r && (step_error_r>$signed({1'b0,step_limit_r}) || step_error_r< -$signed({1'b0,step_limit_r})));
 wire signed [26:0] rlerror=$signed(rlproduct_a)-$signed(rlproduct_b),rrerror=$signed(rrproduct_a)-$signed(rrproduct_b);
 wire [12:0] rladx=rldx[12]?-rldx:rldx,rradx=rrdx[12]?-rrdx:rrdx;
 wire rotated_row_fit=rlerror<= $signed({1'b0,rllimit}) && rlerror>= -$signed({1'b0,rllimit}) &&
                      rrerror<= $signed({1'b0,rrlimit}) && rrerror>= -$signed({1'b0,rrlimit});
 wire [71:0] next_statistics={so+{9'd0,occupied_row},sr+{9'd0,(occupied_row && row_rect)},sc+{9'd0,(occupied_row && circle_fit)},su+{9'd0,(occupied_row && up_fit)},sd+{9'd0,(occupied_row && down_fit)},next_top_strong,next_top_small,next_bottom_strong,next_bottom_small,rotated_rows+{9'd0,(occupied_row && rotated_row_fit)},(rotation_discontinuous || (occupied_row && step_bad)),7'd0};
 // Explicit single read/write address muxes avoid inferring several RAM ports
 // from state-specific indexed expressions. Read enable is always active.
 wire slot_we=(state==INIT) || (state==META_SLOT && profile_count!=0 && profile_root!=0 && existing_slot==0 && component_count<MAX_LABELS);
 wire [LW-1:0] slot_wa=(state==INIT)?init_index:profile_root,slot_ra=(profile_root==0)?1:profile_root;
 wire [RW-1:0] slot_wd=(state==INIT)?0:metadata_slot;
 wire meta_we=(state==INIT && init_index<=MAX_LABELS) || state==META_UPDATE;
 wire [RW-1:0] meta_wa=(state==INIT)?init_index[RW-1:0]:metadata_slot_reg;
 wire [RW-1:0] meta_ra=(state==SELECT || state==SELECT_USE)?select_index:
                          (state==LOAD_CAND || state==FETCH_CAND || state==CALC_CAND || state==EVAL_CAND)?cr:
                          (state==FINISH_LOAD || state==ROT_PREP || state==ROT_MUL || state==TRI_PREP || state==TRI_MUL || state==TRI_EVAL || state==FINISH || state==PUBLISH_LOAD || state==PUBLISH_USE)?fr:metadata_slot;
 wire [MW-1:0] meta_wd=(state==INIT)?{20'd0,{XW{1'b1}},{XW{1'b0}},{YW{1'b1}},{YW{1'b0}},{XW{1'b1}},{XW{1'b0}},{XW{1'b1}},{XW{1'b0}},{(4*YW){1'b0}}}:
 {meta_updated[115:96],meta_updated[84+:XW],meta_updated[72+:XW],meta_updated[60+:YW],meta_updated[48+:YW],meta_updated[36+:XW],meta_updated[24+:XW],meta_updated[12+:XW],meta_updated[0+:XW],next_lfirst,next_llast,next_rfirst,next_rlast};
 // After classification reuse each statistics word for its ranking record;
 // the row-profile RAMs and their port counts are unchanged.
 localparam SW=(MAX_COMPONENTS>1)?$clog2(MAX_COMPONENTS):1;
 reg [RW-1:0] rank_index,rank_best;
 reg [MAX_COMPONENTS-1:0] rank_selected;
 reg [1:0] rank_best_shape,chosen_shape;
 reg [9:0] rank_best_fit,rank_best_rows;
 reg [11:0] rank_best_height;
 reg [19:0] rank_best_count;
 reg [21:0] rank_fit_left,rank_fit_right,rank_rows_left,rank_rows_right;
 reg [1:0] classified;
 wire legacy_density=ma<=(fw+fh)*MAX_EDGE_PER_SPAN;
 wire axis_rect=legacy_density && sr*10>=so*9 && top_strong && bottom_strong;
 reg signed [12:0] corner_lx,corner_rx,corner_ly,corner_ry;
 reg signed [25:0] corner_dot_x,corner_dot_y;
 reg [23:0] box_square;
 reg rot_corner_ok;
 wire signed [26:0] corner_dot=$signed(corner_dot_x)+$signed(corner_dot_y);
 wire [26:0] abs_corner_dot=corner_dot[26]?-corner_dot:corner_dot;
 wire [12:0] finish_tx=({1'b0,fm[47:36]}+{1'b0,fm[35:24]})>>1;
 wire [12:0] finish_bx=({1'b0,fm[23:12]}+{1'b0,fm[11:0]})>>1;
 wire signed [13:0] center_x_error=$signed({1'b0,finish_tx})+$signed({1'b0,finish_bx})-$signed({2'b0,fm[95:84]})-$signed({2'b0,fm[83:72]});
 wire signed [13:0] center_y_error=$signed({1'b0,left_y})+$signed({1'b0,right_y})-$signed({2'b0,fm[71:60]})-$signed({2'b0,fm[59:48]});
 wire rotated_rect=!rotation_discontinuous && rot_corner_ok && abs_corner_dot<=({3'd0,box_square}>>5) && rotated_rows*100>=(fh+1)*95;
 // The same T,L,B,R piecewise boundary describes a triangle when one
 // of its four turns is collinear (or two extrema coincide at one vertex).
 // Evaluate four cross products serially: no additional profile/frame RAM.
 reg [1:0] triangle_turn;
 reg triangle_flat_turn;
 reg signed [12:0] ta_x,ta_y,tb_x,tb_y;
 reg signed [25:0] triangle_product_a,triangle_product_b;
 wire signed [26:0] triangle_cross=$signed(triangle_product_a)-$signed(triangle_product_b);
 wire [26:0] triangle_abs_cross=triangle_cross[26]?-triangle_cross:triangle_cross;
 wire [26:0] triangle_cross_limit=({15'd0,fw}+{15'd0,fh})<<2;
 wire oriented_triangle= fw>=24 && fh>=24 && triangle_flat_turn &&
                       !rotation_discontinuous && rotated_rows*100>=(fh+1)*95;
 wire qualify_we=state==FINISH && !frame_fault && !overflow && !malformed && n_candidates!=0;
 wire [9:0] classified_fit=(classified==1)?(axis_rect?sr:rotated_rows):(classified==2)?sc:oriented_triangle?rotated_rows:(su>sd)?su:sd;
 wire [11:0] classified_height=fh+12'd1;
 wire stat_we=(state==INIT && init_index<=MAX_COMPONENTS) || state==EVAL_CAND || qualify_we;
 wire [SW-1:0] stat_wa=(state==INIT)?(init_index-1'b1):qualify_we?publish_slot[SW-1:0]:ci[SW-1:0];
 wire [SW-1:0] stat_ra=(state==RANK_LOAD || state==RANK_CALC || state==RANK_USE)?rank_index[SW-1:0]:
                         (state==FINISH_LOAD || state==ROT_PREP || state==ROT_MUL || state==TRI_PREP || state==TRI_MUL || state==TRI_EVAL || state==FINISH)?publish_slot[SW-1:0]:ci[SW-1:0];
 wire [71:0] stat_wd=(state==INIT)?72'h140000:qualify_we?{classified,classified_fit,so,classified_height,ma,18'd0}:next_statistics;
 always @(posedge clk)begin
   slot_q<=root_slot[slot_ra];if(slot_we)root_slot[slot_wa]<=slot_wd;
   packed_meta_q<=metadata[meta_ra];if(meta_we)metadata[meta_wa]<=meta_wd;
   stat_q<=statistics[stat_ra];if(stat_we)statistics[stat_wa]<=stat_wd;
 end
 wire [1:0] score_shape=stat_q[71:70];
 wire [9:0] score_fit=stat_q[69:60],score_rows=stat_q[59:50];
 wire [11:0] score_height=stat_q[49:38];
 wire [19:0] score_count=stat_q[37:18];
 // Compare geometry-band support/height without division. Occupied-row
 // coverage breaks ties before edge count. Products get a separate stage.
 wire rank_better=score_shape!=0 && !rank_selected[rank_index] &&
                  (rank_best==MAX_COMPONENTS || rank_fit_left>rank_fit_right ||
                   (rank_fit_left==rank_fit_right && (rank_rows_left>rank_rows_right ||
                    (rank_rows_left==rank_rows_right && score_count>rank_best_count))));
 wire [RW-1:0] rank_pick=rank_better?rank_index:rank_best;
 reg duplicate_box;
 integer kd;
 always @* begin
   duplicate_box=0;
   // Edge-only input cannot disambiguate close nested objects and thick
   // strokes. Collapse only an identical class and all four box coordinates.
   for(kd=0;kd<MAX_OBJECTS;kd=kd+1)
     if(kd<candidate_count && shapes[kd*2+:2]==chosen_shape &&
        boxes_x0[kd*12+:12]==fm[95:84] && boxes_x1[kd*12+:12]==fm[83:72] &&
        boxes_y0[kd*12+:12]==fm[71:60] && boxes_y1[kd*12+:12]==fm[59:48])duplicate_box=1;
 end
 always @* begin
   classified=0;
   // Below 12 pixels on both axes the median/Sobel profiles of a circle
   // and a square can be identical. Do not assign an arbitrary class.
   if(fr!=0 && (fw>=12 || fh>=12) && so*10>=(fh+1)*9)begin
     if(axis_rect)classified=1;
     else if(legacy_density && fw>=12 && fh>=12 && sc*100>=so*85 && fw*8>=fh*7 && fh*8>=fw*7)classified=2;
     else if(((su*100>=so*85 && top_small && bottom_strong) ||(sd*100>=so*85 && bottom_small && top_strong)))classified=3;
     else if(rotated_rect)classified=1;
     else if(oriented_triangle)classified=3;
   end
 end
 // Reason bits: labels, row banks, union events, final components,
 // analysis deadline, malformed frame, input overrun, reserved.
 reg [7:0] diagnostic_reasons;
 reg [4:0] diagnostic_bank_peak,diagnostic_classified;
 reg [15:0] diagnostic_accepted,diagnostic_completed;
 wire [15:0] completed_including_result=(diagnostic_completed==16'hffff)?16'hffff:diagnostic_completed+16'd1;
 assign frame_accepted=sink_valid && sink_sop && accept_input;
 wire [4:0] diagnostic_candidates=n_candidates,diagnostic_components=(component_count>31)?5'd31:component_count;
 wire [11:0] diagnostic_events=event_count;
 wire [7:0] diagnostic_labels=allocated;
 assign diagnostic_data={30'd0,skipped_frames,completed_including_result,
   diagnostic_accepted,analysis_ticks,diagnostic_classified,
   diagnostic_candidates,diagnostic_components,diagnostic_events,
   diagnostic_bank_peak,diagnostic_labels,diagnostic_reasons};
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n)begin diagnostic_accepted<=0;diagnostic_completed<=0;diagnostic_bank_peak<=0;diagnostic_classified<=0;end
   else begin
     if(frame_accepted && diagnostic_accepted!=16'hffff)diagnostic_accepted<=diagnostic_accepted+1'b1;
     if(result_valid && diagnostic_completed!=16'hffff)diagnostic_completed<=diagnostic_completed+1'b1;
     if(dv && ds)begin diagnostic_bank_peak<=hit?1:0;diagnostic_classified<=0;end
     else begin
       if(dv && state==COLLECT && chosen_bank>diagnostic_bank_peak)diagnostic_bank_peak<=chosen_bank;
       if(state==FINISH && !frame_fault && !overflow && !malformed && n_candidates!=0 && classified!=0)
         diagnostic_classified<=diagnostic_classified+1'b1;
     end
   end
 end
 assign busy=(state!=COLLECT);
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n)begin
     diagnostic_reasons<=0;state<=COLLECT;frame_active<=0;frame_fault<=0;allocated<=0;west<=0;northwest<=0;
     event_count<=0;event_index<=0;init_index<=1;flat_index<=1;node<=1;base_id<=0;pair_id<=0;root_a<=0;root_b<=0;pair_index<=0;analysis_ticks<=0;
     result_valid<=0;candidate_count<=0;shapes<=0;boxes_x0<=0;boxes_x1<=0;boxes_y0<=0;boxes_y1<=0;edge_counts<=0;
     overrun<=0;overflow<=0;malformed<=0;scan_y<=0;bank_index<=1;component_count<=0;ci<=0;n_candidates<=0;select_slot<=0;select_index<=1;select_best_root<=0;select_best_count<=0;publish_slot<=0;
     triangle_turn<=0;triangle_flat_turn<=0;ta_x<=0;ta_y<=0;tb_x<=0;tb_y<=0;triangle_product_a<=0;triangle_product_b<=0;
     rotation_tol<=0;step_l<=0;step_r<=0;step_product_l<=0;step_product_r<=0;check_step_l<=0;check_step_r<=0;
     rldx<=0;rldy<=0;rlpx<=0;rlpy<=0;rrdx<=0;rrdy<=0;rrpx<=0;rrpy<=0;
     rlproduct_a<=0;rlproduct_b<=0;rrproduct_a<=0;rrproduct_b<=0;rllimit<=0;rrlimit<=0;
     corner_lx<=0;corner_rx<=0;corner_ly<=0;corner_ry<=0;corner_dot_x<=0;corner_dot_y<=0;box_square<=0;rot_corner_ok<=0;
     sw<=0;sh<=0;tol<=0;radius_squared<=0;lx_delta<=0;rx_delta<=0;yy_delta<=0;l_squared<=0;r_squared<=0;y_squared<=0;up_error<=0;down_error<=0;center_error<=0;
     for(i=1;i<=PROFILE_BANKS;i=i+1)begin row_tag[i]<=0;row_count[i]<=0;row_min[i]<=0;row_max[i]<=0;end
     metadata_slot_reg<=1;
     rank_index<=0;rank_best<=MAX_COMPONENTS;rank_selected<=0;rank_best_shape<=0;chosen_shape<=0;
     rank_best_fit<=0;rank_best_rows<=0;rank_best_height<=0;rank_best_count<=0;
     rank_fit_left<=0;rank_fit_right<=0;rank_rows_left<=0;rank_rows_right<=0;
     for(i=1;i<=MAX_LABELS;i=i+1)selected[i]<=0;
     for(i=0;i<MAX_COMPONENTS;i=i+1)begin candidates[i]<=0;merged_count[i]<=0;merged_min[i]<={XW{1'b1}};merged_max[i]<=0;end
   end else begin
     result_valid<=0;
     if(state==COLLECT)analysis_ticks<=0;else if(analysis_ticks<ANALYSIS_LIMIT)analysis_ticks<=analysis_ticks+1'b1;
     if(dv && ds)begin
       diagnostic_reasons<=0;state<=COLLECT;frame_active<=1;frame_fault<=0;allocated<=hit?1:0;west<=hit?1:0;northwest<=0;
       overflow<=0;malformed<=0;component_count<=0;scan_y<=0;bank_index<=1;n_candidates<=0;publish_slot<=0;
       event_count<=0;event_index<=0;analysis_ticks<=0;
       if(state!=COLLECT)begin diagnostic_reasons[6]<=1;overrun<=1;candidate_count<=0;shapes<=0;boxes_x0<=0;boxes_x1<=0;boxes_y0<=0;boxes_y1<=0;edge_counts<=0;end
       if(state==COLLECT && frame_active)begin diagnostic_reasons[5]<=1;frame_fault<=1;malformed<=1;end
       for(i=1;i<=PROFILE_BANKS;i=i+1)begin row_tag[i]<=(hit && i==1)?1:0;row_count[i]<=(hit && i==1)?1:0;row_min[i]<=0;row_max[i]<=0;end
       for(i=1;i<=MAX_LABELS;i=i+1)selected[i]<=0;
       if(de || WIDTH*HEIGHT==1)begin diagnostic_reasons[5]<=1;frame_fault<=1;malformed<=1;state<=FINISH;end
     end else if(state!=COLLECT && state!=FINISH && analysis_ticks>=ANALYSIS_LIMIT-1)begin
       diagnostic_reasons[4]<=1;frame_fault<=1;overflow<=1;state<=FINISH;publish_slot<=0;
     end else if(dv && state==COLLECT)begin
       if(!frame_active)begin diagnostic_reasons[5]<=1;malformed<=1;frame_fault<=1;end
       if(allocate_pixel)begin if(allocated>=MAX_LABELS)begin diagnostic_reasons[0]<=1;overflow<=1;frame_fault<=1;end else allocated<=allocated+1'b1;end
       if(conflict && !frame_fault)begin
         if(event_count>=EVENT_DEPTH)begin diagnostic_reasons[2]<=1;overflow<=1;frame_fault<=1;end else event_count<=event_count+1'b1;
       end
       west<=(dx==WIDTH-1)?0:chosen_label;northwest<=(dx==WIDTH-1)?0:north_q;
       if(chosen_label!=0)begin
         if(chosen_bank==0)begin diagnostic_reasons[1]<=1;overflow<=1;frame_fault<=1;end
         else begin row_tag[chosen_bank]<=chosen_label;row_count[chosen_bank]<=row_count[chosen_bank]+1'b1;if(row_count[chosen_bank]==0)row_min[chosen_bank]<=dx[XW-1:0];row_max[chosen_bank]<=dx[XW-1:0];end
       end
       if(dx==WIDTH-1)for(i=1;i<=PROFILE_BANKS;i=i+1)begin row_tag[i]<=0;row_count[i]<=0;row_min[i]<=0;row_max[i]<=0;end
       if(de || (dx==WIDTH-1 && dy==HEIGHT-1))begin
         frame_active<=0;scan_y<=0;bank_index<=1;
         if(!de || dx!=WIDTH-1 || dy!=HEIGHT-1)diagnostic_reasons[5]<=1;
         if(!de || dx!=WIDTH-1 || dy!=HEIGHT-1 || frame_fault)begin frame_fault<=1;malformed<=(!de || dx!=WIDTH-1 || dy!=HEIGHT-1 || malformed);state<=FINISH;end else begin init_index<=1;state<=INIT;end
       end
     end else begin
       case(state)
         INIT:begin
           if(init_index==MAX_LABELS)begin event_index<=0;flat_index<=1;state<=(allocated==0)?READ_META:(event_count==0)?FLAT_START:EVENT_READ;end else init_index<=init_index+1'b1;
         end
         EVENT_READ:state<=EVENT_PREP;
         EVENT_PREP:begin base_id<=event_min;pair_index<=0;node<=event_min;state<=FIND_A_READ;end
         PAIR:begin
           if(event_neighbor==0 || event_neighbor==base_id || duplicate_neighbor)begin
             if(pair_index==3)begin if(event_index+1>=event_count)begin flat_index<=1;state<=FLAT_START;end else begin event_index<=event_index+1'b1;state<=EVENT_READ;end end else pair_index<=pair_index+1'b1;
           end else begin node<=event_neighbor;pair_id<=event_neighbor;state<=FIND_B_READ;end
         end
         FIND_A_READ:state<=FIND_A_USE;
         FIND_A_USE:begin
           if(parent_now==node)begin root_a<=node;state<=PAIR;end
           else begin node<=parent_now;state<=FIND_A_READ;end
         end
         FIND_B_READ:state<=FIND_B_USE;
         FIND_B_USE:begin
           if(parent_now==node)begin root_b<=node;state<=LINK_LOSER;end
           else begin node<=parent_now;state<=FIND_B_READ;end
         end
         LINK_LOSER:state<=LINK_WINNER;
         LINK_WINNER:begin
           root_a<=winner;
           if(pair_index==3)begin if(event_index+1>=event_count)begin flat_index<=1;state<=FLAT_START;end else begin event_index<=event_index+1'b1;state<=EVENT_READ;end end
           else begin pair_index<=pair_index+1'b1;state<=PAIR;end
         end
         FLAT_START:begin node<=flat_index;state<=FLAT_READ;end
         FLAT_READ:state<=FLAT_USE;
         FLAT_USE:begin if(parent_now==node)state<=FLAT_STORE;else begin node<=parent_now;state<=FLAT_READ;end end
         FLAT_STORE:begin if(flat_index==allocated)begin scan_y<=0;state<=READ_META;end else begin flat_index<=flat_index+1'b1;state<=FLAT_START;end end
         READ_META:begin bank_index<=1;state<=MERGE_META;end
         MERGE_META:state<=PARENT_META;
         PARENT_META:state<=META_SLOT;
         META_SLOT:begin
           if(profile_count!=0 && profile_root!=0)begin
             if(existing_slot==0 && component_count==MAX_LABELS)begin diagnostic_reasons[3]<=1;overflow<=1;frame_fault<=1;state<=FINISH;end
             else begin
               if(existing_slot==0)component_count<=component_count+1'b1;
               metadata_slot_reg<=metadata_slot;state<=META_UPDATE;
             end
           end
           if(profile_count==0 || profile_root==0)begin
             if(bank_index==PROFILE_BANKS)begin if(scan_y==HEIGHT-1)begin select_slot<=0;select_index<=1;select_best_root<=0;select_best_count<=0;state<=SELECT;end else begin scan_y<=scan_y+1'b1;state<=READ_META;end end
             else begin bank_index<=bank_index+1'b1;state<=MERGE_META;end
           end
         end
         META_UPDATE:begin
           if(bank_index==PROFILE_BANKS)begin if(scan_y==HEIGHT-1)begin select_slot<=0;select_index<=1;select_best_root<=0;select_best_count<=0;state<=SELECT;end else begin scan_y<=scan_y+1'b1;state<=READ_META;end end
           else begin bank_index<=bank_index+1'b1;state<=MERGE_META;end
         end
         SELECT:state<=SELECT_USE;
         SELECT_USE:begin
           if(better_root)begin select_best_root<=select_index;select_best_count<=ma;end
           if(select_index==MAX_LABELS)begin
             if(pick_root!=0)begin candidates[select_slot]<=pick_root;selected[pick_root]<=1;n_candidates<=n_candidates+1'b1;end
             if(pick_root==0 || select_slot==MAX_COMPONENTS-1)begin
               scan_y<=0;ci<=0;for(i=0;i<MAX_COMPONENTS;i=i+1)begin merged_count[i]<=0;merged_min[i]<={XW{1'b1}};merged_max[i]<=0;end
               state<=(pick_root==0 && n_candidates==0)?FINISH:READ_ROW;
             end else begin select_slot<=select_slot+1'b1;select_index<=1;select_best_root<=0;select_best_count<=0;state<=SELECT;end
           end else begin select_index<=select_index+1'b1;state<=SELECT;end
         end
         READ_ROW:begin bank_index<=1;state<=MERGE_ROW;end
         MERGE_ROW:state<=PARENT_ROW;
         PARENT_ROW:state<=ROW_SLOT;
         ROW_SLOT:begin
           if(merge_candidate<MAX_COMPONENTS && profile_count!=0)begin merged_count[merge_candidate]<=merged_count[merge_candidate]+profile_count;if(profile_min<merged_min[merge_candidate])merged_min[merge_candidate]<=profile_min;if(profile_max>merged_max[merge_candidate])merged_max[merge_candidate]<=profile_max;end
           if(bank_index==PROFILE_BANKS)begin ci<=0;state<=LOAD_CAND;end else begin bank_index<=bank_index+1'b1;state<=MERGE_ROW;end
         end
         LOAD_CAND:state<=FETCH_CAND;
         FETCH_CAND:begin
           rotation_tol<=((cx1-cx0)>=80 && (cy1-cy0)>=80)?5:(((cx1-cx0)<(cy1-cy0))?((cx1-cx0)>>4):((cy1-cy0)>>4));
           step_l<=$signed({1'b0,merged_min[ci]})-$signed({1'b0,previous_extrema_q[2*XW-1:XW]});
           step_r<=$signed({1'b0,merged_max[ci]})-$signed({1'b0,previous_extrema_q[XW-1:0]});
           check_step_l<=scan_y>cy0+3 && scan_y+3<cy1 && (scan_y+3<left_y || scan_y>left_y+3);
           check_step_r<=scan_y>cy0+3 && scan_y+3<cy1 && (scan_y+3<right_y || scan_y>right_y+3);
           rldx<=$signed({1'b0,lbx})-$signed({1'b0,lax});rldy<=$signed({1'b0,lby})-$signed({1'b0,lay});
           rlpx<=$signed({1'b0,merged_min[ci]})-$signed({1'b0,lax});rlpy<=$signed({1'b0,scan_y})-$signed({1'b0,lay});
           rrdx<=$signed({1'b0,rbx})-$signed({1'b0,rax});rrdy<=$signed({1'b0,rby})-$signed({1'b0,ray});
           rrpx<=$signed({1'b0,merged_max[ci]})-$signed({1'b0,rax});rrpy<=$signed({1'b0,scan_y})-$signed({1'b0,ray});
           sw<=cx1-cx0;sh<=cy1-cy0;tol<=((cx1-cx0)>>4)+1'b1;
           lx_delta<=$signed({1'b0,merged_min[ci],1'b0})-$signed({2'b0,cx0})-$signed({2'b0,cx1});rx_delta<=$signed({1'b0,merged_max[ci],1'b0})-$signed({2'b0,cx0})-$signed({2'b0,cx1});yy_delta<=$signed({1'b0,scan_y,1'b0})-$signed({2'b0,cy0})-$signed({2'b0,cy1});state<=CALC_CAND;
         end
         CALC_CAND:begin
           rlproduct_a<=rlpx*rldy;rlproduct_b<=rldx*rlpy;rrproduct_a<=rrpx*rrdy;rrproduct_b<=rrdx*rrpy;
           rllimit<=rotation_tol*(rladx+rldy);rrlimit<=rotation_tol*(rradx+rrdy);
           step_product_l<=step_l*rldy;step_product_r<=step_r*rrdy;
           radius_squared<=mean_span*mean_span;l_squared<=lx_delta*lx_delta;r_squared<=rx_delta*rx_delta;y_squared<=yy_delta*yy_delta;
           up_error<=$signed({1'b0,merged_max[ci]-merged_min[ci]})*$signed({1'b0,sh})-$signed({1'b0,sw})*$signed({1'b0,scan_y-cy0});down_error<=$signed({1'b0,merged_max[ci]-merged_min[ci]})*$signed({1'b0,sh})-$signed({1'b0,sw})*$signed({1'b0,cy1-scan_y});
           center_error<=$signed({1'b0,merged_min[ci]})*$signed({1'b0,sh})+$signed({1'b0,merged_max[ci]})*$signed({1'b0,sh})-($signed({1'b0,top_center})*$signed({1'b0,cy1-scan_y})+$signed({1'b0,bottom_center})*$signed({1'b0,scan_y-cy0}));state<=EVAL_CAND;
         end
         EVAL_CAND:begin
           if(ci+1<n_candidates)begin ci<=ci+1'b1;state<=LOAD_CAND;end
           else if(scan_y==HEIGHT-1)begin publish_slot<=0;candidate_count<=0;shapes<=0;boxes_x0<=0;boxes_x1<=0;boxes_y0<=0;boxes_y1<=0;edge_counts<=0;state<=FINISH_LOAD;end
           else begin scan_y<=scan_y+1'b1;for(i=0;i<MAX_COMPONENTS;i=i+1)begin merged_count[i]<=0;merged_min[i]<={XW{1'b1}};merged_max[i]<=0;end state<=READ_ROW;end
         end
         FINISH_LOAD:state<=ROT_PREP;
         ROT_PREP:begin
           corner_lx<=$signed({1'b0,fm[95:84]})-$signed(finish_tx);corner_rx<=$signed({1'b0,fm[83:72]})-$signed(finish_tx);
           corner_ly<=$signed({1'b0,left_y})-$signed({1'b0,fm[71:60]});corner_ry<=$signed({1'b0,right_y})-$signed({1'b0,fm[71:60]});
           rot_corner_ok<=fw>=24 && fh>=24 && center_x_error>=-3 && center_x_error<=3 && center_y_error>=-3 && center_y_error<=3 &&
             left_y>=fm[71:60]+2 && right_y>=fm[71:60]+2 && left_y+2<=fm[59:48] && right_y+2<=fm[59:48];
           state<=ROT_MUL;
         end
         ROT_MUL:begin corner_dot_x<=corner_lx*corner_rx;corner_dot_y<=corner_ly*corner_ry;box_square<=fw*fw+fh*fh;triangle_turn<=0;triangle_flat_turn<=0;state<=TRI_PREP;end
         TRI_PREP:begin
           // Cross at T, B, L, R; vectors point to their two neighbors.
           case(triangle_turn)
             0:begin ta_x<=corner_lx;ta_y<=corner_ly;tb_x<=corner_rx;tb_y<=corner_ry;end
             1:begin
               ta_x<=$signed({1'b0,fm[95:84]})-$signed(finish_bx);ta_y<=$signed({1'b0,left_y})-$signed({1'b0,fm[59:48]});
               tb_x<=$signed({1'b0,fm[83:72]})-$signed(finish_bx);tb_y<=$signed({1'b0,right_y})-$signed({1'b0,fm[59:48]});
             end
             2:begin
               ta_x<=$signed(finish_tx)-$signed({1'b0,fm[95:84]});ta_y<=$signed({1'b0,fm[71:60]})-$signed({1'b0,left_y});
               tb_x<=$signed(finish_bx)-$signed({1'b0,fm[95:84]});tb_y<=$signed({1'b0,fm[59:48]})-$signed({1'b0,left_y});
             end
             3:begin
               ta_x<=$signed(finish_tx)-$signed({1'b0,fm[83:72]});ta_y<=$signed({1'b0,fm[71:60]})-$signed({1'b0,right_y});
               tb_x<=$signed(finish_bx)-$signed({1'b0,fm[83:72]});tb_y<=$signed({1'b0,fm[59:48]})-$signed({1'b0,right_y});
             end
           endcase
           state<=TRI_MUL;
         end
         TRI_MUL:begin triangle_product_a<=ta_x*tb_y;triangle_product_b<=ta_y*tb_x;state<=TRI_EVAL;end
         TRI_EVAL:begin
           if(triangle_abs_cross<=triangle_cross_limit)triangle_flat_turn<=1;
           if(triangle_turn==3)state<=FINISH;
           else begin triangle_turn<=triangle_turn+1'b1;state<=TRI_PREP;end
         end
         FINISH:begin
           if(frame_fault || overflow || malformed || n_candidates==0)begin candidate_count<=0;shapes<=0;boxes_x0<=0;boxes_x1<=0;boxes_y0<=0;boxes_y1<=0;edge_counts<=0;result_valid<=1;state<=COLLECT;if(!frame_fault && !overflow && !malformed)overrun<=0;end
           else begin
             if(publish_slot+1>=n_candidates)begin rank_selected<=0;rank_index<=0;rank_best<=MAX_COMPONENTS;state<=RANK_LOAD;end
             else begin publish_slot<=publish_slot+1'b1;state<=FINISH_LOAD;end
           end
         end
         RANK_LOAD:state<=RANK_CALC;
         RANK_CALC:begin
           rank_fit_left<=score_fit*rank_best_height;rank_fit_right<=rank_best_fit*score_height;
           rank_rows_left<=score_rows*rank_best_height;rank_rows_right<=rank_best_rows*score_height;
           state<=RANK_USE;
         end
         RANK_USE:begin
           if(rank_better)begin rank_best<=rank_index;rank_best_shape<=score_shape;rank_best_fit<=score_fit;rank_best_rows<=score_rows;rank_best_height<=score_height;rank_best_count<=score_count;end
           if(rank_index+1>=n_candidates)begin
             if(rank_pick==MAX_COMPONENTS)begin result_valid<=1;overrun<=0;state<=COLLECT;end
             else begin rank_selected[rank_pick]<=1;publish_slot<=rank_pick;chosen_shape<=rank_better?score_shape:rank_best_shape;state<=PUBLISH_LOAD;end
           end else begin rank_index<=rank_index+1'b1;state<=RANK_LOAD;end
         end
         PUBLISH_LOAD:state<=PUBLISH_USE;
         PUBLISH_USE:begin
           if(!duplicate_box)begin
             for(i=0;i<MAX_OBJECTS;i=i+1)if(candidate_count==i)begin shapes[i*2+:2]<=chosen_shape;boxes_x0[i*12+:12]<=fm[95:84];boxes_x1[i*12+:12]<=fm[83:72];boxes_y0[i*12+:12]<=fm[71:60];boxes_y1[i*12+:12]<=fm[59:48];edge_counts[i*32+:32]<={12'd0,fm[115:96]};end
             candidate_count<=candidate_count+1'b1;
           end
           if(!duplicate_box && candidate_count+1>=MAX_OBJECTS)begin result_valid<=1;overrun<=0;state<=COLLECT;end
           else begin rank_index<=0;rank_best<=MAX_COMPONENTS;state<=RANK_LOAD;end
         end
       endcase
     end
   end
 end
endmodule
