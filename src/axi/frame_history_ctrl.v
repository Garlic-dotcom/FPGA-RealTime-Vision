// 32 x 4 MiB physical slots; publish only fully acknowledged frames.
// The directory is scanned sequentially in vertical blank, never a large
// combinational priority tree. While browsing, capture is stopped at EOF.
module frame_history_ctrl #(
 parameter SLOT_BITS=5,PLAY_TICKS=8,PROCESS_PIN_ENABLE=0
)(input clk,rst,input commit,view_tick,capture_quiet,
 input [1:0] requested_mode,input older,newer,
 output stop_capture,output reg [SLOT_BITS-1:0] write_slot,read_a,read_b,
 output reg compare_frame,read_ready,output reg [1:0] active_mode,
 output [5:0] history_count,output [5:0] history_age,
 input process_request,process_release,
 output reg process_valid,
 output reg [SLOT_BITS-1:0] process_a,process_b,
 output reg process_compare,
 output reg [15:0] read_generation_a,read_generation_b,
 output reg [15:0] process_generation_a,process_generation_b);
 localparam SLOTS=1<<SLOT_BITS;
 reg [SLOTS-1:0] valid;
 reg [31:0] generation[0:SLOTS-1];
 reg [31:0] serial,cursor_gen;
 reg [SLOT_BITS-1:0] latest,cursor,scan;
 reg [5:0] count;
 reg pending_old,pending_new,scanning,finalize;
 reg [1:0] scan_mode;
 reg seek_old,seek_new,found;
 reg [31:0] best_gen,min_gen,max_gen;
 reg [SLOT_BITS-1:0] best_slot,min_slot,max_slot;
 reg [15:0] play_count;
 reg [SLOT_BITS-1:0] next_write;
 integer i,k;
 always @* begin
   next_write=write_slot+1'b1;
   for(k=0;k<4;k=k+1)
     if(next_write==read_a || next_write==read_b ||
        (PROCESS_PIN_ENABLE && process_valid && (next_write==process_a || next_write==process_b)))
       next_write=next_write+1'b1;
 end
 assign stop_capture=requested_mode!=0;
 assign history_count=count;
 wire [31:0] age=serial-cursor_gen;
 assign history_age=(active_mode==0)?6'd0:age[5:0];
 always @(posedge clk) begin
   if(rst) begin
     valid<=0;serial<=0;latest<=0;write_slot<=0;read_a<=2;read_b<=2;
     process_valid<=0;process_a<=0;process_b<=0;process_compare<=0;
     read_generation_a<=0;read_generation_b<=0;process_generation_a<=0;process_generation_b<=0;
     cursor<=0;cursor_gen<=0;count<=0;compare_frame<=0;read_ready<=0;active_mode<=0;
     pending_old<=0;pending_new<=0;scanning<=0;finalize<=0;scan<=0;scan_mode<=0;
     seek_old<=0;seek_new<=0;found<=0;best_gen<=0;best_slot<=0;
     min_gen<=32'hffffffff;max_gen<=0;min_slot<=0;max_slot<=0;play_count<=0;
     for(i=0;i<SLOTS;i=i+1) generation[i]<=0;
   end else begin
     if(PROCESS_PIN_ENABLE) begin
       if(process_release) process_valid<=0;
       // Read leases and a same-cycle commit cannot race a new reservation.
       if(process_request && !process_valid && !process_release && read_ready &&
          valid[read_a] && valid[read_b] && read_a!=write_slot && read_b!=write_slot &&
          !(commit && (read_a==next_write || read_b==next_write))) begin
         process_valid<=1;process_a<=read_a;process_b<=read_b;process_compare<=compare_frame;
         process_generation_a<=read_generation_a;process_generation_b<=read_generation_b;
       end
     end
     if(older) pending_old<=1;
     if(newer) pending_new<=1;
     if(commit) begin
       generation[write_slot]<=serial+1'b1;serial<=serial+1'b1;latest<=write_slot;
       valid[write_slot]<=1;valid[next_write]<=0;write_slot<=next_write;
       if(!valid[next_write]) count<=count+1'b1;
     end
     if(view_tick) begin
       read_ready<=0;compare_frame<=0;
       if(requested_mode==0 || !capture_quiet || count==0) begin
         // Same-cycle commit can be selected: its B response already completed.
         read_a<=commit?write_slot:latest;read_b<=commit?write_slot:latest;
         read_ready<=count!=0 || commit;active_mode<=0;scanning<=0;finalize<=0;
         cursor<=commit?write_slot:latest;cursor_gen<=commit?serial+1'b1:serial;
         read_generation_a<=commit?serial+1'b1:serial;read_generation_b<=commit?serial+1'b1:serial;
         pending_old<=0;pending_new<=0;play_count<=0;
       end else begin
         if(active_mode==0) begin cursor<=latest;cursor_gen<=serial;end
         scan<=0;scanning<=1;scan_mode<=requested_mode;found<=0;
         best_gen<=pending_old?0:32'hffffffff;min_gen<=32'hffffffff;max_gen<=0;
         seek_old<=pending_old || (requested_mode==3 && active_mode!=3 && (active_mode==0 || cursor_gen==serial));
         seek_new<=!pending_old && (pending_new || (requested_mode==2 && play_count==PLAY_TICKS-1));
         if(pending_old || pending_new || requested_mode!=2 || play_count==PLAY_TICKS-1) play_count<=0;
         else play_count<=play_count+1'b1;
         pending_old<=0;pending_new<=0;
       end
     end else if(scanning) begin
       if(valid[scan]) begin
         if(generation[scan]<min_gen) begin min_gen<=generation[scan];min_slot<=scan;end
         if(generation[scan]>max_gen) begin max_gen<=generation[scan];max_slot<=scan;end
         if((seek_old && generation[scan]<cursor_gen && (!found || generation[scan]>best_gen)) ||
            (seek_new && generation[scan]>cursor_gen && (!found || generation[scan]<best_gen))) begin
           found<=1;best_gen<=generation[scan];best_slot<=scan;
         end
       end
       if(scan==SLOTS-1) begin scanning<=0;finalize<=1;end
       else scan<=scan+1'b1;
     end else if(finalize) begin
       finalize<=0;
       // Separate finalize cycle includes the last directory entry.
       if(seek_old || seek_new) begin
         cursor<=found?best_slot:(seek_old?max_slot:min_slot);
         cursor_gen<=found?best_gen:(seek_old?max_gen:min_gen);
         read_b<=found?best_slot:(seek_old?max_slot:min_slot);
         read_a<=(scan_mode==3)?latest:(found?best_slot:(seek_old?max_slot:min_slot));
         read_generation_b<=found?best_gen:(seek_old?max_gen:min_gen);
         read_generation_a<=(scan_mode==3)?serial:(found?best_gen:(seek_old?max_gen:min_gen));
       end else begin
         read_b<=cursor;read_a<=(scan_mode==3)?latest:cursor;
         read_generation_b<=cursor_gen;read_generation_a<=(scan_mode==3)?serial:cursor_gen;
       end
       active_mode<=scan_mode;compare_frame<=scan_mode==3;read_ready<=1;
     end
   end
 end
endmodule
