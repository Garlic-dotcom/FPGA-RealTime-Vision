// Exact directional NMS with one stored row. While row y arrives, retain the
// comparison against row y-1; while y+1 arrives, finish the forward comparison.
// Same strict-forward / nonstrict-backward rule and replicated borders.
// One output per input pixel. As with image_window3x3, leave WIDTH+3 clocks
// after input EOP for the replicated bottom row. WIDTH,HEIGHT must be >=2.
module canny_nms_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [12:0] sink_data,
 output reg source_valid,source_sop,source_eop,
 output reg [10:0] source_magnitude,
 output reg [1:0] source_direction,
 output reg [10:0] source_center_magnitude
);
 localparam XW=$clog2(WIDTH),YW=$clog2(HEIGHT+2);
 // [13:3] magnitude, [2:1] direction, [0] backward (or full horizontal) pass.
 reg [13:0] row[0:WIDTH-1];
 reg [XW-1:0] xcnt,rx;
 reg [YW-1:0] ycnt,ry;
 reg flushing,read_valid,read_flush;
 reg [12:0] input_read;
 reg [13:0] previous_read;
 wire step=flushing || sink_valid;
 wire restart=sink_valid && sink_sop && !flushing;
 wire [XW-1:0] ix=restart ? {XW{1'b0}} : xcnt;
 wire [YW-1:0] iy=restart ? {YW{1'b0}} : ycnt;
 wire [12:0] current_read=read_flush ? previous_read[13:1] : input_read;
 reg [12:0] c0,c1;
 reg [13:0] p0,p1;
 reg [12:0] cl,cc,cr;
 reg [13:0] pl,pc,pr;
 wire [XW-1:0] cx=(rx==0)?WIDTH-1:rx-1'b1;
 wire [YW-1:0] cy=(rx==0)?ry-1'b1:ry;
 wire position_valid=!(ry==0 && rx==0);
 wire write_row=read_valid && position_valid && cy<HEIGHT;
 reg backward_ok,forward_ok;
 reg [10:0] backward;
 always @* begin
   if(rx==0) begin
     cl=c0;cc=c1;cr=c1;pl=p0;pc=p1;pr=p1;
   end else if(rx==1) begin
     cl=c1;cc=c1;cr=current_read;pl=p1;pc=p1;pr=previous_read;
   end else begin
     cl=c0;cc=c1;cr=current_read;pl=p0;pc=p1;pr=previous_read;
   end
   case(cc[1:0])
     0:backward=cl[12:2];
     1:backward=(cy==0)?cl[12:2]:pl[13:3];
     2:backward=(cy==0)?cc[12:2]:pc[13:3];
     3:backward=(cy==0)?cr[12:2]:pr[13:3];
   endcase
   backward_ok=(cc[12:2]>=backward) && (cc[1:0]!=0 || cc[12:2]>cr[12:2]);
   case(pc[2:1])
     0:forward_ok=1'b1;
     1:forward_ok=pc[13:3]>cr[12:2];
     2:forward_ok=pc[13:3]>cc[12:2];
     3:forward_ok=pc[13:3]>cl[12:2];
   endcase
 end
 // Read-before-write. The write is one centered pixel behind the read.
 // No RAM reset: the first row never consumes uninitialized previous data.
 always @(posedge clk) begin
   if(step && rst_n) begin
     // Only the minimum WIDTH=2 case can read a just-completed centered pixel
     // on the same clock as its write. Explicit forwarding defines that case.
     if(WIDTH==2 && write_row && ix==cx) previous_read<={cc,backward_ok};
     else previous_read<=row[ix];
     input_read<=sink_data;
   end
   if(write_row && rst_n) row[cx]<={cc,backward_ok};
 end
 reg pending_valid,pending_sop,pending_eop;
 reg [10:0] pending_magnitude;
 reg [1:0] pending_direction;
 reg [10:0] pending_center_magnitude;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     xcnt<=0;ycnt<=0;rx<=0;ry<=0;flushing<=0;read_valid<=0;read_flush<=0;
     c0<=0;c1<=0;p0<=0;p1<=0;
     pending_valid<=0;pending_sop<=0;pending_eop<=0;pending_magnitude<=0;
     source_valid<=0;source_sop<=0;source_eop<=0;source_magnitude<=0;
     pending_direction<=0;source_direction<=0;
     pending_center_magnitude<=0;source_center_magnitude<=0;
   end else begin
     read_valid<=step;
     if(step) begin
       rx<=ix;ry<=iy;read_flush<=flushing;
       if(ix==WIDTH-1) begin xcnt<=0;ycnt<=iy+1'b1;end
       else begin xcnt<=ix+1'b1;ycnt<=iy;end
       if(!flushing && sink_eop)flushing<=1;
       if(flushing && iy==HEIGHT+1 && ix==0)flushing<=0;
     end
     pending_valid<=0;pending_sop<=0;pending_eop<=0;
     if(read_valid) begin
       c0<=c1;c1<=current_read;p0<=p1;p1<=previous_read;
       if(position_valid && cy>=1 && cy<=HEIGHT) begin
         pending_valid<=1;pending_sop<=(cy==1 && cx==0);
         pending_eop<=(cy==HEIGHT && cx==WIDTH-1);
         pending_magnitude<=(pc[0] && forward_ok)?pc[13:3]:11'd0;
         pending_direction<=pc[2:1];
         pending_center_magnitude<=pc[13:3];
       end
     end
     // Match the original window register + NMS output register latency.
     source_valid<=pending_valid;source_sop<=pending_sop;source_eop<=pending_eop;
     source_magnitude<=pending_magnitude;
     source_direction<=pending_direction;
     source_center_magnitude<=pending_center_magnitude;
   end
 end
endmodule
