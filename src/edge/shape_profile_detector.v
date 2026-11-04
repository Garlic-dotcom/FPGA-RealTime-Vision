// Single-object geometric classification inside a configured ROI. Input is a
// binary EDGE stream, not a foreground mask. Per-row extrema form a contour
// profile; no CPU is used. Supported labels: 0=unknown/none, 1=axis-aligned
// rectangle, 2=circle. Multiple overlapping objects are not segmented here.
// Caller must allow at least 3*HEIGHT+8 idle clocks after input EOP. A new SOP
// during analysis aborts that analysis and sets overrun until a complete new
// analysis succeeds. Labels stay stable between result_valid pulses.
module shape_profile_detector #(
 parameter WIDTH=1280,HEIGHT=720,
 parameter ROI_X0=WIDTH/8,ROI_X1=WIDTH*7/8,ROI_Y0=HEIGHT/8,ROI_Y1=HEIGHT*7/8
)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,sink_edge,
 output reg result_valid,
 output reg [1:0] shape,
 output reg [11:0] box_x0,box_x1,box_y0,box_y1,
 output reg [31:0] edge_count,
 output busy,
 output reg overrun
);
 localparam COLLECT=0,PREPARE=1,FETCH=2,CALCULATE=3,EVALUATE=4,FINISH=5,RADIUS=6;
 reg [2:0] state;
 reg [11:0] x,y,row_min,row_max,min_x,max_x,min_y,max_y;
 reg [12:0] row_count;
 reg [31:0] count;
 reg [36:0] rows[0:HEIGHT-1]; // {count[12:0],min_x[11:0],max_x[11:0]}
 reg [36:0] row_q;
 reg [11:0] scan_y;
 wire [11:0] cx=sink_sop?12'd0:x,cy=sink_sop?12'd0:y;
 wire hit=sink_edge && cx>=ROI_X0 && cx<ROI_X1 && cy>=ROI_Y0 && cy<ROI_Y1;
 wire [12:0] next_row_count=((sink_sop || cx==0)?13'd0:row_count)+(hit?13'd1:13'd0);
 wire [11:0] next_row_min=hit?(((sink_sop || cx==0 || row_count==0) || cx<row_min)?cx:row_min):((sink_sop || cx==0)?12'd0:row_min);
 wire [11:0] next_row_max=hit?cx:((sink_sop || cx==0)?12'd0:row_max);
 reg [12:0] span_w,span_h,tolerance;
 reg [31:0] radius_squared;
 wire [12:0] radius=(span_w+span_h)>>1;
 wire signed [13:0] left_delta=$signed({1'b0,row_q[23:12],1'b0})-$signed({2'b0,min_x})-$signed({2'b0,max_x});
 wire signed [13:0] right_delta=$signed({1'b0,row_q[11:0],1'b0})-$signed({2'b0,min_x})-$signed({2'b0,max_x});
 wire signed [13:0] y_delta=$signed({1'b0,scan_y,1'b0})-$signed({2'b0,min_y})-$signed({2'b0,max_y});
 reg [27:0] left_squared,right_squared,y_squared;
 wire [31:0] r_left=left_squared+y_squared,r_right=right_squared+y_squared;
 wire left_circle=(r_left*4>=radius_squared*3 && r_left*4<=radius_squared*5);
 wire right_circle=(r_right*4>=radius_squared*3 && r_right*4<=radius_squared*5);
 reg [12:0] occupied_rows,rectangle_rows,circle_rows;
 reg [12:0] top_width,bottom_width;
 wire adequate=span_w>=12 && span_h>=12 && count>=((span_w+span_h)>>1) &&
               count<=(span_w+span_h)*8 && occupied_rows*10>=(span_h+1)*9;
 wire rectangular=rectangle_rows*10>=occupied_rows*9 && top_width*2>=span_w && bottom_width*2>=span_w;
 wire circular=circle_rows*100>=occupied_rows*85 && span_w*8>=span_h*7 && span_h*8>=span_w*7;
 assign busy=(state!=COLLECT);
 always @(posedge clk) begin
   if(sink_valid && (state==COLLECT || sink_sop) && cx==WIDTH-1)
     rows[cy]<={next_row_count,next_row_min,next_row_max};
   if(state==FETCH) row_q<=rows[scan_y];
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=COLLECT;x<=0;y<=0;row_min<=0;row_max<=0;row_count<=0;
     min_x<=0;max_x<=0;min_y<=0;max_y<=0;count<=0;scan_y<=0;
     span_w<=0;span_h<=0;tolerance<=0;radius_squared<=0;
     occupied_rows<=0;rectangle_rows<=0;circle_rows<=0;top_width<=0;bottom_width<=0;
     left_squared<=0;right_squared<=0;y_squared<=0;
     result_valid<=0;shape<=0;box_x0<=0;box_x1<=0;box_y0<=0;box_y1<=0;edge_count<=0;overrun<=0;
   end else begin
     result_valid<=0;
     if(sink_valid && sink_sop && state!=COLLECT) begin state<=COLLECT;overrun<=1;end
     if(sink_valid && (state==COLLECT || sink_sop)) begin
       if(cx==WIDTH-1) begin x<=0;y<=(cy==HEIGHT-1)?12'd0:cy+1'b1;end
       else begin x<=cx+1'b1;y<=cy;end
       row_min<=next_row_min;row_max<=next_row_max;row_count<=next_row_count;
       if(sink_sop) begin
         count<=hit?32'd1:32'd0;min_x<=hit?cx:12'd4095;max_x<=hit?cx:12'd0;
         min_y<=hit?cy:12'd4095;max_y<=hit?cy:12'd0;
       end else if(hit) begin
         count<=count+1'b1;
         if(cx<min_x) min_x<=cx;if(cx>max_x) max_x<=cx;
         if(cy<min_y) min_y<=cy;if(cy>max_y) max_y<=cy;
       end
       if(sink_eop) state<=PREPARE;
     end else begin
       case(state)
         PREPARE:begin
           span_w<=max_x-min_x;span_h<=max_y-min_y;
           tolerance<=((max_x-min_x)>>4)+1'b1;
           occupied_rows<=0;rectangle_rows<=0;circle_rows<=0;top_width<=0;bottom_width<=0;scan_y<=0;
           state<=(count==0)?FINISH:RADIUS;
         end
         RADIUS:begin radius_squared<=radius*radius;state<=FETCH;end
         FETCH:state<=CALCULATE;
         CALCULATE:begin
           left_squared<=left_delta*left_delta;right_squared<=right_delta*right_delta;y_squared<=y_delta*y_delta;
           state<=EVALUATE;
         end
         EVALUATE:begin
           if(scan_y>=min_y && scan_y<=max_y && row_q[36:24]!=0) begin
             occupied_rows<=occupied_rows+1'b1;
             if(row_q[23:12]<=min_x+tolerance && row_q[11:0]+tolerance>=max_x) rectangle_rows<=rectangle_rows+1'b1;
             if(left_circle && right_circle) circle_rows<=circle_rows+1'b1;
             if(scan_y<=min_y+2 && row_q[36:24]>top_width) top_width<=row_q[36:24];
             if(scan_y+2>=max_y && row_q[36:24]>bottom_width) bottom_width<=row_q[36:24];
           end
           if(scan_y==HEIGHT-1) state<=FINISH;
           else begin scan_y<=scan_y+1'b1;state<=FETCH;end
         end
         FINISH:begin
           overrun<=0;
           result_valid<=1;shape<=(count!=0 && adequate)?(rectangular?2'd1:circular?2'd2:2'd0):2'd0;
           box_x0<=(count==0)?12'd0:min_x;box_x1<=(count==0)?12'd0:max_x;
           box_y0<=(count==0)?12'd0:min_y;box_y1<=(count==0)?12'd0:max_y;
           edge_count<=count;state<=COLLECT;
         end
       endcase
     end
   end
 end
endmodule
