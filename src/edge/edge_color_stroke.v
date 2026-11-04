// Display-only centered 3x3 color stroke. Original edge colors take priority.
// Codes: 0 original RGB; 1 cyan,2 green,3 yellow,4 red,5 magenta.
// Only new stroke pixels use maximum code (strength order for codes1..4).
module edge_color_stroke #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [2:0] sink_code,
 output source_valid,source_sop,source_eop,
 output [2:0] source_code,
 output [2:0] source_original_code
);
 wire [2:0] p0,p1,p2,p3,p4,p5,p6,p7,p8;
 edge_color_window3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
 .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_code),
 .matrix_valid(source_valid),.matrix_sop(source_sop),.matrix_eop(source_eop),
 .matrix_p11(p0),.matrix_p12(p1),.matrix_p13(p2),.matrix_p21(p3),.matrix_p22(p4),.matrix_p23(p5),.matrix_p31(p6),.matrix_p32(p7),.matrix_p33(p8));
 function [2:0] max2;input [2:0] a,b;begin max2=a>b?a:b;end endfunction
 wire [2:0] maximum=max2(max2(max2(p0,p1),max2(p2,p3)),max2(max2(p5,p6),max2(p7,p8)));
 assign source_code=p4!=0?p4:maximum;
 assign source_original_code=p4;
endmodule

// Centered 3x3 windows, raster order, WIDTH*HEIGHT results with replicated borders.
// Two read-before-write row banks. Leave at least WIDTH+3 idle clocks after EOP.
// WIDTH and HEIGHT must both be >=2. No input is accepted during tail flushing.
module edge_color_window3x3 #(parameter WIDTH=480, parameter HEIGHT=320)(
 input clk,rst_n,sink_sop,sink_eop,sink_valid,input [2:0] sink_data,
 output reg matrix_sop,matrix_eop,matrix_valid,
 output reg [2:0] matrix_p11,matrix_p12,matrix_p13,
 output reg [2:0] matrix_p21,matrix_p22,matrix_p23,
 output reg [2:0] matrix_p31,matrix_p32,matrix_p33
);
 localparam XW=$clog2(WIDTH), YW=$clog2(HEIGHT+2);
 reg [2:0] bank_a[0:WIDTH-1],bank_b[0:WIDTH-1];
 reg [XW-1:0] xcnt;
 reg [YW-1:0] ycnt;
 reg flushing;
 wire step=flushing || sink_valid;
 wire restart=sink_valid && sink_sop && !flushing;
 wire [XW-1:0] ix=restart ? {XW{1'b0}} : xcnt;
 wire [YW-1:0] iy=restart ? {YW{1'b0}} : ycnt;
 wire [2:0] pixel=flushing ? 3'd0 : sink_data;
 reg [2:0] a_read,b_read,pixel_read;
 reg [XW-1:0] rx;
 reg [YW-1:0] ry;
 reg read_valid;
 // RAM contents do not need reset: stale rows are masked by border logic.
 always @(posedge clk) if(step && rst_n) begin
   a_read<=bank_a[ix]; b_read<=bank_b[ix]; pixel_read<=pixel;
   if(iy[0]) bank_b[ix]<=pixel; else bank_a[ix]<=pixel;
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     xcnt<=0;ycnt<=0;flushing<=0;read_valid<=0;rx<=0;ry<=0;
   end else begin
     read_valid<=step;
     if(step) begin
       rx<=ix;ry<=iy;
       if(ix==WIDTH-1) begin xcnt<=0;ycnt<=iy+1'b1;end
       else begin xcnt<=ix+1'b1;ycnt<=iy;end
       if(!flushing && sink_eop) flushing<=1;
       if(flushing && iy==HEIGHT+1 && ix==0) flushing<=0;
     end
   end
 end
 wire [2:0] top_pixel=ry[0] ? b_read : a_read;
 wire [2:0] mid_pixel=ry[0] ? a_read : b_read;
 reg [2:0] t0,t1,m0,m1,b0,b1;
 reg [8:0] top_row,mid_row,bot_row;
 wire [XW-1:0] ox=(rx==0) ? WIDTH-1 : rx-1'b1;
 wire [YW-1:0] oy=(rx==0) ? ry-2'd2 : ry-1'b1;
 wire output_position=((ry==1 && rx!=0) || ry>=2) && oy<HEIGHT;
 always @* begin
   if(rx==0) begin
     // Previous raster row's last center: repeat its rightmost column.
     top_row={t0,t1,t1};mid_row={m0,m1,m1};bot_row={b0,b1,b1};
   end else if(rx==1) begin
     top_row={t1,t1,top_pixel};mid_row={m1,m1,mid_pixel};
     bot_row={b1,b1,pixel_read};
   end else begin
     top_row={t0,t1,top_pixel};mid_row={m0,m1,mid_pixel};
     bot_row={b0,b1,pixel_read};
   end
   if(oy==0) top_row=mid_row;
   if(oy==HEIGHT-1) bot_row=mid_row;
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     t0<=0;t1<=0;m0<=0;m1<=0;b0<=0;b1<=0;
     matrix_sop<=0;matrix_eop<=0;matrix_valid<=0;
     {matrix_p11,matrix_p12,matrix_p13}<=0;
     {matrix_p21,matrix_p22,matrix_p23}<=0;
     {matrix_p31,matrix_p32,matrix_p33}<=0;
   end else begin
     matrix_valid<=0;matrix_sop<=0;matrix_eop<=0;
     if(read_valid) begin
       t0<=t1;t1<=top_pixel;m0<=m1;m1<=mid_pixel;b0<=b1;b1<=pixel_read;
       if(output_position) begin
         {matrix_p11,matrix_p12,matrix_p13}<=top_row;
         {matrix_p21,matrix_p22,matrix_p23}<=mid_row;
         {matrix_p31,matrix_p32,matrix_p33}<=bot_row;
         matrix_valid<=1;
         matrix_sop<=(ox==0 && oy==0);
         matrix_eop<=(ox==WIDTH-1 && oy==HEIGHT-1);
       end
     end
   end
 end
endmodule

