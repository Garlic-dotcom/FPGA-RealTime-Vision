// This is the FRONT END of Canny, not complete Canny: global hysteresis must
// consume these labels. 00=background, 01=weak candidate, 10=strong seed.
module canny_classify_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input contour_mode,
 input [7:0] sink_data,
 input [10:0] low_threshold,high_threshold,
 output reg source_valid,source_sop,source_eop,
 output reg [1:0] source_class
);
 wire pv,ps,pe,gv,gs,ge,dv,ds,de,nv,ns,ne;
 wire contours=contour_mode===1'b1;
 wire [7:0] prefiltered,gaussian;
 wire [12:0] gradient;
 wire [10:0] nms;
 reg [10:0] active_low,active_high;
 wire [10:0] high_now=ns?high_threshold:active_high;
 wire [10:0] low_now=ns?((low_threshold>high_threshold)?high_threshold:low_threshold):active_low;
 impulse_clip3x3_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) prefilter(
 .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_data),
 .source_valid(pv),.source_sop(ps),.source_eop(pe),.source_data(prefiltered));
 gaussian3x3_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) smooth(
 .clk(clk),.rst_n(rst_n),.sink_valid(contours?sink_valid:pv),.sink_sop(contours?sink_sop:ps),.sink_eop(contours?sink_eop:pe),.sink_data(contours?sink_data:prefiltered),
 .source_valid(gv),.source_sop(gs),.source_eop(ge),.source_data(gaussian));
 canny_gradient_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) differentiate(
 .clk(clk),.rst_n(rst_n),.sink_valid(gv),.sink_sop(gs),.sink_eop(ge),.sink_data(gaussian),
 .source_valid(dv),.source_sop(ds),.source_eop(de),.source_data(gradient));
 canny_nms_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) suppress(
 .clk(clk),.rst_n(rst_n),.sink_valid(dv),.sink_sop(ds),.sink_eop(de),.sink_data(gradient),
 .source_valid(nv),.source_sop(ns),.source_eop(ne),.source_magnitude(nms));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin source_valid<=0;source_sop<=0;source_eop<=0;source_class<=0;active_low<=0;active_high<=0;end
   else begin
     source_valid<=contours?dv:nv;source_sop<=contours?ds:ns;source_eop<=contours?de:ne;
     if(nv && ns) begin active_high<=high_threshold;active_low<=low_now;end
     source_class<=contours?((gradient[12:2]>high_threshold)?2'd2:2'd0):(nms==0)?2'd0:(nms>=high_now)?2'd2:(nms>=low_now)?2'd1:2'd0;
   end
 end
endmodule
