// All context enters with the grayscale stream, i.e. RGB565 is already delayed
// by the seven grayscale arithmetic clocks. Output is RGB565 for the DDR FIFO.
module edge_video_pipeline #(
 parameter WIDTH=1280,HEIGHT=720,MEDIAN_ENABLE=1,CONTEXT_DEPTH=4096,OSD_ENABLE=1,SHAPE_ENABLE=0,GAUSSIAN_ENABLE=1,SOBEL_NMS_ENABLE=1,SPLIT_BINARY_ENABLE=1,FILTER_PREVIEW_ENABLE=0,DIRECTION_COLOR_ENABLE=1,EXTERNAL_SHAPES=0,STRENGTH_COLOR_ENABLE=0,RGB_STROKE_ENABLE=0
)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [7:0] sink_gray,
 input [15:0] sink_rgb565,
 input [10:0] threshold,
 input [2:0] mode,
 input filter_gaussian,
 input color_direction,
 input [404:0] contour_shapes,
 output reg source_valid,source_sop,source_eop,
 output reg [15:0] source_rgb565,
 output reg [8:0] source_compare_data,
 output reg [2:0] source_mode,
 output reg [15:0] source_threshold_bcd
);
 wire mv,ms,me,ev,es,ee,edge_bit;
 wire [7:0] median_data,gaussian_data,center;
 reg [2:0] input_mode,filter_mode;
 reg input_gaussian,filter_gaussian_frame;
 reg input_color_direction,filter_color_direction;
 reg [404:0] input_shapes;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)input_shapes<=0;else if(sink_valid&&sink_sop)input_shapes<=contour_shapes;end
 wire current_gaussian=(mv && ms)?input_gaussian:filter_gaussian_frame;
 // Configuration follows the input frame, before either window starts.
 // The first filtered pixel uses input_mode while filter_mode is latched.
 wire [2:0] current_filter_mode=(mv && ms)?input_mode:filter_mode;
 wire gaussian_selected=GAUSSIAN_ENABLE && ((current_filter_mode>=3 && current_filter_mode<=6) || (SPLIT_BINARY_ENABLE && current_filter_mode==0) || (FILTER_PREVIEW_ENABLE && current_filter_mode==2 && current_gaussian));
 wire [7:0] filtered_data=gaussian_selected?gaussian_data:median_data;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n)begin input_mode<=0;filter_mode<=0;input_gaussian<=0;filter_gaussian_frame<=0;input_color_direction<=!STRENGTH_COLOR_ENABLE;filter_color_direction<=!STRENGTH_COLOR_ENABLE;end
   else begin
     if(sink_valid && sink_sop)begin input_mode<=mode;input_gaussian<=FILTER_PREVIEW_ENABLE && filter_gaussian;input_color_direction<=STRENGTH_COLOR_ENABLE?color_direction:1'b1;end
     if(mv && ms)begin filter_mode<=input_mode;filter_gaussian_frame<=input_gaussian;filter_color_direction<=input_color_direction;end
   end
 end
 wire [10:0] magnitude,active_threshold,compare_threshold;
 wire [1:0] direction;
 median3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.FILTER_ENABLE(MEDIAN_ENABLE)) median(
   .clk(clk),.rst_n(rst_n),.sink_valid(sink_valid),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_gray),
   .source_valid(mv),.source_sop(ms),.source_eop(me),.source_data(median_data),.source_gaussian(gaussian_data));
 sobel_l1_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) sobel(
   .clk(clk),.rst_n(rst_n),.sink_valid(mv),.sink_sop(ms),.sink_eop(me),.sink_data(filtered_data),
   .threshold(threshold),.source_valid(ev),.source_sop(es),.source_eop(ee),
   .source_edge(edge_bit),.source_magnitude(magnitude),.source_center(center),.active_threshold(active_threshold),
   .source_direction(direction),.source_compare_threshold(compare_threshold));
 // SPLIT, BINARY and RGB share the same edge policy; SPLIT changes layout only.
 // SPLIT_BINARY_ENABLE=0 retains the E1 legacy comparison path.
 // MAG and the recognition tap retain the original Sobel samples and thresholds.
 // Supported raster blanking exceeds the one-line NMS flush before the
 // next input frame can update filter_mode or active_threshold.
 wire nv,ns,ne;
 wire [10:0] thin_magnitude;
 wire [10:0] thin_center_magnitude;
 wire [1:0] thin_direction;
 generate if(SOBEL_NMS_ENABLE) begin: g_thin
   canny_nms_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) thin(
     .clk(clk),.rst_n(rst_n),.sink_valid(ev),.sink_sop(es),.sink_eop(ee),.sink_data({magnitude,direction}),
     .source_valid(nv),.source_sop(ns),.source_eop(ne),.source_magnitude(thin_magnitude),.source_direction(thin_direction),.source_center_magnitude(thin_center_magnitude));
 end else begin: g_no_thin
   assign nv=ev;assign ns=es;assign ne=ee;assign thin_magnitude=magnitude;assign thin_direction=direction;assign thin_center_magnitude=magnitude;
 end endgenerate
 wire thin_selected=SOBEL_NMS_ENABLE && (filter_mode==3'd4 || filter_mode==3'd5 || (SPLIT_BINARY_ENABLE && filter_mode==3'd0) || (RGB_STROKE_ENABLE && filter_mode>=6));
 wire bv=thin_selected?nv:ev,bs=thin_selected?ns:es,be=thin_selected?ne:ee;
 wire display_edge=thin_selected?(thin_magnitude>compare_threshold):edge_bit;
 wire use_stroke=RGB_STROKE_ENABLE && (filter_mode==0 || filter_mode>=5);
 wire [10:0] edge_magnitude=thin_selected?thin_magnitude:magnitude;
 wire [1:0] edge_direction=thin_selected?thin_direction:direction;
 wire [2:0] original_code=filter_mode!=5?(((filter_mode==0&&display_edge)||(filter_mode==6&&thin_center_magnitude>compare_threshold))?3'd1:3'd0):!display_edge?3'd0:filter_color_direction?
     ((edge_direction==0)?3'd4:(edge_direction==1)?3'd3:(edge_direction==2)?3'd1:3'd5):
     ((edge_magnitude<192)?3'd1:(edge_magnitude<384)?3'd2:(edge_magnitude<576)?3'd3:3'd4);
 wire sv,ss,se;wire [2:0] stroke_code,stroke_original_code;
 generate if(RGB_STROKE_ENABLE)begin: g_stroke
   edge_color_stroke #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) stroke(
    .clk(clk),.rst_n(rst_n),.sink_valid(bv&&use_stroke),.sink_sop(bs),.sink_eop(be),.sink_code(original_code),
    .source_valid(sv),.source_sop(ss),.source_eop(se),.source_code(stroke_code),.source_original_code(stroke_original_code));
 end else begin: g_no_stroke
   assign sv=0;assign ss=0;assign se=0;assign stroke_code=0;assign stroke_original_code=0;
 end endgenerate
 wire dv=use_stroke?sv:bv,ds=use_stroke?ss:bs,de=use_stroke?se:be;
 wire cv,cs,ce;
 wire [23:0] context_data;
 generate if(RGB_STROKE_ENABLE)begin: g_compact_context
   wire [15:0] rgb_context;
   wire [7:0] ctx_r={rgb_context[15:11],3'b111},ctx_g={rgb_context[10:5],2'b11},ctx_b={rgb_context[4:0],3'b111};
   wire [17:0] gray_sum=ctx_r*18'd306+ctx_g*18'd601+ctx_b*18'd117;
   assign context_data={gray_sum[17:10],rgb_context};
   pixel_context_fifo #(.DATA_WIDTH(16),.DEPTH(6144)) context_fifo(
    .clk(clk),.rst_n(rst_n),.write_valid(sink_valid),.write_sop(sink_sop),.write_data(sink_rgb565),
    .read_valid(dv),.read_sop(ds),.read_eop(de),.source_valid(cv),.source_sop(cs),.source_eop(ce),.source_data(rgb_context));
 end else begin: g_original_context
   pixel_context_fifo #(.DATA_WIDTH(24),.DEPTH(CONTEXT_DEPTH)) context_fifo(
    .clk(clk),.rst_n(rst_n),.write_valid(sink_valid),.write_sop(sink_sop),.write_data({sink_gray,sink_rgb565}),
    .read_valid(dv),.read_sop(ds),.read_eop(de),.source_valid(cv),.source_sop(cs),.source_eop(ce),.source_data(context_data));
 end endgenerate
 reg [11:0] x,y,px,py;
 wire [11:0] current_x=ds?12'd0:x,current_y=ds?12'd0:y;
 reg edge_delay;
 reg [2:0] stroke_code_delay;
 function [15:0] stroke_color;
 input [2:0] code;
 begin case(code)1:stroke_color=16'h07ff;2:stroke_color=16'h07e0;3:stroke_color=16'hffe0;4:stroke_color=16'hf800;default:stroke_color=16'hf81f;endcase end endfunction
 wire [15:0] stroke_pixel=stroke_code_delay!=0?stroke_color(stroke_code_delay):context_data[15:0];
 reg [1:0] direction_delay;
 reg [10:0] magnitude_delay;
 reg active_color_direction,direction_style_delay;
 wire [15:0] direction_pixel;
 color_edge_overlay #(.STRENGTH_ENABLE(STRENGTH_COLOR_ENABLE)) direction_overlay(.rgb_in(context_data[15:0]),.edge_valid(edge_delay),.direction(direction_delay),.magnitude(magnitude_delay),.strength_mode(!direction_style_delay),.rgb_out(direction_pixel));
 reg [7:0] center_delay,intensity_delay;
 reg [2:0] active_mode,mode_delay;
 reg active_gaussian,gaussian_delay;
 reg [15:0] bcd_threshold;
 localparam OBJECTS=8;
 wire [3:0] detected_count;
 wire [2*OBJECTS-1:0] detected_shapes;
 wire [12*OBJECTS-1:0] detected_x0,detected_x1,detected_y0,detected_y1;
 wire shape_result_valid,shape_busy,shape_overrun,shape_overflow,shape_malformed;
 wire [159:0] shape_diagnostics;
 wire shape_frame_accepted;
 wire [6:0] recognition_threshold=(active_threshold>11'd64)?7'd64:active_threshold[6:0];
 reg [6:0] threshold_for_job,diagnostic_threshold_pending,diagnostic_threshold_for_frame;
 reg [159:0] diagnostic_pending,diagnostic_for_frame;
 reg diagnostic_ready;
 generate if(SHAPE_ENABLE && !EXTERNAL_SHAPES) begin: g_shape
   // Geometry needs a continuous contour even when the display threshold
   // hides weak edges. Use the same frame-fenced magnitude, with a dedicated
   // sensitivity ceiling; the Sobel/Canny display pixels retain their scale.
   wire recognition_edge=(magnitude>recognition_threshold);
   shape_component_detector #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MAX_OBJECTS(OBJECTS)) detector(
   .clk(clk),.rst_n(rst_n),.sink_valid(ev),.sink_sop(es),.sink_eop(ee),.sink_edge(recognition_edge),
   .result_valid(shape_result_valid),.candidate_count(detected_count),.shapes(detected_shapes),
   .boxes_x0(detected_x0),.boxes_x1(detected_x1),.boxes_y0(detected_y0),.boxes_y1(detected_y1),
   .diagnostic_data(shape_diagnostics),.frame_accepted(shape_frame_accepted),
   .edge_counts(),.busy(shape_busy),.overrun(shape_overrun),.overflow(shape_overflow),.malformed(shape_malformed));
 end else begin: g_no_shape
   assign shape_diagnostics=0;assign shape_frame_accepted=0;
   assign detected_count=EXTERNAL_SHAPES?input_shapes[403:400]:0;assign detected_shapes=EXTERNAL_SHAPES?input_shapes[399:384]:0;
   assign detected_x0=EXTERNAL_SHAPES?input_shapes[383:288]:0;assign detected_x1=EXTERNAL_SHAPES?input_shapes[287:192]:0;assign detected_y0=EXTERNAL_SHAPES?input_shapes[191:96]:0;assign detected_y1=EXTERNAL_SHAPES?input_shapes[95:0]:0;
   assign shape_result_valid=EXTERNAL_SHAPES;assign shape_busy=0;assign shape_overrun=0;assign shape_overflow=EXTERNAL_SHAPES&&input_shapes[404];assign shape_malformed=0;
 end endgenerate
 reg [1:0] shape_for_frame;
 reg [3:0] count_for_frame;
 reg [2*OBJECTS-1:0] shapes_for_frame;
 reg [12*OBJECTS-1:0] box_x0,box_x1,box_y0,box_y1;
 reg limited_for_frame;
 reg shape_pending;
 wire usable_shape=!shape_overrun && !shape_overflow && !shape_malformed;
 wire [15:0] shapes_pixel;
 shape_boxes_overlay #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MAX_OBJECTS(OBJECTS)) boxes_overlay(
   .x(px),.y(py),.object_count(count_for_frame),.shapes(shapes_for_frame),
   .boxes_x0(box_x0),.boxes_x1(box_x1),.boxes_y0(box_y0),.boxes_y1(box_y1),
   .limited(limited_for_frame),.rgb_in(context_data[15:0]),.rgb_out(shapes_pixel));
 function [15:0] to_bcd;
   input [10:0] binary;
   integer i,j;
   reg [15:0] result;
   begin
     result=0;
     for(i=10;i>=0;i=i-1) begin
       for(j=0;j<4;j=j+1)
         if(result[j*4+:4]>=5) result[j*4+:4]=result[j*4+:4]+4'd3;
       result={result[14:0],binary[i]};
     end
     to_bcd=result;
   end
 endfunction
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     threshold_for_job<=0;diagnostic_pending<=0;diagnostic_for_frame<=0;diagnostic_ready<=0;
     diagnostic_threshold_pending<=0;diagnostic_threshold_for_frame<=0;
     x<=0;y<=0;px<=0;py<=0;edge_delay<=0;stroke_code_delay<=0;center_delay<=0;intensity_delay<=0;direction_delay<=0;magnitude_delay<=0;active_color_direction<=!STRENGTH_COLOR_ENABLE;direction_style_delay<=!STRENGTH_COLOR_ENABLE;
     active_mode<=0;mode_delay<=0;bcd_threshold<=0;active_gaussian<=0;gaussian_delay<=0;
     shape_for_frame<=0;count_for_frame<=0;shapes_for_frame<=0;limited_for_frame<=0;shape_pending<=0;box_x0<=0;box_x1<=0;box_y0<=0;box_y1<=0;
   end else begin
     if(shape_frame_accepted)threshold_for_job<=recognition_threshold;
     if(shape_result_valid)begin
       shape_pending<=1;diagnostic_pending<=shape_diagnostics;diagnostic_threshold_pending<=threshold_for_job;
     end
     if(dv) begin
     if(current_x==WIDTH-1) begin x<=0;y<=(current_y==HEIGHT-1)?0:current_y+1'b1;end
     else begin x<=current_x+1'b1;y<=current_y;end
     px<=current_x;py<=current_y;
     edge_delay<=use_stroke?((filter_mode==5)?(stroke_code!=0):(stroke_original_code!=0)):display_edge;stroke_code_delay<=stroke_code;center_delay<=center;
     direction_delay<=thin_selected?thin_direction:direction;
     magnitude_delay<=thin_selected?thin_magnitude:magnitude;
     direction_style_delay<=ds?filter_color_direction:active_color_direction;
     intensity_delay<=(magnitude>1020)?8'd255:magnitude[9:2];
     mode_delay<=ds?filter_mode:active_mode;
     gaussian_delay<=ds?filter_gaussian_frame:active_gaussian;
     if(ds) begin
       active_mode<=filter_mode;active_gaussian<=filter_gaussian_frame;active_color_direction<=filter_color_direction;bcd_threshold<=to_bcd(active_threshold);
       if(shape_pending || shape_result_valid) begin
         shape_pending<=0;
         diagnostic_for_frame<=shape_result_valid?shape_diagnostics:diagnostic_pending;
         diagnostic_threshold_for_frame<=shape_result_valid?threshold_for_job:diagnostic_threshold_pending;
         diagnostic_ready<=1;
         shape_for_frame<=usable_shape?detected_shapes[1:0]:2'd0;
         count_for_frame<=usable_shape?detected_count:4'd0;
         shapes_for_frame<=usable_shape?detected_shapes:0;
         limited_for_frame<=!usable_shape;
         box_x0<=detected_x0;box_x1<=detected_x1;
         box_y0<=detected_y0;box_y1<=detected_y1;
       end
     end
     end
   end
 end
 function [15:0] grayscale565;
   input [7:0] value;
   begin grayscale565={value[7:3],value[7:2],value[7:3]};end
 endfunction
 reg [15:0] pixel;
 always @* begin
   case(mode_delay)
     0:pixel=(px==WIDTH/2)?16'h07e0:(px<WIDTH/2)?grayscale565(context_data[23:16]):{16{edge_delay}};
     1:pixel=grayscale565(context_data[23:16]);
     2:pixel=grayscale565(center_delay);
     3:pixel=grayscale565(intensity_delay);
     4:pixel={16{edge_delay}};
     5:pixel=RGB_STROKE_ENABLE?stroke_pixel:DIRECTION_COLOR_ENABLE?direction_pixel:(edge_delay?16'hf800:context_data[15:0]);
     6:pixel=SHAPE_ENABLE?shapes_pixel:context_data[15:0];
     7:pixel=context_data[15:0]; // Fully processed, frame-fenced Canny DDR result.
     default:pixel=grayscale565(context_data[23:16]);
   endcase
 end
 wire [15:0] with_osd;
 video_status_osd  #(.DIAGNOSTICS_ENABLE(SHAPE_ENABLE),.FILTER_PREVIEW_ENABLE(FILTER_PREVIEW_ENABLE),.DIRECTION_LEGEND_ENABLE(DIRECTION_COLOR_ENABLE),.STRENGTH_COLOR_ENABLE(STRENGTH_COLOR_ENABLE)) status_text(.x(px),.y(py),.mode(mode_delay),.filter_gaussian(gaussian_delay),.color_direction(direction_style_delay),.shape(shape_for_frame),.threshold_bcd(bcd_threshold),.diagnostics(diagnostic_for_frame),
   .recognition_threshold(diagnostic_threshold_for_frame),.diagnostic_ready(diagnostic_ready),.object_count(count_for_frame),.rgb_in(pixel),.rgb_out(with_osd));
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin source_valid<=0;source_sop<=0;source_eop<=0;source_rgb565<=0;source_compare_data<=0;source_mode<=0;source_threshold_bcd<=0;end
   else begin
     source_valid<=cv;source_sop<=cs;source_eop<=ce;
     source_rgb565<=OSD_ENABLE?with_osd:pixel;
     source_compare_data<={context_data[23:16],edge_delay};
     source_mode<=mode_delay;source_threshold_bcd<=bcd_threshold;
   end
 end
endmodule
