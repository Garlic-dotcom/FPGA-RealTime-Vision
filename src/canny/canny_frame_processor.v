// Complete grayscale stream -> frontend -> AXI byte labels -> exact hysteresis.
// All ports use clk. Input MUST honor ready (intended for a stored frame reader).
// Caller owns disjoint aligned label/queue ranges until done. No external writes.
// CREDIT_DEPTH is power-of-two and must exceed 4*WIDTH+96 (four windows).
// Fault is sticky; reset this block and outstanding bus transactions together.
module canny_frame_processor #(
 parameter WIDTH=1280,HEIGHT=720,CREDIT_DEPTH=8192
)(
 input clk,rst_n,start,
 input contour_mode,
 input [31:0] contour_raw_base,contour_second_base,
 input contour_split_frame,
 output [404:0] contour_shapes,
 input [31:0] label_base,queue_base,
 input [10:0] low_threshold,high_threshold,
 input sink_valid,sink_sop,sink_eop,
 input [7:0] sink_data,
 output sink_ready,
 output reg busy,done,fault,
 output source_valid,source_sop,source_eop,
 output [7:0] source_data,
 input source_ready,
 output [31:0] cycle_count,accepted_count,
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
 localparam N=WIDTH*HEIGHT,LABEL_BYTES=((N+15)/16)*16,QUEUE_BYTES=N*4,CW=$clog2(CREDIT_DEPTH+1);
 wire [32:0] label_end={1'b0,label_base}+LABEL_BYTES;
 wire [32:0] queue_end={1'b0,queue_base}+QUEUE_BYTES;
 wire bad_range=label_base[3:0]!=0 || queue_base[3:0]!=0 || label_end[32] || queue_end[32] ||
     ({1'b0,label_base}<queue_end && {1'b0,queue_base}<label_end);
 reg phase_hys;
 wire contours=contour_mode===1'b1;
 reg engine_flushed,shape_finished;
 reg frontend_weak_seen;
 reg [31:0] active_labels,active_queue,input_count;
 reg [10:0] active_low,active_high;
 reg [CW-1:0] outstanding;
 wire launch=start && !busy && !fault && !bad_range;
 assign sink_ready=busy && !fault && !phase_hys && input_count<N && outstanding<CREDIT_DEPTH;
 wire feed=sink_valid && sink_ready;
 wire packet_ok=(sink_sop==(input_count==0)) && (sink_eop==(input_count==N-1));
 wire lv,ls,le,qr,qv,wr,wb,wdone,wfault;
 wire [1:0] label;
 wire [3:0] queued;
 canny_classify_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) frontend(
 .clk(clk),.rst_n(rst_n),.contour_mode(contours),.sink_valid(feed&&packet_ok),.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_data(sink_data),
 .low_threshold(active_low),.high_threshold(active_high),.source_valid(lv),.source_sop(ls),.source_eop(le),.source_class(label));
 stream_fifo #(.DATA_WIDTH(4),.DEPTH(CREDIT_DEPTH)) label_fifo(
 .clk(clk),.rst_n(rst_n),.sink_valid(lv),.sink_data({ls,le,label}),.sink_ready(qr),
 .source_valid(qv),.source_data(queued),.source_ready(wr&&!fault));
 wire retire=qv && wr && !fault;
 wire [31:0] law,haw;
 wire [7:0] lal,hal;
 wire [2:0] las,has;
 wire [1:0] lab,hab;
 wire lav,hawvalid,lwv,hwv,lwl,hwl,lbr,hbr;
 wire [127:0] lwd,hwd;
 wire [15:0] lws,hws;
 axi_label_writer #(.PIXELS(N)) label_writer(
 .clk(clk),.rst_n(rst_n),.start(launch),.frame_base(label_base),.busy(wb),.done(wdone),.fault(wfault),
 .sink_valid(qv&&!fault),.sink_sop(queued[3]),.sink_eop(queued[2]),.sink_data(queued[1:0]),.sink_ready(wr),
 .axi_awaddr(law),.axi_awlen(lal),.axi_awsize(las),.axi_awburst(lab),.axi_awvalid(lav),.axi_awready(axi_awready&&!phase_hys),
 .axi_wdata(lwd),.axi_wstrb(lws),.axi_wvalid(lwv),.axi_wready(axi_wready&&!phase_hys),.axi_wlast(lwl),
 .axi_bresp(axi_bresp),.axi_bvalid(axi_bvalid&&!phase_hys),.axi_bready(lbr));
 wire hstart=wdone && busy && !fault && !phase_hys;
 wire hb,hdone,hv,hs,he,mv,mw,mr,mrv,hfault;
 wire [7:0] hd;
 wire [31:0] ma,md,mrd;
 wire [127:0] mline;
 wire [3:0] ms;
 // Recognition uses the same thresholded frontend but no contour coloring.
 // Equal low/high thresholds in SHAPES produce only strong/background labels.
 // Reuse the existing emitter; the job joins its packet with original RGB.
 canny_hysteresis_engine #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.NO_WEAK_HINT_ENABLE(1),
                           .WIDE_SCAN_ENABLE(WIDTH%16==0)) hysteresis(
 .clk(clk),.rst_n(rst_n),.start(hstart),.no_weak_hint(!frontend_weak_seen),.label_base(active_labels),.queue_base(active_queue),.busy(hb),.done(hdone),
 .mem_valid(mv),.mem_write(mw),.mem_address(ma),.mem_wdata(md),.mem_wstrb(ms),.mem_ready(mr),.mem_rvalid(mrv),.mem_rdata(mrd),.mem_line_rdata(mline),
 .source_valid(hv),.source_sop(hs),.source_eop(he),.source_data(hd),.source_ready(source_ready&&!fault),
 .cycle_count(cycle_count),.accepted_count(accepted_count));
 wire shape_result,shape_overrun,shape_overflow,shape_malformed;
 wire [3:0] shape_count;wire [15:0] shape_kinds;
 wire [95:0] shape_x0,shape_x1,shape_y0,shape_y1;
 shape_component_detector #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) contour_detector(
 .clk(clk),.rst_n(rst_n),.sink_valid(lv&&contours),.sink_sop(ls),.sink_eop(le),.sink_edge(label==2),
 .result_valid(shape_result),.candidate_count(shape_count),.shapes(shape_kinds),
 .boxes_x0(shape_x0),.boxes_x1(shape_x1),.boxes_y0(shape_y0),.boxes_y1(shape_y1),
 .edge_counts(),.busy(),.overrun(shape_overrun),.overflow(shape_overflow),.malformed(shape_malformed),.skipped_frames(),.diagnostic_data(),.frame_accepted());
 wire shape_limited=shape_overrun||shape_overflow||shape_malformed;
 assign contour_shapes={shape_limited,shape_limited?4'd0:shape_count,shape_kinds,shape_x0,shape_x1,shape_y0,shape_y1};
 wire flush_done,prefetch_fault;
 wire [31:0] baraddr;
 wire [7:0] barlen;
 wire [2:0] barsize;
 wire [1:0] barburst,brresp;
 wire barvalid,barready,brlast,brvalid,brready;
 wire [127:0] brdata;
 canny_queue_axi128_bridge #(.QUEUE_WORDS(N),.CACHE_LINES(32)) bridge(
 .clk(clk),.rst_n(rst_n),.cache_clear(hstart),.queue_base(active_queue),.flush(hdone),.flush_done(flush_done),.queue_write_beats(),
 .mem_valid(mv),.mem_write(mw),.mem_address(ma),.mem_wdata(md),.mem_wstrb(ms),
 .mem_ready(mr),.mem_rvalid(mrv),.mem_rdata(mrd),.mem_line_rdata(mline),.fault(hfault),
 .axi_awaddr(haw),.axi_awlen(hal),.axi_awsize(has),.axi_awburst(hab),.axi_awvalid(hawvalid),.axi_awready(axi_awready&&phase_hys),
 .axi_wdata(hwd),.axi_wstrb(hws),.axi_wvalid(hwv),.axi_wready(axi_wready&&phase_hys),.axi_wlast(hwl),
 .axi_bresp(axi_bresp),.axi_bvalid(axi_bvalid&&phase_hys),.axi_bready(hbr),
 .axi_araddr(baraddr),.axi_arlen(barlen),.axi_arsize(barsize),.axi_arburst(barburst),.axi_arvalid(barvalid),.axi_arready(barready),
 .axi_rdata(brdata),.axi_rresp(brresp),.axi_rlast(brlast),.axi_rvalid(brvalid),.axi_rready(brready));
 canny_label_read_prefetch #(.LABEL_BYTES(LABEL_BYTES),.CACHE_BLOCKS(4)) label_prefetch(
 .clk(clk),.rst_n(rst_n),.cache_clear(hstart),.label_base(active_labels),.fault(prefetch_fault),
 .write_awaddr(haw),.write_awvalid(hawvalid&&phase_hys),.write_awready(axi_awready&&phase_hys),
 .write_bvalid(axi_bvalid&&phase_hys),.write_bready(hbr&&phase_hys),.write_bresp(axi_bresp),
 .upstream_araddr(baraddr),.upstream_arlen(barlen),.upstream_arsize(barsize),.upstream_arburst(barburst),
 .upstream_arvalid(barvalid),.upstream_arready(barready),
 .upstream_rdata(brdata),.upstream_rresp(brresp),.upstream_rlast(brlast),.upstream_rvalid(brvalid),.upstream_rready(brready),
 .axi_araddr(axi_araddr),.axi_arlen(axi_arlen),.axi_arsize(axi_arsize),.axi_arburst(axi_arburst),.axi_arvalid(axi_arvalid),.axi_arready(axi_arready),
 .axi_rdata(axi_rdata),.axi_rresp(axi_rresp),.axi_rlast(axi_rlast),.axi_rvalid(axi_rvalid),.axi_rready(axi_rready));
 // Exclusive phases, switched only after the final label B response.
 assign axi_awaddr=phase_hys?haw:law;
 assign axi_awlen=phase_hys?hal:lal;
 assign axi_awsize=phase_hys?has:las;
 assign axi_awburst=phase_hys?hab:lab;
 assign axi_awvalid=phase_hys?hawvalid:lav;
 assign axi_wdata=phase_hys?hwd:lwd;
 assign axi_wstrb=phase_hys?hws:lws;
 assign axi_wvalid=phase_hys?hwv:lwv;
 assign axi_wlast=phase_hys?hwl:lwl;
 assign axi_bready=phase_hys?hbr:lbr;
 assign source_valid=hv&&!fault;
 assign source_sop=hs;
 assign source_eop=he;
 assign source_data=hd;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     busy<=0;done<=0;fault<=0;phase_hys<=0;input_count<=0;outstanding<=0;frontend_weak_seen<=0;
     active_labels<=0;active_queue<=0;active_low<=0;active_high<=0;engine_flushed<=0;shape_finished<=0;
   end else begin
     done<=0;
     if(start && !busy && !fault) begin
       if(bad_range) fault<=1;
       else begin
         busy<=1;phase_hys<=0;input_count<=0;outstanding<=0;frontend_weak_seen<=0;
         active_labels<=label_base;active_queue<=queue_base;active_low<=low_threshold;active_high<=high_threshold;engine_flushed<=0;shape_finished<=0;
       end
     end
     // hstart follows the final label B response, so this summarizes the
     // complete classified frame, including pixels buffered in label_fifo.
     if(lv && label==1) frontend_weak_seen<=1;
     if(feed) input_count<=input_count+1'b1;
     case({feed,retire}) 2'b10:outstanding<=outstanding+1'b1;2'b01:outstanding<=outstanding-1'b1;endcase
     if(hstart) phase_hys<=1;
     // Engine cycle_count excludes this final memory-visibility fence.
     if(shape_result)shape_finished<=1;
     if(flush_done)engine_flushed<=1;
     if(busy && (flush_done||engine_flushed) && (!contours||shape_finished||shape_result)) begin busy<=0;done<=1;end
     if(wfault || hfault || prefetch_fault || (feed&&!packet_ok) || (lv&&!qr) || (retire&&outstanding==0)) begin
       fault<=1;busy<=0;done<=0;
     end
   end
 end
endmodule
