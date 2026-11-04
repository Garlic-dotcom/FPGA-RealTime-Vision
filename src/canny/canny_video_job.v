// Full stored RGB565 -> grayscale -> Gaussian -> gradient/direction -> NMS ->
// thresholds -> exact global hysteresis -> RGB565 DDR result. clk = AXI clock.
// Caller owns both raw leases until input_done and disjoint scratch/result until
// done. done includes the algorithm flush, result B, and LIVE history B fences.
module canny_video_job #(parameter WIDTH=1280,HEIGHT=720,CREDIT_DEPTH=8192,
 parameter TEMPORAL_BASE=32'h09c00000)(
 input clk,rst_n,start,split_frame,temporal_enable,temporal_history_valid,
 input contour_mode,
 output [404:0] contour_shapes,
 input [31:0] source_base,second_base,destination_base,label_base,queue_base,
 input [10:0] threshold,
 output reg busy,done,fault,
 output input_done,
 output reg [31:0] total_cycles,input_cycles,algorithm_cycles,
 output [31:0] hysteresis_cycles,accepted_count,
 output [31:0] axi_awaddr,
 output [7:0] axi_awlen,
 output [2:0] axi_awsize,
 output [1:0] axi_awburst,
 output [0:0] axi_awvalid,
 input [0:0] axi_awready,
 output [127:0] axi_wdata,
 output [15:0] axi_wstrb,
 output [0:0] axi_wlast,
 output [0:0] axi_wvalid,
 input [0:0] axi_wready,
 input [1:0] axi_bresp,
 input [0:0] axi_bvalid,
 output [0:0] axi_bready,
 output [31:0] axi_araddr,
 output [7:0] axi_arlen,
 output [2:0] axi_arsize,
 output [1:0] axi_arburst,
 output [0:0] axi_arvalid,
 input [0:0] axi_arready,
 input [127:0] axi_rdata,
 input [1:0] axi_rresp,
 input [0:0] axi_rlast,
 input [0:0] axi_rvalid,
 output [0:0] axi_rready
);
 // Same monotone user sensitivity scale as the ordinary Sobel path. The
 // generic classifier still receives explicit physical L1 high/low values;
 // low is half the mapped high, so T160 remains exactly high160/low80.
 function [10:0] quality_threshold;
   input [10:0] user_threshold;
   reg [11:0] value,delta;
   begin
     if(user_threshold<=11'd32) value={1'b0,user_threshold}+({1'b0,user_threshold}>>1);
     else if(user_threshold<11'd160) begin
       delta=12'd160-{1'b0,user_threshold};
       value={1'b0,user_threshold}+(delta>>3);
     end else if(user_threshold<=11'd320) begin
       delta={1'b0,user_threshold}-12'd160;
       value=12'd160+(delta>>1);
     end else begin
       delta={1'b0,user_threshold}-12'd320;
       value={1'b0,user_threshold}-12'd80+(delta>>4);
     end
     quality_threshold=(value>12'd2047)?11'd2047:value[10:0];
   end
 endfunction
 wire [10:0] effective_threshold=quality_threshold(threshold);
 wire [31:0] d_awaddr;
 wire [7:0] d_awlen;
 wire [2:0] d_awsize;
 wire [1:0] d_awburst;
 wire [0:0] d_awvalid;
 wire [0:0] d_awready;
 wire [127:0] d_wdata;
 wire [15:0] d_wstrb;
 wire [0:0] d_wlast;
 wire [0:0] d_wvalid;
 wire [0:0] d_wready;
 wire [1:0] d_bresp;
 wire [0:0] d_bvalid;
 wire [0:0] d_bready;
 wire [31:0] d_araddr;
 wire [7:0] d_arlen;
 wire [2:0] d_arsize;
 wire [1:0] d_arburst;
 wire [0:0] d_arvalid;
 wire [0:0] d_arready;
 wire [127:0] d_rdata;
 wire [1:0] d_rresp;
 wire [0:0] d_rlast;
 wire [0:0] d_rvalid;
 wire [0:0] d_rready;
 wire [31:0] c_awaddr;
 wire [7:0] c_awlen;
 wire [2:0] c_awsize;
 wire [1:0] c_awburst;
 wire [0:0] c_awvalid;
 wire [0:0] c_awready;
 wire [127:0] c_wdata;
 wire [15:0] c_wstrb;
 wire [0:0] c_wlast;
 wire [0:0] c_wvalid;
 wire [0:0] c_wready;
 wire [1:0] c_bresp;
 wire [0:0] c_bvalid;
 wire [0:0] c_bready;
 wire [31:0] c_araddr;
 wire [7:0] c_arlen;
 wire [2:0] c_arsize;
 wire [1:0] c_arburst;
 wire [0:0] c_arvalid;
 wire [0:0] c_arready;
 wire [127:0] c_rdata;
 wire [1:0] c_rresp;
 wire [0:0] c_rlast;
 wire [0:0] c_rvalid;
 wire [0:0] c_rready;
 wire [31:0] registered_c_araddr;
 wire [7:0] registered_c_arlen;
 wire [2:0] registered_c_arsize;
 wire [1:0] registered_c_arburst;
 wire registered_c_arvalid,registered_c_arready;
 wire [31:0] t_awaddr,t_araddr,dt_awaddr,dt_araddr;
 wire [7:0] t_awlen,t_arlen,dt_awlen,dt_arlen;
 wire [2:0] t_awsize,t_arsize,dt_awsize,dt_arsize;
 wire [1:0] t_awburst,t_arburst,dt_awburst,dt_arburst;
 wire t_awvalid,t_awready,t_wlast,t_wvalid,t_wready,t_bvalid,t_bready;
 wire t_arvalid,t_arready,t_rlast,t_rvalid,t_rready;
 wire dt_awvalid,dt_awready,dt_wlast,dt_wvalid,dt_wready,dt_bvalid,dt_bready;
 wire dt_arvalid,dt_arready,dt_rlast,dt_rvalid,dt_rready;
 wire [127:0] t_wdata,t_rdata,dt_wdata,dt_rdata;
 wire [15:0] t_wstrb,dt_wstrb;
 wire [1:0] t_bresp,t_rresp,dt_bresp,dt_rresp;
 wire tr,tv,ts,te,tdone,tfault;
 wire [7:0] stable_gray;
 wire launch=start && !busy && !fault;
 wire rv,rs,re,rr,rdone,rfault,gv,gs,ge,qready,qvalid,cr,cv,cs,ce,cdone,cfault,wr,wdone,wfault;
 wire [15:0] rgb;
 wire [7:0] gray,edge_data;
 wire [9:0] queued;
 reg [5:0] gray_outstanding;
 reg saw_algorithm,saw_writer,saw_temporal;
 wire contours=contour_mode===1'b1;
 reg second_pass;
 wire second_start=contours && cv && !second_pass;
 wire feed=rv && rr && !second_pass;
 wire [10:0] shape_threshold=threshold>64?11'd64:threshold;
 wire retire=qvalid && tr && !fault;
 assign rr=second_pass?(busy&&!fault&&cv&&wr):(busy&&!fault&&gray_outstanding<32);
 assign input_done=rdone && (!contours||second_pass);
 axi_frame_reader #(.PIXELS(WIDTH*HEIGHT),.SPLIT_ENABLE(1)) reader(
 .clk(clk),.rst_n(rst_n),.start(launch||second_start),.frame_base(source_base),.second_base(second_base),.split_frame(split_frame),
 .busy(),.done(rdone),.fault(rfault),.source_valid(rv),.source_sop(rs),.source_eop(re),.source_data(rgb),.source_ready(rr),
 .axi_araddr(d_araddr),
 .axi_arlen(d_arlen),
 .axi_arsize(d_arsize),
 .axi_arburst(d_arburst),
 .axi_arvalid(d_arvalid),
 .axi_arready(d_arready),
 .axi_rdata(d_rdata),
 .axi_rresp(d_rresp),
 .axi_rlast(d_rlast),
 .axi_rvalid(d_rvalid),
 .axi_rready(d_rready));
 vip_rgb2gray_proc grayscale(.clk(clk),.rst_n(rst_n),.sink_valid(feed),.sink_sop(rs&&feed),.sink_eop(re&&feed),
 .sink_data({rgb[15:11],3'b111,rgb[10:5],2'b11,rgb[4:0],3'b111}),
 .source_valid(gv),.source_sop(gs),.source_eop(ge),.source_data(gray));
 stream_fifo #(.DATA_WIDTH(10),.DEPTH(32)) gray_fifo(.clk(clk),.rst_n(rst_n),
 .sink_valid(gv),.sink_data({gs,ge,gray}),.sink_ready(qready),
 .source_valid(qvalid),.source_data(queued),.source_ready(tr&&!fault));
 canny_temporal_gray #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.HISTORY_BASE(TEMPORAL_BASE)) temporal(
 .clk(clk),.rst_n(rst_n),.start(launch),.enable(temporal_enable),.history_valid(temporal_history_valid),
 .busy(),.done(tdone),.fault(tfault),
 .sink_valid(qvalid&&!fault),.sink_sop(queued[9]),.sink_eop(queued[8]),.sink_data(queued[7:0]),.sink_ready(tr),
 .source_valid(tv),.source_sop(ts),.source_eop(te),.source_data(stable_gray),.source_ready(cr&&!fault),
 .axi_awaddr(t_awaddr),.axi_awlen(t_awlen),.axi_awsize(t_awsize),.axi_awburst(t_awburst),
 .axi_awvalid(t_awvalid),.axi_awready(t_awready),.axi_wdata(t_wdata),.axi_wstrb(t_wstrb),
 .axi_wlast(t_wlast),.axi_wvalid(t_wvalid),.axi_wready(t_wready),
 .axi_bresp(t_bresp),.axi_bvalid(t_bvalid),.axi_bready(t_bready),
 .axi_araddr(t_araddr),.axi_arlen(t_arlen),.axi_arsize(t_arsize),.axi_arburst(t_arburst),
 .axi_arvalid(t_arvalid),.axi_arready(t_arready),.axi_rdata(t_rdata),.axi_rresp(t_rresp),
 .axi_rlast(t_rlast),.axi_rvalid(t_rvalid),.axi_rready(t_rready));
 canny_frame_processor #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.CREDIT_DEPTH(CREDIT_DEPTH)) algorithm(
 .clk(clk),.rst_n(rst_n),.start(launch),.contour_mode(contours),.contour_raw_base(source_base),.contour_second_base(second_base),.contour_split_frame(split_frame),.contour_shapes(contour_shapes),.label_base(label_base),.queue_base(queue_base),
 .low_threshold(contours?shape_threshold:{1'b0,effective_threshold[10:1]}),.high_threshold(contours?shape_threshold:effective_threshold),.busy(),.done(cdone),.fault(cfault),
 .sink_valid(tv&&!fault),.sink_sop(ts),.sink_eop(te),.sink_data(stable_gray),.sink_ready(cr),
 .source_valid(cv),.source_sop(cs),.source_eop(ce),.source_data(edge_data),.source_ready(wr&&!fault&&(!contours||(second_pass&&rv))),
 .cycle_count(hysteresis_cycles),.accepted_count(accepted_count),
 .axi_awaddr(c_awaddr),
 .axi_awlen(c_awlen),
 .axi_awsize(c_awsize),
 .axi_awburst(c_awburst),
 .axi_awvalid(c_awvalid),
 .axi_awready(c_awready),
 .axi_wdata(c_wdata),
 .axi_wstrb(c_wstrb),
 .axi_wlast(c_wlast),
 .axi_wvalid(c_wvalid),
 .axi_wready(c_wready),
 .axi_bresp(c_bresp),
 .axi_bvalid(c_bvalid),
 .axi_bready(c_bready),
 .axi_araddr(c_araddr),
 .axi_arlen(c_arlen),
 .axi_arsize(c_arsize),
 .axi_arburst(c_arburst),
 .axi_arvalid(c_arvalid),
 .axi_arready(c_arready),
 .axi_rdata(c_rdata),
 .axi_rresp(c_rresp),
 .axi_rlast(c_rlast),
 .axi_rvalid(c_rvalid),
 .axi_rready(c_rready));
 axi_frame_writer #(.PIXELS(WIDTH*HEIGHT)) writer(.clk(clk),.rst_n(rst_n),.start(launch),.frame_base(destination_base),
 .busy(),.done(wdone),.fault(wfault),.sink_valid(cv&&!fault&&(!contours||(second_pass&&rv))),.sink_sop(cs),.sink_eop(ce),
 .sink_data(contours?rgb:{16{edge_data[7]}}),.sink_ready(wr),
 .axi_awaddr(d_awaddr),
 .axi_awlen(d_awlen),
 .axi_awsize(d_awsize),
 .axi_awburst(d_awburst),
 .axi_awvalid(d_awvalid),
 .axi_awready(d_awready),
 .axi_wdata(d_wdata),
 .axi_wstrb(d_wstrb),
 .axi_wlast(d_wlast),
 .axi_wvalid(d_wvalid),
 .axi_wready(d_wready),
 .axi_bresp(d_bresp),
 .axi_bvalid(d_bvalid),
 .axi_bready(d_bready));
 axi_two_master_mux temporal_dma_mux(.clk(clk),.rst_n(rst_n),
 .m_awaddr({t_awaddr,d_awaddr}),.s_awaddr(dt_awaddr),
 .m_awlen({t_awlen,d_awlen}),.s_awlen(dt_awlen),
 .m_awsize({t_awsize,d_awsize}),.s_awsize(dt_awsize),
 .m_awburst({t_awburst,d_awburst}),.s_awburst(dt_awburst),
 .m_awvalid({t_awvalid,d_awvalid}),.s_awvalid(dt_awvalid),
 .m_awready({t_awready,d_awready}),.s_awready(dt_awready),
 .m_wdata({t_wdata,d_wdata}),.s_wdata(dt_wdata),
 .m_wstrb({t_wstrb,d_wstrb}),.s_wstrb(dt_wstrb),
 .m_wlast({t_wlast,d_wlast}),.s_wlast(dt_wlast),
 .m_wvalid({t_wvalid,d_wvalid}),.s_wvalid(dt_wvalid),
 .m_wready({t_wready,d_wready}),.s_wready(dt_wready),
 .m_bresp({t_bresp,d_bresp}),.s_bresp(dt_bresp),
 .m_bvalid({t_bvalid,d_bvalid}),.s_bvalid(dt_bvalid),
 .m_bready({t_bready,d_bready}),.s_bready(dt_bready),
 .m_araddr({t_araddr,d_araddr}),.s_araddr(dt_araddr),
 .m_arlen({t_arlen,d_arlen}),.s_arlen(dt_arlen),
 .m_arsize({t_arsize,d_arsize}),.s_arsize(dt_arsize),
 .m_arburst({t_arburst,d_arburst}),.s_arburst(dt_arburst),
 .m_arvalid({t_arvalid,d_arvalid}),.s_arvalid(dt_arvalid),
 .m_arready({t_arready,d_arready}),.s_arready(dt_arready),
 .m_rdata({t_rdata,d_rdata}),.s_rdata(dt_rdata),
 .m_rresp({t_rresp,d_rresp}),.s_rresp(dt_rresp),
 .m_rlast({t_rlast,d_rlast}),.s_rlast(dt_rlast),
 .m_rvalid({t_rvalid,d_rvalid}),.s_rvalid(dt_rvalid),
 .m_rready({t_rready,d_rready}),.s_rready(dt_rready));
 // Cut the cached algorithm address/VALID decode before the shared DDR mux.
 // Preserve its original one-outstanding-read contract until accepted RLAST.
 canny_job_ar_cut algorithm_ar_cut(.clk(clk),.rst_n(rst_n),
 .s_araddr(c_araddr),.s_arlen(c_arlen),.s_arsize(c_arsize),.s_arburst(c_arburst),
 .s_arvalid(c_arvalid),.s_arready(c_arready),
 .m_araddr(registered_c_araddr),.m_arlen(registered_c_arlen),
 .m_arsize(registered_c_arsize),.m_arburst(registered_c_arburst),
 .m_arvalid(registered_c_arvalid),.m_arready(registered_c_arready),
 .response_last(c_rvalid && c_rready && c_rlast));
 axi_two_master_mux memory_mux(.clk(clk),.rst_n(rst_n),
 .m_awaddr({c_awaddr,dt_awaddr}),.s_awaddr(axi_awaddr),
 .m_awlen({c_awlen,dt_awlen}),.s_awlen(axi_awlen),
 .m_awsize({c_awsize,dt_awsize}),.s_awsize(axi_awsize),
 .m_awburst({c_awburst,dt_awburst}),.s_awburst(axi_awburst),
 .m_awvalid({c_awvalid,dt_awvalid}),.s_awvalid(axi_awvalid),
 .m_awready({c_awready,dt_awready}),.s_awready(axi_awready),
 .m_wdata({c_wdata,dt_wdata}),.s_wdata(axi_wdata),
 .m_wstrb({c_wstrb,dt_wstrb}),.s_wstrb(axi_wstrb),
 .m_wlast({c_wlast,dt_wlast}),.s_wlast(axi_wlast),
 .m_wvalid({c_wvalid,dt_wvalid}),.s_wvalid(axi_wvalid),
 .m_wready({c_wready,dt_wready}),.s_wready(axi_wready),
 .m_bresp({c_bresp,dt_bresp}),.s_bresp(axi_bresp),
 .m_bvalid({c_bvalid,dt_bvalid}),.s_bvalid(axi_bvalid),
 .m_bready({c_bready,dt_bready}),.s_bready(axi_bready),
 .m_araddr({registered_c_araddr,dt_araddr}),.s_araddr(axi_araddr),
 .m_arlen({registered_c_arlen,dt_arlen}),.s_arlen(axi_arlen),
 .m_arsize({registered_c_arsize,dt_arsize}),.s_arsize(axi_arsize),
 .m_arburst({registered_c_arburst,dt_arburst}),.s_arburst(axi_arburst),
 .m_arvalid({registered_c_arvalid,dt_arvalid}),.s_arvalid(axi_arvalid),
 .m_arready({registered_c_arready,dt_arready}),.s_arready(axi_arready),
 .m_rdata({c_rdata,dt_rdata}),.s_rdata(axi_rdata),
 .m_rresp({c_rresp,dt_rresp}),.s_rresp(axi_rresp),
 .m_rlast({c_rlast,dt_rlast}),.s_rlast(axi_rlast),
 .m_rvalid({c_rvalid,dt_rvalid}),.s_rvalid(axi_rvalid),
 .m_rready({c_rready,dt_rready}),.s_rready(axi_rready));

 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     busy<=0;done<=0;fault<=0;gray_outstanding<=0;second_pass<=0;saw_algorithm<=0;saw_writer<=0;saw_temporal<=0;
     total_cycles<=0;input_cycles<=0;algorithm_cycles<=0;
   end else begin
     done<=0;
     if(launch) begin
       busy<=1;saw_algorithm<=0;saw_writer<=0;saw_temporal<=0;gray_outstanding<=0;second_pass<=0;
       total_cycles<=0;input_cycles<=0;algorithm_cycles<=0;
     end
     if(busy) total_cycles<=total_cycles+1'b1;
     case({feed,retire}) 2'b10:gray_outstanding<=gray_outstanding+1'b1;2'b01:gray_outstanding<=gray_outstanding-1'b1;endcase
     if(rdone) input_cycles<=total_cycles;
     if(cdone) begin saw_algorithm<=1;algorithm_cycles<=total_cycles;end
     if(second_start)second_pass<=1;
     if(wdone) saw_writer<=1;
     if(tdone) saw_temporal<=1;
     if(busy && (saw_algorithm||cdone) && (saw_writer||wdone) && (saw_temporal||tdone)) begin busy<=0;done<=1;end
     if(rfault || cfault || wfault || tfault || (gv&&!qready) || (retire&&gray_outstanding==0)) begin
       busy<=0;done<=0;fault<=1;
     end
   end
 end
endmodule

// One registered AR slot. Source acceptance reserves the slot until final R,
// not only until forwarding AR, so this does not add outstanding transactions.
// Local job starts/faults cannot withdraw a queued request; shared reset can.
module canny_job_ar_cut(
 input clk,rst_n,
 input [31:0] s_araddr,
 input [7:0] s_arlen,
 input [2:0] s_arsize,
 input [1:0] s_arburst,
 input s_arvalid,
 output s_arready,
 output reg [31:0] m_araddr,
 output reg [7:0] m_arlen,
 output reg [2:0] m_arsize,
 output reg [1:0] m_arburst,
 output m_arvalid,
 input m_arready,response_last
);
 reg pending,occupied;
 assign s_arready=rst_n && !occupied;
 assign m_arvalid=rst_n && pending;
 wire capture=s_arvalid && s_arready;
 wire forward=m_arvalid && m_arready;
 wire retire=occupied && response_last && (!pending || forward);
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     pending<=0;occupied<=0;m_araddr<=0;m_arlen<=0;m_arsize<=0;m_arburst<=0;
   end else begin
     if(capture) begin
       pending<=1;occupied<=1;
       m_araddr<=s_araddr;m_arlen<=s_arlen;m_arsize<=s_arsize;m_arburst<=s_arburst;
     end
     if(forward) pending<=0;
     if(retire) occupied<=0;
   end
 end
endmodule
