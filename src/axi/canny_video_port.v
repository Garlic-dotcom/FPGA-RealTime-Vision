// Video and full Canny share the established DDR port. Three result slots
// prevent writer/latest/display collisions, including simultaneous publication
// and frame-boundary adoption. Scratch never overlaps 32 history slots/selftest.
module canny_video_port #(
 parameter WIDTH=1280,HEIGHT=720,CREDIT_DEPTH=8192,
 parameter RESULT_BASE=32'h09000000,LABEL_BASE=32'h08400000,QUEUE_BASE=32'h08800000,
 parameter TEMPORAL_BASE=32'h09c00000,
 parameter SLOT_SHIFT=22,
 // Default to spatial-only LIVE: do not blend weak moving detail with old frames.
 parameter LIVE_TEMPORAL_ENABLE=0
)(
 input clk,rst_n,enable,
 input contour_mode,
 output reg result_contour,
 output reg [404:0] result_shapes,
 // Only an explicitly asserted LIVE qualifier enables identical-input reuse.
 // History modes and legacy instances with this input unconnected still run
 // the original scheduling path.
 input live_mode,
 input [10:0] threshold,
 output process_request,process_release,
 input process_valid,process_compare,
 input [4:0] process_a,process_b,
 input [15:0] process_generation_a,process_generation_b,
 input display_active,
 input [31:0] display_base,
 output reg result_valid,
 output [31:0] result_base,
 output reg [10:0] result_threshold,
 output reg [15:0] result_generation_a,result_generation_b,
 output reg result_compare,
 output fault,busy,
 output reg [31:0] completed_frames,last_total_cycles,last_input_cycles,last_algorithm_cycles,last_hysteresis_cycles,
 // Last successful job's overlapping bus observations. These do not alter
 // scheduling, ALG/FPS definitions, or the final successful-B publication fence.
 output reg [31:0] last_ar_wait_cycles,last_read_wait_cycles,last_write_wait_cycles,
 input [31:0] video_awaddr,
 input [7:0] video_awlen,
 input [2:0] video_awsize,
 input [1:0] video_awburst,
 input [0:0] video_awvalid,
 output [0:0] video_awready,
 input [127:0] video_wdata,
 input [15:0] video_wstrb,
 input [0:0] video_wlast,
 input [0:0] video_wvalid,
 output [0:0] video_wready,
 output [1:0] video_bresp,
 output [0:0] video_bvalid,
 input [0:0] video_bready,
 input [31:0] video_araddr,
 input [7:0] video_arlen,
 input [2:0] video_arsize,
 input [1:0] video_arburst,
 input [0:0] video_arvalid,
 output [0:0] video_arready,
 output [127:0] video_rdata,
 output [1:0] video_rresp,
 output [0:0] video_rlast,
 output [0:0] video_rvalid,
 input [0:0] video_rready,
 output [31:0] ddr_awaddr,
 output [7:0] ddr_awlen,
 output [2:0] ddr_awsize,
 output [1:0] ddr_awburst,
 output [0:0] ddr_awvalid,
 input [0:0] ddr_awready,
 output [127:0] ddr_wdata,
 output [15:0] ddr_wstrb,
 output [0:0] ddr_wlast,
 output [0:0] ddr_wvalid,
 input [0:0] ddr_wready,
 input [1:0] ddr_bresp,
 input [0:0] ddr_bvalid,
 output [0:0] ddr_bready,
 output [31:0] ddr_araddr,
 output [7:0] ddr_arlen,
 output [2:0] ddr_arsize,
 output [1:0] ddr_arburst,
 output [0:0] ddr_arvalid,
 input [0:0] ddr_arready,
 input [127:0] ddr_rdata,
 input [1:0] ddr_rresp,
 input [0:0] ddr_rlast,
 input [0:0] ddr_rvalid,
 output [0:0] ddr_rready
);
 wire [31:0] j_awaddr;
 wire [7:0] j_awlen;
 wire [2:0] j_awsize;
 wire [1:0] j_awburst;
 wire [0:0] j_awvalid;
 wire [0:0] j_awready;
 wire [127:0] j_wdata;
 wire [15:0] j_wstrb;
 wire [0:0] j_wlast;
 wire [0:0] j_wvalid;
 wire [0:0] j_wready;
 wire [1:0] j_bresp;
 wire [0:0] j_bvalid;
 wire [0:0] j_bready;
 wire [31:0] j_araddr;
 wire [7:0] j_arlen;
 wire [2:0] j_arsize;
 wire [1:0] j_arburst;
 wire [0:0] j_arvalid;
 wire [0:0] j_arready;
 wire [127:0] j_rdata;
 wire [1:0] j_rresp;
 wire [0:0] j_rlast;
 wire [0:0] j_rvalid;
 wire [0:0] j_rready;
 reg [1:0] state,write_slot,latest_slot;
 reg [31:0] raw_a,raw_b;
 reg split;
 reg job_live,live_signature_valid,duplicate_release;
 reg job_contour;wire [404:0] job_shapes;
 reg job_temporal,job_history_valid,temporal_ready;
 reg [15:0] job_generation_a,job_generation_b;
 reg [10:0] job_threshold;
 wire job_done,job_busy,job_input_done;
 reg read_inflight;
 reg [31:0] ar_wait_cycles,read_wait_cycles,write_wait_cycles;
 wire [31:0] total,ic,ac,hc;
 wire [31:0] destination=RESULT_BASE+({30'd0,write_slot}<<SLOT_SHIFT);
 assign result_base=RESULT_BASE+({30'd0,latest_slot}<<SLOT_SHIFT);
 assign process_request=enable && !fault && state==0;
 assign process_release=job_input_done || duplicate_release;
 assign busy=state!=0;
 reg [1:0] free_slot;
 integer i;
 always @* begin
   free_slot=0;
   for(i=0;i<3;i=i+1)
     if((!result_valid || i!=latest_slot) &&
        (!display_active || RESULT_BASE+(i<<SLOT_SHIFT)!=display_base)) free_slot=i;
 end
 canny_video_job #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.CREDIT_DEPTH(CREDIT_DEPTH),.TEMPORAL_BASE(TEMPORAL_BASE)) job(
 .clk(clk),.rst_n(rst_n),.start(state==1 && !fault),.split_frame(split),.contour_mode(job_contour),.contour_shapes(job_shapes),
 .temporal_enable(job_temporal),.temporal_history_valid(job_history_valid),
 .source_base(raw_a),.second_base(raw_b),.destination_base(destination),.label_base(LABEL_BASE),.queue_base(QUEUE_BASE),
 .threshold(job_threshold),.busy(job_busy),.done(job_done),.fault(fault),.input_done(job_input_done),
 .total_cycles(total),.input_cycles(ic),.algorithm_cycles(ac),.hysteresis_cycles(hc),.accepted_count(),
 .axi_awaddr(j_awaddr),
 .axi_awlen(j_awlen),
 .axi_awsize(j_awsize),
 .axi_awburst(j_awburst),
 .axi_awvalid(j_awvalid),
 .axi_awready(j_awready),
 .axi_wdata(j_wdata),
 .axi_wstrb(j_wstrb),
 .axi_wlast(j_wlast),
 .axi_wvalid(j_wvalid),
 .axi_wready(j_wready),
 .axi_bresp(j_bresp),
 .axi_bvalid(j_bvalid),
 .axi_bready(j_bready),
 .axi_araddr(j_araddr),
 .axi_arlen(j_arlen),
 .axi_arsize(j_arsize),
 .axi_arburst(j_arburst),
 .axi_arvalid(j_arvalid),
 .axi_arready(j_arready),
 .axi_rdata(j_rdata),
 .axi_rresp(j_rresp),
 .axi_rlast(j_rlast),
 .axi_rvalid(j_rvalid),
 .axi_rready(j_rready));
 axi_two_master_mux video_job_mux(.clk(clk),.rst_n(rst_n),
 .m_awaddr({j_awaddr,video_awaddr}),.s_awaddr(ddr_awaddr),
 .m_awlen({j_awlen,video_awlen}),.s_awlen(ddr_awlen),
 .m_awsize({j_awsize,video_awsize}),.s_awsize(ddr_awsize),
 .m_awburst({j_awburst,video_awburst}),.s_awburst(ddr_awburst),
 .m_awvalid({j_awvalid,video_awvalid}),.s_awvalid(ddr_awvalid),
 .m_awready({j_awready,video_awready}),.s_awready(ddr_awready),
 .m_wdata({j_wdata,video_wdata}),.s_wdata(ddr_wdata),
 .m_wstrb({j_wstrb,video_wstrb}),.s_wstrb(ddr_wstrb),
 .m_wlast({j_wlast,video_wlast}),.s_wlast(ddr_wlast),
 .m_wvalid({j_wvalid,video_wvalid}),.s_wvalid(ddr_wvalid),
 .m_wready({j_wready,video_wready}),.s_wready(ddr_wready),
 .m_bresp({j_bresp,video_bresp}),.s_bresp(ddr_bresp),
 .m_bvalid({j_bvalid,video_bvalid}),.s_bvalid(ddr_bvalid),
 .m_bready({j_bready,video_bready}),.s_bready(ddr_bready),
 .m_araddr({j_araddr,video_araddr}),.s_araddr(ddr_araddr),
 .m_arlen({j_arlen,video_arlen}),.s_arlen(ddr_arlen),
 .m_arsize({j_arsize,video_arsize}),.s_arsize(ddr_arsize),
 .m_arburst({j_arburst,video_arburst}),.s_arburst(ddr_arburst),
 .m_arvalid({j_arvalid,video_arvalid}),.s_arvalid(ddr_arvalid),
 .m_arready({j_arready,video_arready}),.s_arready(ddr_arready),
 .m_rdata({j_rdata,video_rdata}),.s_rdata(ddr_rdata),
 .m_rresp({j_rresp,video_rresp}),.s_rresp(ddr_rresp),
 .m_rlast({j_rlast,video_rlast}),.s_rlast(ddr_rlast),
 .m_rvalid({j_rvalid,video_rvalid}),.s_rvalid(ddr_rvalid),
 .m_rready({j_rready,video_rready}),.s_rready(ddr_rready));

 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     state<=0;write_slot<=0;latest_slot<=0;result_valid<=0;result_threshold<=160;
     raw_a<=0;raw_b<=0;split<=0;job_contour<=0;result_contour<=0;result_shapes<=0;job_threshold<=160;completed_frames<=0;
     job_live<=0;live_signature_valid<=0;duplicate_release<=0;
     job_temporal<=0;job_history_valid<=0;temporal_ready<=0;
     job_generation_a<=0;job_generation_b<=0;result_generation_a<=0;result_generation_b<=0;result_compare<=0;
     last_total_cycles<=0;last_input_cycles<=0;last_algorithm_cycles<=0;last_hysteresis_cycles<=0;
     read_inflight<=0;ar_wait_cycles<=0;read_wait_cycles<=0;write_wait_cycles<=0;
     last_ar_wait_cycles<=0;last_read_wait_cycles<=0;last_write_wait_cycles<=0;
   end else begin
     // A skipped reservation is released with a single independent pulse;
     // sticky faults must not stretch that pulse indefinitely.
     duplicate_release<=0;
     if(fault || !enable || live_mode!==1'b1 || threshold!=result_threshold)
       temporal_ready<=0;
     if(!fault) begin
     case(live_mode)
       1'b1:if(!enable) begin live_signature_valid<=0;job_live<=0;end
       default:begin live_signature_valid<=0;job_live<=0;end
     endcase
     // The in-flight job retains its captured controls. A session/configuration
     // interruption invalidates its history for every subsequent job.
     if(state!=0 && threshold!=job_threshold) begin job_live<=0;temporal_ready<=0;end
     // One read is outstanding at this port. Same-cycle final R takes
     // precedence so a zero-latency response cannot leave a stale pending bit.
     if(state==1) begin
       read_inflight<=0;ar_wait_cycles<=0;read_wait_cycles<=0;write_wait_cycles<=0;
     end else if(state==2) begin
       if(j_arvalid && j_arready) read_inflight<=1;
       if(j_rvalid && j_rready && j_rlast) read_inflight<=0;
       if(job_busy) begin
         if(j_arvalid && !j_arready) ar_wait_cycles<=ar_wait_cycles+1'b1;
         if(read_inflight && !j_rvalid) read_wait_cycles<=read_wait_cycles+1'b1;
         if((j_awvalid && !j_awready) || (j_wvalid && !j_wready))
           write_wait_cycles<=write_wait_cycles+1'b1;
       end
     end
     case(state)
       0:if(process_valid) begin
         // The current result is reusable only within an uninterrupted LIVE
         // session and after successful full-job completion. Compare both the
         // logical identities and physical leases; configuration changes must
         // start a fresh task even when the camera frame is unchanged.
         if(enable && live_mode && live_signature_valid && result_valid &&
            threshold==result_threshold && (contour_mode===1'b1)==result_contour && process_compare==result_compare &&
            process_generation_a==result_generation_a && process_generation_b==result_generation_b &&
            process_a==raw_a[26:22] && process_b==raw_b[26:22]) begin
           duplicate_release<=1;state<=3;
         end else begin
         raw_a<={5'd0,process_a,22'd0};raw_b<={5'd0,process_b,22'd0};split<=process_compare;
         job_generation_a<=process_generation_a;job_generation_b<=process_generation_b;
         job_threshold<=threshold;write_slot<=free_slot;state<=1;job_contour<=contour_mode===1'b1;
         job_temporal<=contour_mode!==1'b1 && LIVE_TEMPORAL_ENABLE && enable && live_mode===1'b1 && !process_compare;
         job_history_valid<=LIVE_TEMPORAL_ENABLE && temporal_ready && enable && live_mode===1'b1 &&
                            !process_compare && !result_compare && threshold==result_threshold;
         if(live_mode) job_live<=1;else job_live<=0;
         end
       end
       1:state<=2;
       2:if(job_done) begin
         latest_slot<=write_slot;result_valid<=1;result_threshold<=job_threshold;result_contour<=job_contour;result_shapes<=job_shapes;
         result_generation_a<=job_generation_a;result_generation_b<=job_generation_b;result_compare<=split;
         if(enable && live_mode && job_live && threshold==job_threshold) live_signature_valid<=1;
         else live_signature_valid<=0;
         temporal_ready<=job_temporal && !split && enable && live_mode===1'b1 &&
                         job_live && threshold==job_threshold;
         completed_frames<=completed_frames+1'b1;last_total_cycles<=total;last_input_cycles<=ic;
         last_algorithm_cycles<=ac;last_hysteresis_cycles<=hc;state<=3;
         last_ar_wait_cycles<=ar_wait_cycles;last_read_wait_cycles<=read_wait_cycles;
         last_write_wait_cycles<=write_wait_cycles;
       end
       3:if(!process_valid) state<=0;
     endcase
     end
   end
 end
endmodule
