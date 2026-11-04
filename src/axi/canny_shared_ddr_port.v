// Preserve the existing video controller as master 0. Diagnostic scratch owns
// master 1; response ownership stays latched through successful handshakes.
module canny_shared_ddr_port #(parameter START_DELAY=96000000)(
 input clk,rst_n,enable,
 output done,failed,
 output [2:0] test_index,
 output [31:0] total_cycles,
 input [31:0] video_awaddr,
 input [31:0] video_araddr,
 input [7:0] video_awlen,
 input [7:0] video_arlen,
 input [2:0] video_awsize,
 input [2:0] video_arsize,
 input [1:0] video_awburst,
 input [1:0] video_arburst,
 input video_awvalid,
 input video_arvalid,
 output video_awready,
 output video_arready,
 input [127:0] video_wdata,
 input [15:0] video_wstrb,
 input video_wlast,
 input video_wvalid,
 output video_wready,
 output [1:0] video_bresp,
 output video_bvalid,
 input video_bready,
 output [127:0] video_rdata,
 output [1:0] video_rresp,
 output video_rlast,
 output video_rvalid,
 input video_rready,
 output [31:0] ddr_awaddr,
 output [31:0] ddr_araddr,
 output [7:0] ddr_awlen,
 output [7:0] ddr_arlen,
 output [2:0] ddr_awsize,
 output [2:0] ddr_arsize,
 output [1:0] ddr_awburst,
 output [1:0] ddr_arburst,
 output ddr_awvalid,
 output ddr_arvalid,
 input ddr_awready,
 input ddr_arready,
 output [127:0] ddr_wdata,
 output [15:0] ddr_wstrb,
 output ddr_wlast,
 output ddr_wvalid,
 input ddr_wready,
 input [1:0] ddr_bresp,
 input ddr_bvalid,
 output ddr_bready,
 input [127:0] ddr_rdata,
 input [1:0] ddr_rresp,
 input ddr_rlast,
 input ddr_rvalid,
 output ddr_rready
);
 wire [31:0] q_awaddr;
 wire [31:0] q_araddr;
 wire [7:0] q_awlen;
 wire [7:0] q_arlen;
 wire [2:0] q_awsize;
 wire [2:0] q_arsize;
 wire [1:0] q_awburst;
 wire [1:0] q_arburst;
 wire q_awvalid;
 wire q_arvalid;
 wire q_awready;
 wire q_arready;
 wire [127:0] q_wdata;
 wire [15:0] q_wstrb;
 wire q_wlast;
 wire q_wvalid;
 wire q_wready;
 wire [1:0] q_bresp;
 wire q_bvalid;
 wire q_bready;
 wire [127:0] q_rdata;
 wire [1:0] q_rresp;
 wire q_rlast;
 wire q_rvalid;
 wire q_rready;
 canny_ddr_selftest #(.START_DELAY(START_DELAY)) selftest(
 .clk(clk),.rst_n(rst_n),.enable(enable),.done(done),.failed(failed),.test_index(test_index),.total_cycles(total_cycles),
 .axi_awaddr(q_awaddr),
 .axi_araddr(q_araddr),
 .axi_awlen(q_awlen),
 .axi_arlen(q_arlen),
 .axi_awsize(q_awsize),
 .axi_arsize(q_arsize),
 .axi_awburst(q_awburst),
 .axi_arburst(q_arburst),
 .axi_awvalid(q_awvalid),
 .axi_arvalid(q_arvalid),
 .axi_awready(q_awready),
 .axi_arready(q_arready),
 .axi_wdata(q_wdata),
 .axi_wstrb(q_wstrb),
 .axi_wlast(q_wlast),
 .axi_wvalid(q_wvalid),
 .axi_wready(q_wready),
 .axi_bresp(q_bresp),
 .axi_bvalid(q_bvalid),
 .axi_bready(q_bready),
 .axi_rdata(q_rdata),
 .axi_rresp(q_rresp),
 .axi_rlast(q_rlast),
 .axi_rvalid(q_rvalid),
 .axi_rready(q_rready));
 axi_two_master_mux mux(
 .clk(clk),.rst_n(rst_n),
 .m_awaddr({q_awaddr,video_awaddr}),.s_awaddr(ddr_awaddr),
 .m_araddr({q_araddr,video_araddr}),.s_araddr(ddr_araddr),
 .m_awlen({q_awlen,video_awlen}),.s_awlen(ddr_awlen),
 .m_arlen({q_arlen,video_arlen}),.s_arlen(ddr_arlen),
 .m_awsize({q_awsize,video_awsize}),.s_awsize(ddr_awsize),
 .m_arsize({q_arsize,video_arsize}),.s_arsize(ddr_arsize),
 .m_awburst({q_awburst,video_awburst}),.s_awburst(ddr_awburst),
 .m_arburst({q_arburst,video_arburst}),.s_arburst(ddr_arburst),
 .m_awvalid({q_awvalid,video_awvalid}),.s_awvalid(ddr_awvalid),
 .m_arvalid({q_arvalid,video_arvalid}),.s_arvalid(ddr_arvalid),
 .m_awready({q_awready,video_awready}),.s_awready(ddr_awready),
 .m_arready({q_arready,video_arready}),.s_arready(ddr_arready),
 .m_wdata({q_wdata,video_wdata}),.s_wdata(ddr_wdata),
 .m_wstrb({q_wstrb,video_wstrb}),.s_wstrb(ddr_wstrb),
 .m_wlast({q_wlast,video_wlast}),.s_wlast(ddr_wlast),
 .m_wvalid({q_wvalid,video_wvalid}),.s_wvalid(ddr_wvalid),
 .m_wready({q_wready,video_wready}),.s_wready(ddr_wready),
 .m_bresp({q_bresp,video_bresp}),.s_bresp(ddr_bresp),
 .m_bvalid({q_bvalid,video_bvalid}),.s_bvalid(ddr_bvalid),
 .m_bready({q_bready,video_bready}),.s_bready(ddr_bready),
 .m_rdata({q_rdata,video_rdata}),.s_rdata(ddr_rdata),
 .m_rresp({q_rresp,video_rresp}),.s_rresp(ddr_rresp),
 .m_rlast({q_rlast,video_rlast}),.s_rlast(ddr_rlast),
 .m_rvalid({q_rvalid,video_rvalid}),.s_rvalid(ddr_rvalid),
 .m_rready({q_rready,video_rready}),.s_rready(ddr_rready));
endmodule

