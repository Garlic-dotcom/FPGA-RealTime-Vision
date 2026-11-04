// Two 128-bit AXI masters, one slave, same clock. Flattened buses place master0
// in the low slice. One outstanding write AND one outstanding read, independent.
// Round-robin ownership is held through B / final R acceptance, not just address
// acceptance. No write-data interleaving. IDs can be tied to zero at the slave.
// Reset must also reset/drain the attached masters and slave transaction state.
module axi_two_master_mux(
 input clk,rst_n,
 input [63:0] m_awaddr,m_araddr,
 input [15:0] m_awlen,m_arlen,
 input [5:0] m_awsize,m_arsize,
 input [3:0] m_awburst,m_arburst,
 input [1:0] m_awvalid,m_arvalid,
 output [1:0] m_awready,m_arready,
 input [255:0] m_wdata,
 input [31:0] m_wstrb,
 input [1:0] m_wlast,m_wvalid,
 output [1:0] m_wready,
 output [3:0] m_bresp,
 output [1:0] m_bvalid,
 input [1:0] m_bready,
 output [255:0] m_rdata,
 output [3:0] m_rresp,
 output [1:0] m_rlast,m_rvalid,
 input [1:0] m_rready,
 output [31:0] s_awaddr,s_araddr,
 output [7:0] s_awlen,s_arlen,
 output [2:0] s_awsize,s_arsize,
 output [1:0] s_awburst,s_arburst,
 output s_awvalid,s_arvalid,
 input s_awready,s_arready,
 output [127:0] s_wdata,
 output [15:0] s_wstrb,
 output s_wlast,s_wvalid,
 input s_wready,
 input [1:0] s_bresp,
 input s_bvalid,
 output s_bready,
 input [127:0] s_rdata,
 input [1:0] s_rresp,
 input s_rlast,s_rvalid,
 output s_rready
);
 reg write_active,read_active,write_owner,read_owner,write_turn,read_turn;
 reg aw_pending,ar_pending,write_data_done;
 wire choose_write=(m_awvalid==2'b11)?write_turn:m_awvalid[1];
 wire choose_read=(m_arvalid==2'b11)?read_turn:m_arvalid[1];
 wire write_select=write_active?write_owner:choose_write;
 wire read_select=read_active?read_owner:choose_read;
 wire write_response=write_active && !aw_pending && write_data_done;
 // An idle read address can be accepted on the first cycle. A slave may also
 // return its first R beat in that cycle, before read_owner is registered.
 wire read_response=(read_active && !ar_pending) || (s_arvalid && s_arready);
 // Idle AW and its selected master's W can cross on the first cycle. The
 // owner is latched even if AW is stalled or W completes before AW. B remains
 // registered-owner-only, avoiding a combinational response/ready path.
 assign s_awaddr=m_awaddr[32*write_select+:32];
 assign s_awlen=m_awlen[8*write_select+:8];
 assign s_awsize=m_awsize[3*write_select+:3];
 assign s_awburst=m_awburst[2*write_select+:2];
 assign s_awvalid=rst_n && (write_active?(aw_pending && m_awvalid[write_owner]):(|m_awvalid));
 assign m_awready=(s_awvalid && s_awready)?(2'b01<<write_select):2'b00;
 assign s_wdata=m_wdata[128*write_select+:128];
 assign s_wstrb=m_wstrb[16*write_select+:16];
 assign s_wlast=m_wlast[write_select];
 assign s_wvalid=rst_n && (write_active?!write_data_done:(|m_awvalid)) && m_wvalid[write_select];
 assign m_wready=(s_wvalid && s_wready)?(2'b01<<write_select):2'b00;
 assign m_bresp={s_bresp,s_bresp};
 assign m_bvalid=(rst_n && write_response && s_bvalid)?(2'b01<<write_owner):2'b00;
 assign s_bready=rst_n && write_response && m_bready[write_owner];
 assign s_araddr=m_araddr[32*read_select+:32];
 assign s_arlen=m_arlen[8*read_select+:8];
 assign s_arsize=m_arsize[3*read_select+:3];
 assign s_arburst=m_arburst[2*read_select+:2];
 assign s_arvalid=rst_n && (read_active?(ar_pending && m_arvalid[read_owner]):(|m_arvalid));
 assign m_arready=(s_arvalid && s_arready)?(2'b01<<read_select):2'b00;
 assign m_rdata={s_rdata,s_rdata};
 assign m_rresp={s_rresp,s_rresp};
 assign m_rlast={s_rlast,s_rlast};
 assign m_rvalid=(rst_n && read_response && s_rvalid)?(2'b01<<read_select):2'b00;
 assign s_rready=rst_n && read_response && m_rready[read_select];
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     write_active<=0;read_active<=0;write_owner<=0;read_owner<=0;write_turn<=0;read_turn<=0;
     aw_pending<=0;ar_pending<=0;write_data_done<=0;
   end else begin
     if(!write_active && m_awvalid!=0) begin
       write_active<=1;write_owner<=choose_write;aw_pending<=!(s_awvalid && s_awready);write_data_done<=0;
     end
     if(s_awvalid && s_awready) aw_pending<=0;
     if(s_wvalid && s_wready && s_wlast) write_data_done<=1;
     if(s_bvalid && s_bready) begin write_active<=0;write_turn<=!write_owner;end
     if(!read_active && m_arvalid!=0) begin
       read_active<=1;read_owner<=choose_read;ar_pending<=!(s_arvalid && s_arready);
     end
     if(s_arvalid && s_arready) ar_pending<=0;
     if(s_rvalid && s_rready && s_rlast) begin read_active<=0;read_turn<=!read_select;end
   end
 end
endmodule
