//  by CrazyBird
module axi4_ctrl #(
	parameter HISTORY_ENABLE=0,CANNY_ENABLE=0,
              C_ID_LEN      = 8, 
			C_DATA_LEN    = 128, 
			C_DATA_SIZE   = 4, 	//	C_DATA_LEN=8:0, 16:1, 32:2, 64:3, 128:4, 256:5, 512:6, 1024:7
			
			C_BURST_LEN   = 128,
			
			C_STRB_LEN    = C_DATA_LEN / 8, 
			C_ADDR_INC    = C_BURST_LEN * C_STRB_LEN,
			
			C_BASE_ADDR   = 32'h00000000, 	//	Start at 0x00000000 by default
			C_BUF_SIZE    = 22, 			//	Allocate 2^22 bytes (4MB) by default. The module takes 4*4MB by default. 
			C_RD_END_ADDR = 1280 * 720,
			
			C_W_WIDTH     = 64, 
			C_R_WIDTH     = 8
)(
    input  wire                 axi_clk         ,
    input  wire                 axi_reset       ,
    
    output       [C_ID_LEN-1:0]      axi_awid        ,
    output       [31:0]      axi_awaddr      ,
    output       [ 7:0]      axi_awlen       ,
    output       [ 2:0]      axi_awsize      ,
    output       [ 1:0]      axi_awburst     ,
    output                   axi_awlock      ,
    output       [ 3:0]      axi_awcache     ,
    output       [ 2:0]      axi_awprot      ,
    output       [ 3:0]      axi_awqos       ,
    output reg                  axi_awvalid     ,
    input  wire                 axi_awready     ,
    
    output       [C_DATA_LEN-1:0]      axi_wdata       ,
    output       [C_STRB_LEN-1:0]      axi_wstrb       ,
    output                   axi_wlast       ,
    output reg                  axi_wvalid      ,
    input  wire                 axi_wready      ,
    
    input  wire     [C_ID_LEN-1:0]      axi_bid         ,
    input  wire     [ 1:0]      axi_bresp       ,
    input  wire                 axi_bvalid      ,
    output                   axi_bready      ,
    
    output       [C_ID_LEN-1:0]      axi_arid        ,
    output       [31:0]      axi_araddr      ,
    output       [ 7:0]      axi_arlen       ,
    output       [ 2:0]      axi_arsize      ,
    output       [ 1:0]      axi_arburst     ,
    output                   axi_arlock      ,
    output       [ 3:0]      axi_arcache     ,
    output       [ 2:0]      axi_arprot      ,
    output       [ 3:0]      axi_arqos       ,
    output reg                  axi_arvalid     ,
    input  wire                 axi_arready     ,
    
    input  wire     [C_ID_LEN-1:0]      axi_rid         ,
    input  wire     [C_DATA_LEN-1:0]      axi_rdata       ,
    input  wire     [ 1:0]      axi_rresp       ,
    input  wire                 axi_rlast       ,
    input  wire                 axi_rvalid      ,
    output                   axi_rready      ,
    
    input  wire                 wframe_pclk     ,
    input  wire                 wframe_vsync    ,		//	Writter VSync. Flush on falling edge. Connect to EOF. 
    input  wire                 wframe_data_en  ,
    input  wire     [C_W_WIDTH-1:0]      wframe_data     ,
    
    input  wire                 rframe_pclk     ,
    input  wire                 rframe_vsync    ,		//	Reader VSync. Flush on falling edge. Connect to ~EOF. 
    input  wire                 rframe_data_en  ,
    output wire     [C_R_WIDTH-1:0]      rframe_data     ,
    output wire raw_read_empty,
    
    input [1:0] history_mode,
    input history_older,history_newer,
    output [15:0] history_status,
    input process_request,process_release,
    output process_valid,process_compare,
    output [4:0] process_a,process_b,
    output [15:0] process_generation_a,process_generation_b,
    input [2:0] requested_display_mode,
    input [10:0] requested_threshold,result_threshold,
    input result_valid,
    input result_contour,
    input [404:0] result_shapes,
    output reg [404:0] display_shapes,
    input [15:0] result_generation_a,result_generation_b,
    input result_compare,
    input [31:0] result_base,
    output reg result_display_active,
    output reg [31:0] result_display_base,
    output reg [2:0] applied_display_mode,
    output reg [10:0] applied_threshold,
    output reg [31:0] display_metadata,
    // Read-only identity of the raw source adopted at the DDR read boundary.
    // This is distinct from a complete HDMI output, which FPS already counts.
    output [15:0] adopted_generation_a,
    output 		[31:0] 	tp_o
);
	
	initial begin
		axi_awvalid <= 0; 
		axi_wvalid <= 0; 
		axi_arvalid <= 0; 
	end

	assign axi_awid    = {C_ID_LEN{1'b0}};
	assign axi_awlen   = C_BURST_LEN - 1'b1;
	assign axi_awsize  = C_DATA_SIZE;
	assign axi_awburst = 2'b01;	//	INCR
	assign axi_awlock  = 1'b0;
	assign axi_awcache = 0;
	assign axi_awprot  = 3'b0;
	assign axi_awqos   = 4'b0;
	assign axi_wstrb   = 32'hFFFFFFFF;
	assign axi_bready  = 1; 
	    
	assign axi_arid    = {C_ID_LEN{1'b0}};
	assign axi_arlen   = C_BURST_LEN - 1'b1;
	assign axi_arsize  = C_DATA_SIZE;
	assign axi_arburst = 2'b01;	//	INCR
	assign axi_arlock  = 1'b0;
	assign axi_arcache = 0;
	assign axi_arprot  = 3'b0;
	assign axi_arqos   = 4'b0;
	assign axi_rready  = 1; 
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	R/W Schedule
    reg [C_BUF_SIZE-1:0] rc_w_ptr=0;
    wire [4:0] history_wslot,history_ra,history_rb;
    wire history_compare,history_ready,stop_capture;
    wire [1:0] active_history_mode;
    wire [5:0] history_count,history_age;
    wire [15:0] read_generation_a,read_generation_b;
    reg [4:0] rc_wframe_index,rc_rframe_index;
    reg r_wframe_inc=0,r_rframe_inc=0;
    reg frame_error=0;
    reg [10:0] writes_pending=0;
    // A whole camera frame is either admitted or discarded. The first pixel
    // precedes the frame_level rising edge, so decide on data_en, not VSYNC.
    reg stop_meta,stop_sync,in_capture,selected,camera_vs_d;
    wire admit=in_capture ? selected : !stop_sync;
    wire capture_wen=wframe_data_en && (!HISTORY_ENABLE || admit);
    wire capture_level=wframe_vsync && (!HISTORY_ENABLE || selected);
    always @(posedge wframe_pclk or posedge axi_reset) begin
      if(axi_reset) begin stop_meta<=0;stop_sync<=0;in_capture<=0;selected<=0;camera_vs_d<=0;end
      else begin
        stop_meta<=stop_capture;stop_sync<=stop_meta;camera_vs_d<=wframe_vsync;
        if(!in_capture && wframe_data_en) begin in_capture<=1;selected<=!stop_sync;end
        if(camera_vs_d && !wframe_vsync) in_capture<=0;
      end
    end
    reg quiet_meta,quiet_sync;
    always @(posedge axi_clk) begin
      if(axi_reset) begin quiet_meta<=0;quiet_sync<=0;end
      else begin quiet_meta<=stop_sync && !(in_capture && selected);quiet_sync<=quiet_meta;end
    end
    wire capture_quiet=quiet_sync && rs_w==0 && !r_weof_pending && w_wfifo_empty &&
                       !axi_awvalid && writes_pending==0;
    generate if(HISTORY_ENABLE) begin: history_schedule
      frame_history_ctrl #(.PROCESS_PIN_ENABLE(CANNY_ENABLE)) directory(.clk(axi_clk),.rst(axi_reset),
        .process_request(process_request),.process_release(process_release),.process_valid(process_valid),
        .process_a(process_a),.process_b(process_b),.process_compare(process_compare),
        .process_generation_a(process_generation_a),.process_generation_b(process_generation_b),
        .read_generation_a(read_generation_a),.read_generation_b(read_generation_b),
        .commit(r_wframe_inc && !frame_error && rc_w_ptr==C_RD_END_ADDR),
        .view_tick(r_rframe_inc),.capture_quiet(capture_quiet),.requested_mode(history_mode),
        .older(history_older),.newer(history_newer),.stop_capture(stop_capture),
        .write_slot(history_wslot),.read_a(history_ra),.read_b(history_rb),
        .compare_frame(history_compare),.read_ready(history_ready),.active_mode(active_history_mode),
        .history_count(history_count),.history_age(history_age));
      always @* begin rc_wframe_index=history_wslot;rc_rframe_index=history_ra;end
    end else begin: legacy_schedule
      wire [1:0] w_wframe_index_p1=rc_wframe_index+1;
      wire [1:0] w_wframe_index_next=(w_wframe_index_p1==rc_rframe_index)?rc_wframe_index+2:rc_wframe_index+1;
      reg [1:0] r_wframe_index_last=0;
      assign stop_capture=0;assign history_compare=0;assign history_ready=1;
      assign history_rb=0;assign active_history_mode=0;assign history_count=0;assign history_age=0;
      assign process_valid=0;assign process_a=0;assign process_b=0;assign process_compare=0;
      assign process_generation_a=0;assign process_generation_b=0;
      assign read_generation_a=0;assign read_generation_b=0;
	always @(posedge axi_clk) begin
		if(axi_reset) begin
			rc_wframe_index <= 2'b0;
			rc_rframe_index <= 2'd2;
			r_wframe_index_last <= 0; 
			
		end else begin
			rc_wframe_index <= rc_wframe_index; 
			rc_rframe_index <= rc_rframe_index; 
			
			//	When wfifo_rd_rst_busy_neg, write pointer increments. When rfifo_wr_rst_busy_neg, read pointer increments. 
			case ({r_rframe_inc, r_wframe_inc})
				2'b01: begin
						//	Write increment only. 
						rc_wframe_index <= w_wframe_index_next; 	//	Use 4 buffers. 
						r_wframe_index_last <= rc_wframe_index; 
					end
				2'b10: begin
						//	Use r_wframe_index_last. 
						rc_rframe_index <= r_wframe_index_last; 
					end
				2'b11: begin
						//	Update write & read pointer simutaneously. 
						rc_wframe_index <= w_wframe_index_next; 	//	Use 4 buffers. 
						rc_rframe_index <= rc_wframe_index; 		
					end
			endcase
			
		end
	end
	
	
	    end endgenerate
    assign history_status={frame_error || read_fault,1'b0,active_history_mode,history_count,history_age};
    // Select an acknowledged result only at the existing FIFO/frame boundary.
    // Keep this lease until the next boundary even when the buttons change.
    reg [15:0] selected_result_generation_a,selected_result_generation_b;
    reg selected_result_compare;
    wire stored_mode=requested_display_mode==7 || requested_display_mode==6;
    wire matching_result=result_valid && ((requested_display_mode==6 && result_contour===1'b1) || (requested_display_mode==7 && result_contour!==1'b1));
    always @(posedge axi_clk) begin
      if(axi_reset) begin
        result_display_active<=0;result_display_base<=0;display_shapes<=0;applied_display_mode<=0;applied_threshold<=160;
        selected_result_generation_a<=0;selected_result_generation_b<=0;selected_result_compare<=0;
      end else if(CANNY_ENABLE && r_rframe_inc) begin
        result_display_active<=stored_mode && matching_result;display_shapes<=result_shapes;
        result_display_base<=result_base;
        applied_display_mode<=(stored_mode && !matching_result)?3'd1:requested_display_mode;
        applied_threshold<=(stored_mode && matching_result)?result_threshold:requested_threshold;
        selected_result_generation_a<=result_generation_a;selected_result_generation_b<=result_generation_b;
        selected_result_compare<=result_compare;
      end
    end
    // Wait for actual AXI B completion before publishing a stored frame.
    always @(posedge axi_clk) begin
      if(axi_reset) begin writes_pending<=0;frame_error<=0;end
      else if(HISTORY_ENABLE) begin
        case({axi_awvalid && axi_awready,axi_bvalid && axi_bready})
          2'b10:writes_pending<=writes_pending+1'b1;
          2'b01:writes_pending<=writes_pending-1'b1;
          default:;
        endcase
        if(axi_bvalid && axi_bresp!=0) frame_error<=1;
        if(r_wframe_inc && rc_w_ptr!=C_RD_END_ADDR) frame_error<=1;
      end
    end

	////////////////////////////////////////////////////////////////
	//	AXI Writter

	wire 				w_wfifo_pempty, w_wfifo_empty; 
	wire 				w_wfifo_ren; 
	wire 	[C_DATA_LEN-1:0] 	w_wfifo_rdata; 

	//	On EOF when ~empty, flush the last data then reset FIFO. 
	reg 	[3:0] 	rs_w = 0; 
	wire 	[3:0] 	ws_w_idle = 0; 
	wire 	[3:0] 	ws_w_wdata = 1; 
	wire 	[3:0] 	ws_w_winc = 2; 
	wire 	[3:0] 	ws_w_eof = 3; 

	//	Write Pointer. Burst bytes is always C_ADDR_INC. 
	 
	reg 	[0:0] 		rc_w_eof = 0; 
	reg 				r_w_rst = 1; 
	
	//	Burst Control
	reg 	[7:0] 	rc_burst = 0; 	//	Compare to C_BURST_LEN - 1. 
	
	//	EOF Monitor. 
	reg 	[1:0] 	r_wframe_sync = 0; 	
	reg 			r_weof_pending = 0; 

	always @(posedge axi_clk or posedge axi_reset) begin
		if(axi_reset) begin
			rs_w <= 0; 
			rc_w_ptr <= 0; 
			rc_w_eof <= 0; 
			r_w_rst <= 1; 
			axi_awvalid <= 0; 
			axi_wvalid <= 0; 
			r_wframe_sync <= 0; 
			r_weof_pending <= 0; 
			r_wframe_inc <= 0; 
			
		end else begin
			rc_w_eof <= 0; 
			r_wframe_inc <= 0; 
			
			//	Raise EOF on falling edge of VSYNC. 
			r_wframe_sync <= {r_wframe_sync, capture_level}; 
			if(r_wframe_sync == 2'b10) begin
				r_weof_pending <= 1; 
			end else begin
			end
			
			if(axi_awready) begin
				axi_awvalid <= 0; 
			end else begin
			end
			
			case (rs_w)
				ws_w_idle: begin
						rc_burst <= 0; 
						r_w_rst <= 0; 
						
						if(~w_wfifo_pempty) begin
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
						
						end else if(r_weof_pending && (~w_wfifo_empty)) begin
							//	There's some data in the FIFO. write last. 
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
							
						end else if(r_weof_pending && (!HISTORY_ENABLE || (writes_pending==0 && !axi_awvalid))) begin
							//	EOF. Reset FIFO. Increment write pointer. 
							r_w_rst <= 1; 
							r_wframe_inc <= 1; 
							rs_w <= ws_w_eof; 
							
						end else begin
						end
					end
					
				ws_w_eof: begin
						//	Wait for some cycles to release reset. 
						r_weof_pending <= 0; 
						rc_w_ptr <= 0; 
						
						rc_w_eof <= rc_w_eof + 1; 
						if(&rc_w_eof)
							rs_w <= ws_w_idle; 
						else begin
						end
					end
					
				ws_w_wdata: begin
						rc_burst <= rc_burst + axi_wready; 
						
						//	On last transaction terminate write. 
						if(axi_wlast && axi_wready) begin
							axi_wvalid <= 0; 
							rs_w <= ws_w_winc; 
						end else begin
						end
					end
					
				ws_w_winc: begin
						if(!HISTORY_ENABLE || !axi_awvalid) begin rc_w_ptr <= rc_w_ptr + C_ADDR_INC; rs_w <= ws_w_idle;end 
					end
			endcase
		end
	end
	assign axi_awaddr = C_BASE_ADDR + {rc_wframe_index, rc_w_ptr}; 
	assign axi_wdata = w_wfifo_rdata; 
	assign axi_wlast = (rc_burst >= C_BURST_LEN - 1); 
	
	assign w_wfifo_ren = axi_wvalid && axi_wready; 
	
	
	assign tp_o[1:0] = {rs_w[1:0]}; 
	assign tp_o[2] = axi_wvalid; 
	assign tp_o[3] = wframe_data_en; 
	
	
	generate
		if(C_W_WIDTH == 64) begin: Gen_FIFO_W64
			W0_FIFO_64 u_W0_FIFO_64 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (capture_wen),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 32) begin: Gen_FIFO_W32
			W0_FIFO_32 u_W0_FIFO_32
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (capture_wen),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 16) begin: Gen_FIFO_W16
			W0_FIFO_16 u_W0_FIFO_16
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (capture_wen),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 8) begin: Gen_FIFO_W8
			W0_FIFO_8 u_W0_FIFO_8 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (capture_wen),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end
	endgenerate




















reg                             rframe_vsync_dly;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rframe_vsync_dly <= 1'b0;
    else
        rframe_vsync_dly <= rframe_vsync;
end

wire                            rframe_vsync_neg;
assign rframe_vsync_neg =  rframe_vsync_dly & ~rframe_vsync;

reg             [4:0]           rfifo_cnt;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rfifo_cnt <= 5'h1f;
    else
    begin
        if(rframe_vsync_neg == 1'b1)
            rfifo_cnt <= 5'h1f;
        else if(rfifo_cnt > 5'd0)
            rfifo_cnt <= rfifo_cnt - 1'b1;
        else
            rfifo_cnt <= 5'b0;
    end
end

reg                             rfifo_rst;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rfifo_rst <= 1'b1;
    else
    begin
        if(rfifo_cnt == 5'b0)
            rfifo_rst <= 1'b0;
        else
            rfifo_rst <= 1'b1;
    end
end


wire 			w_rfifo_rst = axi_reset || rfifo_rst; 

reg                             rfifo_wenb;
reg             [C_DATA_LEN-1:0]          rfifo_wdata;
wire            [ 9:0]          rfifo_wcnt;
wire 					  rfifo_wfull; 
wire                            rfifo_wr_rst_busy;
wire 					w_rfifo_aempty; 
wire 					w_rfifo_empty; 
assign raw_read_empty=w_rfifo_empty || read_fault;


generate
	if(C_R_WIDTH == 8) begin: Gen_RFIFO_8
		R0_FIFO_8 u_R0_FIFO_8
		(
			.wr_clk_i		(axi_clk),
			.wr_en_i		(rfifo_wenb),
			.wdata			(rfifo_wdata),
			.a_rst_i		(w_rfifo_rst), 
			.prog_full_o		(rfifo_wfull),
			
			.rd_clk_i		(rframe_pclk),
			.rd_en_i		(rframe_data_en),
			.rdata			(rframe_data),
			.empty_o		(w_rfifo_empty),
			.almost_empty_o	(w_rfifo_aempty)
		);
	end else if(C_R_WIDTH == 16) begin: Gen_RFIFO_16	
		R0_FIFO_16 u_R0_FIFO_16
		(
			.wr_clk_i		(axi_clk),
			.wr_en_i		(rfifo_wenb),
			.wdata			(rfifo_wdata),
			.a_rst_i		(w_rfifo_rst), 
			.prog_full_o		(rfifo_wfull),
			
			.rd_clk_i		(rframe_pclk),
			.rd_en_i		(rframe_data_en),
			.rdata			(rframe_data),
			.empty_o		(w_rfifo_empty),
			.almost_empty_o	(w_rfifo_aempty)
		);
	end
endgenerate
		
reg r_rfifo_rst = 0; 
always @(posedge axi_clk)
begin
	r_rfifo_rst <= w_rfifo_rst; 
end

//assign tp_o[7:0] = {w_rfifo_empty, rfifo_wfull, rframe_data_en, rfifo_wenb, rfifo_rst, axi_reset, rframe_pclk, axi_clk}; 

reg             [15:0]               rfifo_wr_rst_busy_dly = 2'b0;

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        rfifo_wr_rst_busy_dly <= 0;
    else
        rfifo_wr_rst_busy_dly <= {rfifo_wr_rst_busy_dly,1'b1};
end

wire                                rfifo_wr_rst_busy_neg;
assign rfifo_wr_rst_busy_neg = (rfifo_wr_rst_busy_dly[1:0] == 2'b01) ? 1'b1 : 1'b0;

reg             [4:0]               read_ddr_delay_cnt;

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        read_ddr_delay_cnt <= 5'b0;
    else
    begin
        if(rfifo_wr_rst_busy_neg == 1'b1)
            read_ddr_delay_cnt <= 5'd31;
        else if(read_ddr_delay_cnt > 5'b0)
            read_ddr_delay_cnt <= read_ddr_delay_cnt - 1'b1;
        else
            read_ddr_delay_cnt <= 5'b0;
    end
end

always @(posedge axi_clk or posedge axi_reset) begin
	if(axi_reset)
		r_rframe_inc <= 0; 
	else
		// Keep the old raw/result leases while a previous AXI read drains.
		r_rframe_inc <= rfifo_wr_rst_busy_neg && !read_transaction_busy && !read_fault;
end


wire                                read_ddr_init_flag;
assign read_ddr_init_flag = (read_ddr_delay_cnt == 5'd1) ? 1'b1 : 1'b0;

//----------------------------------------------------------------------




//----------------------------------------------------------------------
localparam S_READ_IDLE = 2'd0;
localparam S_READ_ADDR = 2'd1;
localparam S_READ_DATA = 2'd2;

reg             [ 1:0]          rd_state;
reg             [ 8:0]          rdata_cnt;
reg             [C_BUF_SIZE-1:0]          araddr;
reg 					r_rd_pend = 0; 
reg [31:0] read_address;
reg read_discard,read_fault;
wire read_transaction_busy=rd_state!=S_READ_IDLE || axi_arvalid || r_rd_pend;
wire read_address_accept=axi_arvalid && axi_arready;
wire read_response_owned=r_rd_pend || read_address_accept;
wire read_response_shape_ok=axi_rlast==(rdata_cnt==C_BURST_LEN-1);

// Identity describes complete input content, never the repeating HDMI tick or
// the rotating physical result slot. Metadata is published in vertical blank
// at the first actual read request, after the directory has finished scanning.
reg [15:0] display_token,previous_generation_a,previous_generation_b;
assign adopted_generation_a=previous_generation_a;
reg [10:0] previous_threshold;
reg [2:0] previous_mode;
reg previous_compare,identity_valid;
wire [15:0] selected_generation_a=result_display_active?selected_result_generation_a:read_generation_a;
wire [15:0] selected_generation_b=result_display_active?selected_result_generation_b:read_generation_b;
wire selected_compare=result_display_active?selected_result_compare:history_compare;
wire identity_changed=!identity_valid || selected_generation_a!=previous_generation_a ||
 selected_generation_b!=previous_generation_b || selected_compare!=previous_compare ||
 applied_display_mode!=previous_mode || applied_threshold!=previous_threshold;
always @(posedge axi_clk or posedge axi_reset) begin
 if(axi_reset) begin
  display_token<=0;display_metadata<=0;identity_valid<=0;
  previous_generation_a<=0;previous_generation_b<=0;previous_threshold<=0;previous_mode<=0;previous_compare<=0;
 end else if(rd_state==S_READ_ADDR && araddr==0 && !read_discard && !read_fault && !w_rfifo_rst && !r_rfifo_rst) begin
  if(identity_changed) display_token<=display_token+1'b1;
  display_metadata<={10'd0,active_history_mode,applied_display_mode,1'b1,
                     identity_changed?display_token+16'd1:display_token};
  previous_generation_a<=selected_generation_a;previous_generation_b<=selected_generation_b;
  previous_threshold<=applied_threshold;previous_mode<=applied_display_mode;previous_compare<=selected_compare;
  identity_valid<=1;
 end
end


// Only the shared AXI reset cancels a transaction. Local frame resets discard
// the old response while preserving ARVALID/payload and draining through RLAST.
always @(posedge axi_clk or posedge axi_reset)
begin
    if(axi_reset) begin
        rd_state <= S_READ_IDLE;
	  axi_arvalid <= 0; 
	  r_rd_pend <= 0; 
        read_address<=0;read_discard<=0;read_fault<=0;rdata_cnt<=0;
    end else
    begin
	
	
	
	if(read_address_accept) begin
		axi_arvalid <= 0; 
		r_rd_pend <= 1;
	end else begin
	end
	if(w_rfifo_rst || r_rfifo_rst) read_discard<=1;
	else if(rd_state==S_READ_IDLE) read_discard<=0;
	if(axi_rvalid && axi_rready) begin
          if(!read_response_owned || axi_rresp!=0 || !read_response_shape_ok) read_fault<=1;
          if(read_response_owned) begin
            rdata_cnt<=rdata_cnt+1'b1;
            if(axi_rlast) begin r_rd_pend<=0;rd_state<=S_READ_IDLE;read_discard<=0;end
          end
	end
	
        case(rd_state)
            S_READ_IDLE : 
            begin
			//	Continue if RFIFO ~Full. 
                if(!w_rfifo_rst && !r_rfifo_rst && !read_fault && !axi_rvalid && history_ready && rfifo_wr_rst_busy_dly[15] && (araddr < C_RD_END_ADDR)&&(~rfifo_wfull)) begin
                    rd_state <= S_READ_ADDR;
                    read_address<=(CANNY_ENABLE && result_display_active)?result_display_base+araddr:
                                  C_BASE_ADDR+{selected_read_slot,araddr};
                    rdata_cnt<=0;
                end else
                    rd_state <= S_READ_IDLE;
            end
		
            S_READ_ADDR : 
            begin
				axi_arvalid <= 1; 
                rd_state <= S_READ_DATA;
            end
            S_READ_DATA : 
            begin
                // Accepted RLAST above is the only normal transaction end.
            end
            default : 
            begin
                rd_state <= S_READ_IDLE;
            end
        endcase
    end
end

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        araddr <= 0;
    else
    begin
        if(read_address_accept && !read_discard)
            araddr <= araddr + C_ADDR_INC;
        else
            araddr <= araddr;
    end
end

wire [4:0] selected_read_slot=(HISTORY_ENABLE && history_compare && araddr>=C_RD_END_ADDR/2)?history_rb:rc_rframe_index;
assign axi_araddr = read_address;


always @(posedge axi_clk)
begin
    if(axi_reset == 1'b1)
        rfifo_wenb <= 1'b0;
    else
    begin
        if(axi_rvalid && axi_rready && read_response_owned && !read_discard &&
           !w_rfifo_rst && !r_rfifo_rst && !read_fault && axi_rresp==0 && read_response_shape_ok)
            rfifo_wenb <= 1'b1;
        else
            rfifo_wenb <= 1'b0;
    end
end

always @(posedge axi_clk)
begin
    rfifo_wdata <= axi_rdata;
end

endmodule
