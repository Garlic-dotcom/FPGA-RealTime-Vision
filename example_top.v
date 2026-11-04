`timescale 1ns/1ps

//`include "ddr3_controller.vh"


module example_top #(parameter MEDIAN_ENABLE=1, parameter DISPLAY_TEST=0, parameter USE_EDGE_FEATURE=1, parameter SHAPE_ENABLE=1, parameter PROCESS_FROM_DDR=1, parameter CANNY_SELFTEST=1, parameter PRESENTATION_MODE=1)
(
	////////////////////////////////////////////////////////////////
	//	External Clock & Reset
	//input 			nrst, 			//	Button K2
	input 			clk_24m,			//	24MHz Crystal
	input 			clk_25m,			//	25MHz Crystal 
	
	
	////////////////////////////////////////////////////////////////
	//	System Clock
	output 			sys_pll_rstn_o, 		
	
	input 			clk_sys,			//	Sys PLL 96MHz 
	input 			clk_pixel,			//	Sys PLL 74.25MHz
	input 			clk_pixel_2x,		//	Sys PLL 148.5MHz
	input 			clk_pixel_10x,		//	Sys PLL 742.5MHz
	
	input 			sys_pll_lock,		//	Sys PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	MIPI-DSI Clock & Reset
	output 			dsi_pll_rstn_o,
	
	input 			dsi_refclk_i,		//	48MHz Reference Clock (for DSI PLL)
	input 			dsi_byteclk_i,		//	DSI Byte Clock (1X)
	input 			dsi_serclk_i,		//	DSI Serial Clock (4X 45)
	input 			dsi_txcclk_i,		//	DSI Serial Clock (4X 135)
	
	input 			dsi_pll_lock,
	
	////////////////////////////////////////////////////////////////
	//	DDR Clock
	output 			ddr_pll_rstn_o, 
	
	input 			tdqss_clk,			
	input 			core_clk,			//	DDR PLL 200MHz
	input 			tac_clk,			
	input 			twd_clk,			
	
	input 			ddr_pll_lock,		//	DDR PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	DDR PLL Phase Shift Interface
	output 	[2:0] 	shift,
	output 	[4:0] 	shift_sel,
	output 			shift_ena,
	
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Clock
	output 			lvds_pll_rstn_o, 
	
	input 			clk_lvds_1x, 
	input 			clk_lvds_7x, 
	input 			clk_27m, 			//	RGB 1X Clock (16MHz)
	input 			clk_54m, 			//	RGB 2X Clock (32MHz, for export control)
	
	input 			lvds_pll_lock, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DDR Interface Ports
	output 	[15:0] 	addr,
	output 	[2:0] 	ba,
	output 			we,
	output 			reset,
	output 			ras,
	output 			cas,
	output 			odt,
	output 			cke,
	output 			cs,
	
	//	DQ I/O
	input 	[15:0] 	i_dq_hi,
	input 	[15:0] 	i_dq_lo,
	
	output 	[15:0] 	o_dq_hi,
	output 	[15:0] 	o_dq_lo,
	output 	[15:0] 	o_dq_oe,
	
	//	DM O
	output 	[1:0] 	o_dm_hi,
	output 	[1:0] 	o_dm_lo,
	
	//	DQS I/O
	input 	[1:0] 	i_dqs_hi,
	input 	[1:0] 	i_dqs_lo,
	
	input 	[1:0] 	i_dqs_n_hi,
	input 	[1:0] 	i_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_hi,
	output 	[1:0] 	o_dqs_lo,
	
	output 	[1:0] 	o_dqs_n_hi,
	output 	[1:0] 	o_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_oe,
	output 	[1:0] 	o_dqs_n_oe,
	
	//	CK
	output 			clk_p_hi, 
	output 			clk_p_lo, 
	output 			clk_n_hi, 
	output 			clk_n_lo, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Ctl / I2C
	output 			csi_ctl0_o,
	output 			csi_ctl0_oe,
	input 			csi_ctl0_i,
	
	output 			csi_ctl1_o,
	output 			csi_ctl1_oe,
	input 			csi_ctl1_i,
	
	output 			csi_scl_o,
	output 			csi_scl_oe,
	input 			csi_scl_i,
	
	output 			csi_sda_o,
	output 			csi_sda_oe,
	input 			csi_sda_i,
	
	//	MIPI-CSI RXC 
	input 			csi_rxc_lp_p_i,
	input 			csi_rxc_lp_n_i,
	output 			csi_rxc_hs_en_o,
	output 			csi_rxc_hs_term_en_o,
	input 			csi_rxc_i,
	
	//	MIPI-CSI RXD0
	output 			csi_rxd0_rst_o,
	output 			csi_rxd0_hs_en_o,
	output 			csi_rxd0_hs_term_en_o,
	
	input 			csi_rxd0_lp_p_i,
	input 			csi_rxd0_lp_n_i,
	input 	[7:0] 	csi_rxd0_hs_i,
	
	//	MIPI-CSI RXD1
	output 			csi_rxd1_rst_o,
	output 			csi_rxd1_hs_en_o,
	output 			csi_rxd1_hs_term_en_o,
	
	input 			csi_rxd1_lp_n_i,
	input 			csi_rxd1_lp_p_i,
	input 	[7:0] 	csi_rxd1_hs_i,
	
	//	MIPI-CSI RXD2
	output 			csi_rxd2_rst_o,
	output 			csi_rxd2_hs_en_o,
	output 			csi_rxd2_hs_term_en_o,
	
	input 			csi_rxd2_lp_p_i,
	input 			csi_rxd2_lp_n_i,
	input 	[7:0] 	csi_rxd2_hs_i,
	
	//	MIPI-CSI RXD3
	output 			csi_rxd3_rst_o,
	output 			csi_rxd3_hs_en_o,
	output 			csi_rxd3_hs_term_en_o,
	
	input 			csi_rxd3_lp_p_i,
	input 			csi_rxd3_lp_n_i,
	input 	[7:0] 	csi_rxd3_hs_i,
	
	//output 			csi_rxd0_fifo_rd_o, 
	//input 			csi_rxd0_fifo_empty_i, 
	//output 			csi_rxd1_fifo_rd_o, 
	//input 			csi_rxd1_fifo_empty_i, 
	//output 			csi_rxd2_fifo_rd_o, 
	//input 			csi_rxd2_fifo_empty_i, 
	//output 			csi_rxd3_fifo_rd_o, 
	//input 			csi_rxd3_fifo_empty_i, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DSI PWM & Reset Control 
	output 			dsi_pwm_o,			//	MIPI-DSI LCD PWM
	output 			dsi_resetn_o,		//	MIPI-DSI LCD Reset
	
	//	MIPI-DSI TXC / TXD
	output 			dsi_txc_rst_o,
	output 			dsi_txc_lp_p_oe,
	output 			dsi_txc_lp_p_o,
	output 			dsi_txc_lp_n_oe,
	output 			dsi_txc_lp_n_o,
	output 			dsi_txc_hs_oe,
	output 	[7:0] 	dsi_txc_hs_o,
	
	output 			dsi_txd0_rst_o,
	output 			dsi_txd0_hs_oe,
	output 	[7:0] 	dsi_txd0_hs_o,
	output 			dsi_txd0_lp_p_oe,
	output 			dsi_txd0_lp_p_o,
	output 			dsi_txd0_lp_n_oe,
	output 			dsi_txd0_lp_n_o,
	
	output 			dsi_txd1_rst_o,
	output 			dsi_txd1_lp_p_oe,
	output 			dsi_txd1_lp_p_o,
	output 			dsi_txd1_lp_n_oe,
	output 			dsi_txd1_lp_n_o,
	output 			dsi_txd1_hs_oe,
	output 	[7:0] 	dsi_txd1_hs_o,
	
	output 			dsi_txd2_rst_o,
	output 			dsi_txd2_lp_p_oe,
	output 			dsi_txd2_lp_p_o,
	output 			dsi_txd2_lp_n_oe,
	output 			dsi_txd2_lp_n_o,
	output 			dsi_txd2_hs_oe,
	output 	[7:0] 	dsi_txd2_hs_o,
	
	output 			dsi_txd3_rst_o,
	output 			dsi_txd3_lp_p_oe,
	output 			dsi_txd3_lp_p_o,
	output 			dsi_txd3_lp_n_oe,
	output 			dsi_txd3_lp_n_o,
	output 			dsi_txd3_hs_oe,
	output 	[7:0] 	dsi_txd3_hs_o,
	
	input 			dsi_txd0_lp_p_i,
	input 			dsi_txd0_lp_n_i,
	input 			dsi_txd1_lp_p_i,
	input 			dsi_txd1_lp_n_i,
	input 			dsi_txd2_lp_p_i,
	input 			dsi_txd2_lp_n_i,
	input 			dsi_txd3_lp_p_i,
	input 			dsi_txd3_lp_n_i,
	
	
	////////////////////////////////////////////////////////////////
	//	UART Interface
	input 		 	uart_rx_i,			//	Support 460800-8-N-1. 
	output 		 	uart_tx_o, 
	
	
	output 	[5:0] 	led_o,			//	
    input [1:0] key_n, // K1/K2, active low, 1.8 V GPIO inputs
	
	
	////////////////////////////////////////////////////////////////
	//	CMOS Sensor
	output 			cmos_sclk,
	input 			cmos_sdat_IN,
	output 			cmos_sdat_OUT,
	output 			cmos_sdat_OE,
	
	//	CMOS Interface
	input 			cmos_pclk,
	input 			cmos_vsync,
	input 			cmos_href,
	input 	[7:0] 	cmos_data,
	
	output 			cmos_ctl1_o, 
	output 			cmos_ctl1_oe, 
	input 			cmos_ctl1_i,
	output 			cmos_ctl2_o,
	output 			cmos_ctl2_oe, 
	input 			cmos_ctl2_i, 
	output 			cmos_ctl3_o,
	output 			cmos_ctl3_oe, 
	input 			cmos_ctl3_i, 
	
	
	////////////////////////////////////////////////////////////////
	//	HDMI Interface
	output 			hdmi_txc_oe,
	output 			hdmi_txd0_oe,
	output 			hdmi_txd1_oe,
	output 			hdmi_txd2_oe,
	
	output 			hdmi_txc_rst_o,
	output 			hdmi_txd0_rst_o,
	output 			hdmi_txd1_rst_o,
	output 			hdmi_txd2_rst_o,
	
	output 	[9:0] 	hdmi_txc_o,
	output 	[9:0] 	hdmi_txd0_o,
	output 	[9:0] 	hdmi_txd1_o,
	output 	[9:0] 	hdmi_txd2_o,
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Interface
	output 			lvds_txc_oe,
	output 	[6:0] 	lvds_txc_o,
	output 			lvds_txc_rst_o,
	
	output 			lvds_txd0_oe,
	output 	[6:0] 	lvds_txd0_o,
	output 			lvds_txd0_rst_o,
	
	output 			lvds_txd1_oe,
	output 	[6:0] 	lvds_txd1_o,
	output 			lvds_txd1_rst_o,
	
	output 			lvds_txd2_oe,
	output 	[6:0] 	lvds_txd2_o,
	output 			lvds_txd2_rst_o,
	
	output 			lvds_txd3_oe,
	output 	[6:0] 	lvds_txd3_o,
	output 			lvds_txd3_rst_o,
	
	
	////////////////////////////////////////////////////////////////
	//	RGB LCD 5Inch 800x480
	output 			lcd_tp_sda_o,		//	TP SDA
	output 			lcd_tp_sda_oe,
	input 			lcd_tp_sda_i,
	
	output 			lcd_tp_scl_o,		//	TP SCL
	output 			lcd_tp_scl_oe,
	input 			lcd_tp_scl_i,
	
	output 			lcd_tp_int_o,		//	TP INT
	output 			lcd_tp_int_oe,
	input 			lcd_tp_int_i,
	
	output 			lcd_tp_rst_o,		//	TP RST
	
	output 			lcd_pwm_o,			//	Backlight
	output 			lcd_blen_o,
	
	//output 			lcd_pclk_o,			//	PCLK & SCK Mux
	output 			lcd_vs_o,			//	VS & SSN Mux. Fixed to 1. Use DE-Only mode. 
	output 			lcd_hs_o,			//	HS. Fixed to 1. Use DE-Only mode. 

	output 	[7:0] 	lcd_b7_0_o,			//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_o,			//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_o,			//	R7:R0. 
	
	output 	[7:0] 	lcd_b7_0_oe,		//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_oe,		//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_oe,		//	R7:R0. 

	input 	[7:0] 	lcd_b7_0_i,			//	B7:B0. 
	input 	[7:0] 	lcd_g7_0_i,			//	G7:G0. Must output 8'hFF when access SPI. 
	input 	[7:0] 	lcd_r7_0_i,			//	R7:R0. 
	
	//	SPI Pins
	output 			spi_sck_o, 
	output 			spi_ssn_o 			
);
	
	wire 			csi_rxd0_fifo_rd_o; 
	wire 			csi_rxd0_fifo_empty_i = 0;  
	wire 			csi_rxd1_fifo_rd_o;  
	wire 			csi_rxd1_fifo_empty_i = 0;  
	wire 			csi_rxd2_fifo_rd_o;  
	wire 			csi_rxd2_fifo_empty_i = 0;  
	wire 			csi_rxd3_fifo_rd_o;  
	wire 			csi_rxd3_fifo_empty_i = 0;  
	
	
	parameter 	SIM_DATA 	= 0; 
	
	//	Hardware Configuration
	assign clk_p_hi = 1'b0;	//	DDR3 Clock requires 180 degree shifted. 
	assign clk_p_lo = 1'b1;
	assign clk_n_hi = 1'b1;
	assign clk_n_lo = 1'b0; 
	
	assign cmos_ctl1_o = 0; 
	assign cmos_ctl1_oe = 1; 
	assign cmos_ctl2_o = 0; 
	assign cmos_ctl2_oe = 1; 
	assign cmos_ctl3_o = 0; 
	assign cmos_ctl3_oe = 1; 
	
	//	System Clock Tree Control
	assign sys_pll_rstn_o = 1'b1; 	//	nrst; 	//	Reset whole system when nrst (K2) is pressed. 
	
	assign dsi_pll_rstn_o = sys_pll_lock; 
	assign ddr_pll_rstn_o = sys_pll_lock; 
	assign lvds_pll_rstn_o = sys_pll_lock; 
	
	wire 			w_pll_lock = sys_pll_lock && dsi_pll_lock && ddr_pll_lock && lvds_pll_lock; 
	
	//	Synchronize System Resets. 
	reg 			rstn_sys = 0, rstn_pixel = 0; 
	wire 			rst_sys = ~rstn_sys, rst_pixel = ~rstn_pixel; 
	
	reg 			rstn_dsi_refclk = 0, rstn_dsi_byteclk = 0; 
	wire 			rst_dsi_refclk = ~rstn_dsi_refclk, rst_dsi_byteclk = ~rstn_dsi_byteclk; 
	
	reg 			rstn_lvds_1x = 0; 
	wire 			rst_lvds_1x = ~rstn_lvds_1x; 
	
	reg 			rstn_27m = 0, rstn_54m = 0; 
	wire 			rst_27m = ~rstn_27m, rst_54m = ~rstn_54m; 
	
	//	Clock Gen
	always @(posedge clk_27m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_27m <= 0; else rstn_27m <= 1; end
	always @(posedge clk_54m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_54m <= 0; else rstn_54m <= 1; end
	always @(posedge clk_sys or negedge w_pll_lock) begin if(~w_pll_lock) rstn_sys <= 0; else rstn_sys <= 1; end
	always @(posedge clk_pixel or negedge w_pll_lock) begin if(~w_pll_lock) rstn_pixel <= 0; else rstn_pixel <= 1; end
	always @(posedge dsi_refclk_i or negedge w_pll_lock) begin if(~w_pll_lock) rstn_dsi_refclk <= 0; else rstn_dsi_refclk <= 1; end
	always @(posedge dsi_byteclk_i or negedge w_pll_lock) begin if(~w_pll_lock) rstn_dsi_byteclk <= 0; else rstn_dsi_byteclk <= 1; end
	always @(posedge clk_lvds_1x or negedge w_pll_lock) begin if(~w_pll_lock) rstn_lvds_1x <= 0; else rstn_lvds_1x <= 1; end
	
	
	localparam 	CLOCK_MAIN 	= 96000000; 	//	System clock using 96MHz. 
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Flash Burner Control
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 			w_ustick, w_mstick; 
	
	wire  [7:0] 	w_dev_index_o;  
	wire  [7:0] 	w_dev_cmd_o;  
	wire  [31:0] 	w_dev_wdata_o;  
	wire  		w_dev_wvalid_o;  
	wire  		w_dev_rvalid_o;  
	wire 	[31:0] 	w_dev_rdata_i;  
	
	wire 			w_spi_ssn_o, w_spi_sck_o; 
	wire 	[3:0] 	w_spi_data_o, w_spi_data_oe; 
	wire 	[3:0] 	w_spi_data_i; 
	
	//	Flash Control
	reg 			r_flash_en = 0; 		//	0x00:0x00 Enable Flash
	
	always @(posedge clk_sys) begin
		r_flash_en <= (w_dev_wvalid_o && (w_dev_index_o == 8'h00) && (w_dev_cmd_o == 8'h00)) ? w_dev_wdata_o : r_flash_en; 
	end
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	LCD Data Mux
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 	[7:0] 	w_lcd_b_o, w_lcd_g_o, w_lcd_r_o; 
	
	assign lcd_b7_0_o = r_flash_en ? {4'b0, w_spi_data_o[3:2], 2'b0} : w_lcd_b_o; 
	assign lcd_g7_0_o = r_flash_en ? {6'h0, w_spi_data_o[1:0]} : w_lcd_g_o; 
	assign lcd_r7_0_o = r_flash_en ? {8'h00} : w_lcd_r_o; 
	
	assign lcd_b7_0_oe = r_flash_en ? {4'b0, w_spi_data_oe[3:2], 2'b0} : 8'hFF; 
	assign lcd_g7_0_oe = r_flash_en ? {6'h0, w_spi_data_oe[1:0]} : 8'hFF; 
	assign lcd_r7_0_oe = r_flash_en ? {8'h00} : 8'hFF; 
	
	assign spi_sck_o = w_spi_sck_o; 
	assign spi_ssn_o = w_spi_ssn_o; 
	assign w_spi_data_i = {lcd_b7_0_i[3:2], lcd_g7_0_i[1:0]}; 
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	DDR3 Controller
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	wire			w_ddr3_ui_clk = clk_sys;
	wire			w_ddr3_ui_rst = rst_sys;
	wire			w_ddr3_ui_areset = rst_sys;
	wire			w_ddr3_ui_aresetn = rstn_sys;
	

	//	General AXI Interface 
	wire	[3:0] 	w_ddr3_awid;
	wire	[31:0]	w_ddr3_awaddr;
	wire	[7:0]		w_ddr3_awlen;
	wire			w_ddr3_awvalid;
	wire			w_ddr3_awready;
	
	wire 	[3:0]  	w_ddr3_wid;
	wire 	[127:0] 	w_ddr3_wdata;
	wire 	[15:0]	w_ddr3_wstrb;
	wire			w_ddr3_wlast;
	wire			w_ddr3_wvalid;
	wire			w_ddr3_wready;
	
	wire 	[3:0] 	w_ddr3_bid;
	wire 	[1:0] 	w_ddr3_bresp;
	wire			w_ddr3_bvalid;
	wire			w_ddr3_bready;
	
	wire	[3:0] 	w_ddr3_arid;
	wire	[31:0]	w_ddr3_araddr;
	wire	[7:0]		w_ddr3_arlen;
	wire			w_ddr3_arvalid;
	wire			w_ddr3_arready;
	
	wire 	[3:0] 	w_ddr3_rid;
	wire 	[127:0] 	w_ddr3_rdata;
	wire			w_ddr3_rlast;
	wire			w_ddr3_rvalid;
	wire			w_ddr3_rready;
	wire 	[1:0] 	w_ddr3_rresp;
	
	
	//	AXI Interface Request
	wire 	[3:0] 	w_ddr3_aid;
	wire 	[31:0] 	w_ddr3_aaddr;
	wire 	[7:0]  	w_ddr3_alen;
	wire 	[2:0]  	w_ddr3_asize;
	wire 	[1:0]  	w_ddr3_aburst;
	wire 	[1:0]  	w_ddr3_alock;
	wire			w_ddr3_avalid;
	wire			w_ddr3_aready;
	wire			w_ddr3_atype;
	
	wire 			w_ddr3_cal_done, w_ddr3_cal_pass; 
	
	//	Do not issue DDR read / write when ~cal_done. 
	reg 			r_ddr_unlock = 0; 
	always @(posedge w_ddr3_ui_clk or negedge w_ddr3_ui_aresetn) begin
		if(~w_ddr3_ui_aresetn)
			r_ddr_unlock <= 0; 
		else
			r_ddr_unlock <= w_ddr3_cal_done; 
	end
	
	DdrCtrl ddr3_ctl_axi (	
		.core_clk		(core_clk),
		.tac_clk		(tac_clk),
		.twd_clk		(twd_clk),	
		.tdqss_clk		(tdqss_clk),
		
		.reset		(reset),
		.cs			(cs),
		.ras			(ras),
		.cas			(cas),
		.we			(we),
		.cke			(cke),    
		.addr			(addr),
		.ba			(ba),
		.odt			(odt),
		
		.o_dm_hi		(o_dm_hi),
		.o_dm_lo		(o_dm_lo),
		
		.i_dq_hi		(i_dq_hi),
		.i_dq_lo		(i_dq_lo),
		.o_dq_hi		(o_dq_hi),
		.o_dq_lo		(o_dq_lo),
		.o_dq_oe		(o_dq_oe),
		
		.i_dqs_hi		(i_dqs_hi),
		.i_dqs_lo		(i_dqs_lo),
		.i_dqs_n_hi		(i_dqs_n_hi),
		.i_dqs_n_lo		(i_dqs_n_lo),
		.o_dqs_hi		(o_dqs_hi),
		.o_dqs_lo		(o_dqs_lo),
		.o_dqs_n_hi		(o_dqs_n_hi),
		.o_dqs_n_lo		(o_dqs_n_lo),
		.o_dqs_oe		(o_dqs_oe),
		.o_dqs_n_oe		(o_dqs_n_oe),
		
		.clk			(w_ddr3_ui_clk),
		.reset_n		(w_ddr3_ui_aresetn),
		
		.axi_avalid		(w_ddr3_avalid),	//	Enable command only when unlocked. 
		.axi_aready		(w_ddr3_aready),
		.axi_aaddr		(w_ddr3_aaddr),
		.axi_aid		(w_ddr3_aid),
		.axi_alen		(w_ddr3_alen),
		.axi_asize		(w_ddr3_asize),
		.axi_aburst		(w_ddr3_aburst),
		.axi_alock		(w_ddr3_alock),
		.axi_atype		(w_ddr3_atype),
		
		.axi_wid		(w_ddr3_wid),
		.axi_wvalid		(w_ddr3_wvalid),
		.axi_wready		(w_ddr3_wready),
		.axi_wdata		(w_ddr3_wdata),
		.axi_wstrb		(w_ddr3_wstrb),
		.axi_wlast		(w_ddr3_wlast),
		
		.axi_bvalid		(w_ddr3_bvalid),
		.axi_bready		(w_ddr3_bready),
		.axi_bid		(w_ddr3_bid),
		.axi_bresp		(w_ddr3_bresp),
		
		.axi_rvalid		(w_ddr3_rvalid),
		.axi_rready		(w_ddr3_rready),
		.axi_rdata		(w_ddr3_rdata),
		.axi_rid		(w_ddr3_rid),
		.axi_rresp		(w_ddr3_rresp),
		.axi_rlast		(w_ddr3_rlast),
		
		.shift		(shift),
		.shift_sel		(),
		.shift_ena		(shift_ena),
		
		.cal_ena		(1'b1),
		.cal_done		(w_ddr3_cal_done),
		.cal_pass		(w_ddr3_cal_pass)
	);
	
	// B readiness now comes from the shared-port owner, held until completion.
	assign shift_sel = 5'b00100; 		//	ddr_tac_clk always use PLLOUT[2]. 
	
	
	AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) axi4_awar_mux (
		.aclk_i			(w_ddr3_ui_clk), 
		.arst_i			(w_ddr3_ui_rst), 
		
		.awid_i			(w_ddr3_awid),
		.awaddr_i			(w_ddr3_awaddr),
		.awlen_i			(w_ddr3_awlen),
		.awvalid_i			(w_ddr3_awvalid && r_ddr_unlock),
		.awready_o			(w_ddr3_awready),
		
		.arid_i			(w_ddr3_arid),
		.araddr_i			(w_ddr3_araddr),
		.arlen_i			(w_ddr3_arlen),
		.arvalid_i			(w_ddr3_arvalid && r_ddr_unlock),
		.arready_o			(w_ddr3_arready),
		
		.aid_o			(w_ddr3_aid),
		.aaddr_o			(w_ddr3_aaddr),
		.alen_o			(w_ddr3_alen),
		.atype_o			(w_ddr3_atype),
		.avalid_o			(w_ddr3_avalid),
		.aready_i			(w_ddr3_aready)
	);
	
	assign w_ddr3_asize = 4; 		//	Fixed 128 bits (16 bytes, size = 4)
	assign w_ddr3_aburst = 1; 
	assign w_ddr3_alock = 0; 
	
	//assign led_o[1:0] = {w_ddr3_cal_pass, w_ddr3_cal_done}; 
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	I2C Config (OV5640)
	
	//  i2c timing controller module of 16Bit
	wire            [ 7:0]          ov5640_i2c_config_index;
	wire            [23:0]          ov5640_i2c_config_data;
	wire            [ 7:0]          ov5640_i2c_config_size;
	wire                            ov5640_i2c_config_done;
	wire            [15:0]          ov5640_i2c_rdata;                              //  i2c register data

	i2c_timing_ctrl_16bit
	#(
	    .CLK_FREQ           (CLOCK_MAIN),                              //  100 MHz
	    .I2C_FREQ           (50_000    )                               //  10 KHz(<= 400KHz)
	) 
	u_i2c_timing_ctrl_16bit 
	(
	    //global clock
	    .clk                (clk_sys                 ),                          //  96MHz
	    .rst_n              (rstn_sys                ),                          //  system reset

	    //i2c interface
	    .i2c_sclk           (cmos_sclk                  ),                  //  i2c clock
		.i2c_sdat_OUT 	(cmos_sdat_OUT), 
		.i2c_sdat_OE	(cmos_sdat_OE), 
		.i2c_sdat_IN 	(cmos_sdat_IN), 
		
	    //i2c config data
	    .i2c_config_index   (ov5640_i2c_config_index           ),                  //  i2c config reg index, read 2 reg and write xx reg
	    .i2c_config_data    ({8'h78, ov5640_i2c_config_data}   ),                  //  i2c config data
	    .i2c_config_size    (ov5640_i2c_config_size            ),                  //  i2c config data counte
	    .i2c_config_done    (ov5640_i2c_config_done            ),                  //  i2c config timing complete
	    .i2c_rdata          (ov5640_i2c_rdata                  )                   //  i2c register data while read i2c slave
	);
	
	//----------------------------------------------------------------------
	//  I2C Configure Data of OV5640
	I2C_OV5640_1280720_Config u_I2C_OV5640_1280720_Config
	(
	    .LUT_INDEX  (ov5640_i2c_config_index   ),
	    .LUT_DATA   (ov5640_i2c_config_data    ),
	    .LUT_SIZE   (ov5640_i2c_config_size    )
	); 
	
	
	//	CMOS Interface
	//input 			cmos_pclk,
	//input 			cmos_vsync,
	//input 			cmos_href,
	//input 	[7:0] 	cmos_data,
	
	wire 			w_cmos_pclk = cmos_pclk; 
	
	
	
	//	Output LED
	reg 	[3:0]		r_cmos_fv_o = 0; 
	reg 	[1:0] 	r_cmos_rx_vsync0_in = 0; 
	always @(posedge w_cmos_pclk) begin
		r_cmos_rx_vsync0_in <= {r_cmos_rx_vsync0_in, cmos_vsync}; 
		r_cmos_fv_o <= r_cmos_fv_o + ((r_cmos_rx_vsync0_in == 2'b01) ? 1 : 0); 
	end
	assign led_o[5] = r_cmos_fv_o[3]; 
	
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	System Control. Can be removed for public. 
	
	localparam 	CLK_FREQ 	= 96_000_000; 	//	clk_sys is 96MHz. 
	localparam 	BAUD_RATE 	= 460_800; 		//	Use 460800-8-N-1. 
	
	
	//	SFR I/O Interface
	wire 	[7:0] 	w_sfr_addr_o; 	//	SFR Address (0xFF00 ~ 0xFFFF). 00:Power; 40~5F:Stream0; 60~7F:Stream1. 
	wire 	[7:0] 	w_sfr_wdata_o; 	//	SFR Write Data. 
	wire 			w_sfr_we_o; 		//	SFR WE. 
	reg 	[7:0] 	w_sfr_rdata_i; 	//	Must be valid after sfr_rd_o. 
	wire 			w_sfr_rd_o; 		//	SFR RD. 
	
	
	//	System Control Registers
	reg 			r_dsi_tx_rstn = 0; 	//	DSI TX Reset
	reg 	[7:0] 	r_dsi_pwm = 64; 		//	[6:0]PWM, [7]Pol
	reg 			r_dsi_resetn_o = 0; 	//	DSI Panel Reset
	reg 			r_dsi_data_rstn = 0; 	//	DSI TX Reset
	
	reg 	[3:0] 	r_dsi_lp_p_ovr = 0; 	
	reg 	[3:0] 	r_dsi_lp_n_ovr = 0; 	
	
	
	
	//	AXI-Lite Interface Bridge
	localparam 	CSI_AXILITE_ID 	= 0; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	localparam 	DSI_AXILITE_ID 	= 1; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	
	reg 	[7:0] 	r_axi_addr = 8'h18; 		//	0xE0 (RW)
	reg 	[31:0] 	r_axi_wdata = 32'h0000000A; 	//	0xE1~0xE4 (RW)
	wire 	[31:0] 	w_axi_rdata; 			//	0xE5~0xE8 (RO)
	reg 	[0:0] 	r_axi_sel = 1; 			//	0xE9[7:2] (RW)
	reg 			r_axi_r1w0 = 0; 			//	0xE9[1] (RW)
	reg 			r_axi_req = 0; 			//	0xE9[0] (WO, Single Cycle)
	reg 			r_axi_req_o = 0; 			//	Delayed of r_axi_req. 
	
	
	//	Buffered AXI Read Data
	reg 	[31:0] 	r_axi_rdata = 0; 		//	Use state machine
	reg 			r_axi_idle = 0; 		//	AXI Idle 
	
	reg 	[3:0] 	rs_axilite = 0; 		//	AXI Access
	wire 	[3:0] 	ws_axilite_idle = 0; 		
	wire 	[3:0] 	ws_axilite_write = 1; 
	wire 	[3:0] 	ws_axilite_read = 2; 
	wire 	[3:0] 	ws_axilite_endread = 3; 
	
	reg 			r_axi_awvalid = 0, r_axi_wvalid = 0, r_axi_arvalid = 0; 
	wire 			w_axi_awready, w_axi_wready, w_axi_arready, w_axi_rvalid; 
	
	
	reg 	[3:0] 	rc_axi_init = 0; 

	always @(posedge clk_sys or posedge rst_sys) begin
		if(rst_sys) begin
			r_dsi_tx_rstn <= 0; 
			r_dsi_pwm <= 64; 
			r_axi_req <= 0; 
			r_dsi_resetn_o <= 0; 
			r_dsi_data_rstn <= 0; 
			
			rs_axilite <= 0; 
			r_axi_awvalid <= 0; 
			r_axi_wvalid <= 0;
			r_axi_arvalid <= 0; 
			
			r_dsi_lp_p_ovr <= 0; 
			r_dsi_lp_n_ovr <= 0; 
			
			rc_axi_init <= 0; 
			r_axi_idle <= 0; 
			
		end else begin
			r_dsi_tx_rstn <= 1; 
			r_dsi_resetn_o <= 1; 
			r_dsi_data_rstn <= 1; 
			
		end
	end
	
	assign dsi_resetn_o = r_dsi_resetn_o; 
	
	assign csi_ctl0_oe = 0; 
	assign csi_ctl1_oe = 0; 
	
	
	PWMLite dsi_pwm (		//	#(.ENABLE_TICK(0), .PWM_BITS(7)) 
		.clk_i			(clk_sys),
		.rst_i			(rst_sys),
		.pwm_i			(r_dsi_pwm[6:0]),
		.pol_i			(r_dsi_pwm[7]), 
		.pwm_o			(dsi_pwm_o)
	);






	////////////////////////////////////////////////////////////////
	//	MIPI CSI RX
	
	//	The CSI RXC shall not be inverted. Data can be inverted with swapped LP data and flipped HS data. 
	localparam 	CSI_RXD_INV 	= 4'b1111; 
	localparam 	CSI_DATA_WIDTH 	= 8; 			
	localparam 	CSI_STRB_WIDTH 	= CSI_DATA_WIDTH / 8; 

	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Crop
	
	wire			XYCrop_frame_vsync; 
	wire			XYCrop_frame_href;
	wire			XYCrop_frame_de;
	wire	[63:0]	XYCrop_frame_Gray;

	Sensor_Image_XYCrop
	#(
		//	RGB width doubled. 
		.IMAGE_HSIZE_SOURCE (1280 * 2 / CSI_STRB_WIDTH),
		.IMAGE_VSIZE_SOURCE (720	 ),
		.IMAGE_HSIZE_TARGET (1280 * 2 / CSI_STRB_WIDTH),
		.IMAGE_YSIZE_TARGET (720 	 ),
		.PIXEL_DATA_WIDTH	(CSI_DATA_WIDTH) 		//	32		 )
	)
	u_Sensor_Image_XYCrop
	(
		//	globel clock
		.clk			(w_cmos_pclk),			//	image pixel clock
		.rst_n		(rstn_sys),			//	system reset
		
		//CMOS Sensor interface
		.image_in_vsync (cmos_vsync		),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_in_href	(cmos_href		),			//H : Data vaild, L : Line Sync
		.image_in_de	(cmos_href		), 			//H : Data Enable, L : Line Sync
		.image_in_data	(cmos_data),			//8 bits cmos data input
		
		.image_out_vsync(XYCrop_frame_vsync ),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_out_href (XYCrop_frame_href	),			//H : Data vaild, L : Line Sync
		.image_out_de	(XYCrop_frame_de	), 			//H : Data Enable, L : Line Sync
		.image_out_data (XYCrop_frame_Gray	)			//8 bits cmos data input	
	);
//摄像头采集模块，dvp转avalon-st裸流
// Assert asynchronously, release synchronously in the camera processing domain.
reg [1:0] camera_reset_pipe = 2'b00;
always @(posedge w_cmos_pclk or negedge rstn_sys) begin
    if(!rstn_sys) camera_reset_pipe <= 2'b00;
    else camera_reset_pipe <= {camera_reset_pipe[0],1'b1};
end
wire camera_rst_n = camera_reset_pipe[1];
wire [15:0] data_w0;
wire valid_w0,ready_w0,sop_w0,eop_w0;	
cmos_top u_cmos_top_0( 
		.clk         (w_cmos_pclk),
		.rst_n       (camera_rst_n),
		.cmos_pclk   (w_cmos_pclk ),       //  cmos.pclk
		.cmos_vsync  (cmos_vsync),         //      .vsync
		.cmos_href   (cmos_href ),         //      .href
		.cmos_data   (cmos_data ),         //      .data
		.source_sop  (sop_w0    ),
		.source_valid(valid_w0  ),
		.source_data (data_w0   ),
		.source_eop  (eop_w0    ),
		.source_ready(1'b1      )
);
//RGB转灰度：cmos_top输出 -> 灰度8bit
wire        gray_sop, gray_valid, gray_eop;
wire [7:0]  gray_data;
vip_rgb2gray_proc #(.BPS(8)) u_vip_rgb2gray_proc (
	.clk         (w_cmos_pclk),
	.rst_n       (camera_rst_n),
	.sink_sop    (sop_w0    ),
	.sink_valid  (valid_w0  ),
	.sink_data   ({data_w0[15:11],3'b111,data_w0[10:5],2'b11,data_w0[4:0],3'b111}   ),
	.sink_eop    (eop_w0    ),
	.source_sop  (gray_sop  ),
	.source_valid(gray_valid),
	.source_data (gray_data ),
	.source_eop  (gray_eop  )
);
// Keep the legacy path available as a compile-time comparison. The archived
// frame-resync release contains the original constraints and legacy defaults.
wire feature_valid,feature_sop,feature_eop;
wire [15:0] feature_rgb565;
wire sobel_valid,sobel_sop,sobel_eop;
wire [7:0] sobel_data;
generate if(USE_EDGE_FEATURE && !PROCESS_FROM_DDR) begin: g_edge_feature
    wire [10:0] threshold_sys,threshold_camera;
    wire [2:0] mode_sys,mode_camera;
    wire [15:0] rgb565_aligned;
    video_buttons #(.MODE_COUNT(SHAPE_ENABLE?7:6)) u_video_buttons(.clk(clk_sys),.rst_n(rstn_sys),.key_n(key_n),
        .threshold(threshold_sys),.mode(mode_sys));
    video_control_cdc u_video_control_cdc(
        .source_clk(clk_sys),.source_rst_n(rstn_sys),.threshold_in(canny_applied_threshold),.mode_in(canny_applied_mode),
        .dest_clk(w_cmos_pclk),.dest_rst_n(camera_rst_n),.threshold_out(threshold_camera),.mode_out(mode_camera));
    vip_rgb2gray_shift_regs #(.DWIDTH(16),.DELAY_DUTY(7)) u_rgb_context_delay(
        .clk(w_cmos_pclk),.rst_n(camera_rst_n),.idata(data_w0),.odata(rgb565_aligned));
    edge_video_pipeline #(.WIDTH(1280),.HEIGHT(720),.MEDIAN_ENABLE(MEDIAN_ENABLE),.SHAPE_ENABLE(SHAPE_ENABLE)) u_edge_video_pipeline(
        .clk(w_cmos_pclk),.rst_n(camera_rst_n),.sink_valid(gray_valid),.sink_sop(gray_sop),.sink_eop(gray_eop),
        .sink_gray(gray_data),.sink_rgb565(rgb565_aligned),.threshold(threshold_camera),.mode(mode_camera),
        .source_valid(feature_valid),.source_sop(feature_sop),.source_eop(feature_eop),.source_rgb565(feature_rgb565));
    assign sobel_valid=0;assign sobel_sop=0;assign sobel_eop=0;assign sobel_data=0;
end else if(!PROCESS_FROM_DDR) begin: g_legacy_edge
    wire median_sop,median_eop,median_valid;
    wire [7:0] median_data;
    median3x3 #(.WIDTH(1280),.HEIGHT(720),.FILTER_ENABLE(MEDIAN_ENABLE)) u_median3x3(
        .clk(w_cmos_pclk),.rst_n(camera_rst_n),.sink_sop(gray_sop),.sink_eop(gray_eop),
        .sink_valid(gray_valid),.sink_data(gray_data),.source_sop(median_sop),.source_eop(median_eop),
        .source_valid(median_valid),.source_data(median_data));
    Sobeledge_8d_proc #(.WIDTH(1280),.HEIGHT(720)) u_Sobeledge_8d_proc(
        .clk(w_cmos_pclk),.rst_n(camera_rst_n),.edge_en(1'b1),.bin_en(1'b0),
        .sink_sop(median_sop),.sink_eop(median_eop),.sink_valid(median_valid),.sink_data(median_data),
        .source_sop(sobel_sop),.source_eop(sobel_eop),.source_valid(sobel_valid),.source_data(sobel_data));
    assign feature_valid=0;assign feature_sop=0;assign feature_eop=0;assign feature_rgb565=0;
end else begin: g_raw_capture_only
    assign feature_valid=0;assign feature_sop=0;assign feature_eop=0;assign feature_rgb565=0;
    assign sobel_valid=0;assign sobel_sop=0;assign sobel_eop=0;assign sobel_data=0;
end endgenerate
// DISPLAY_TEST 1/2 bypass both algorithm paths and retain the diagnostic format.
wire [7:0] probe_data;
capture_display_probe #(.WIDTH(1280),.HEIGHT(720)) u_capture_display_probe(
    .clk(w_cmos_pclk),.rst_n(camera_rst_n),.cmos_vsync(cmos_vsync),.cmos_href(cmos_href),
    .gray_valid(gray_valid),.gray_sop(gray_sop),.pattern_data(probe_data));
wire diagnostic_mode=(DISPLAY_TEST==1 || DISPLAY_TEST==2);
wire display_valid=diagnostic_mode ? gray_valid : USE_EDGE_FEATURE ? feature_valid : sobel_valid;
wire display_sop=diagnostic_mode ? gray_sop : USE_EDGE_FEATURE ? feature_sop : sobel_sop;
wire display_eop=diagnostic_mode ? gray_eop : USE_EDGE_FEATURE ? feature_eop : sobel_eop;
wire [15:0] display_data=(DISPLAY_TEST==1)?{8'd0,gray_data}:
                         (DISPLAY_TEST==2)?{8'd0,probe_data}:
                         USE_EDGE_FEATURE ? feature_rgb565 : {8'd0,sobel_data};
wire processed_frame_level;
wire raw_frame_level;
reg [15:0] camera_completed;
always @(posedge w_cmos_pclk or negedge camera_rst_n) begin
 if(!camera_rst_n) camera_completed<=0;
 else if(valid_w0 && eop_w0) camera_completed<=camera_completed+1'b1;
end
wire [15:0] camera_completed_sys;
history_status_cdc camera_counter_mailbox(
 .source_clk(w_cmos_pclk),.source_rst_n(camera_rst_n),.data_in(camera_completed),
 .dest_clk(clk_sys),.dest_rst_n(rstn_sys),.data_out(camera_completed_sys));
wire [7:0] display_fps;
wire [31:0] display_metadata;
wire raw_read_empty;
wire [15:0] process_generation_a,process_generation_b,result_generation_a,result_generation_b;
wire result_compare;
video_frame_level u_raw_frame_level (
    .clk(w_cmos_pclk),.rst_n(camera_rst_n),
    .pixel_valid(valid_w0),.pixel_sop(sop_w0),.pixel_eop(eop_w0),
    .frame_level(raw_frame_level)
);
video_frame_level u_processed_frame_level (
    .clk(w_cmos_pclk),.rst_n(camera_rst_n),
    .pixel_valid(display_valid),.pixel_sop(display_sop),.pixel_eop(display_eop),
    .frame_level(processed_frame_level)
);
	reg			r_XYCrop_frame_vsync = 0; 
	reg			r_XYCrop_frame_href = 0;
	reg			r_XYCrop_frame_de = 0;
	reg	[63:0]	r_XYCrop_frame_Gray = 0;
	
	always @(posedge w_cmos_pclk) begin
		r_XYCrop_frame_vsync <= XYCrop_frame_vsync; 
		r_XYCrop_frame_href <= XYCrop_frame_href;
		r_XYCrop_frame_de <= XYCrop_frame_de;
		r_XYCrop_frame_Gray <= XYCrop_frame_Gray;
	end
	
	//	Data Write Assignment
	wire			cmos_frame_vsync = r_XYCrop_frame_vsync;                     //  cmos frame data vsync valid signal
	wire			cmos_frame_href = r_XYCrop_frame_href && r_XYCrop_frame_de;	 //  cmos frame data href vaild  signal
	wire	[63:0]	cmos_frame_Gray = r_XYCrop_frame_Gray; 
	wire 			cmos_vsync_end;

	












	////////////////////////////////////////////////////////////////
	//	DDR R/W Control
	

	wire                            lcd_de;
	wire                            lcd_hs;      
	wire                            lcd_vs;
	wire                            ddr_read_vs;
	wire 					  lcd_request; 
	wire            [7:0]           lcd_red, lcd_red2;
	wire            [7:0]           lcd_green, lcd_green2;
	wire            [7:0]           lcd_blue, lcd_blue2;
	wire            [15:0]          lcd_data;


	assign w_ddr3_awid = 0; 
	assign w_ddr3_wid = 0; 
	
	wire 			w_wframe_vsync; 
	wire 	[7:0] 	w_axi_tp; 
	
	//	Write in 8 bits. Read in 16 bits. 
	wire [1:0] history_mode;
    wire history_older,history_newer;
    wire [15:0] history_status;
    wire [31:0] video_awaddr,video_araddr;
    wire [7:0] video_awlen,video_arlen;
    wire video_awvalid,video_awready,video_arvalid,video_arready;
    wire [127:0] video_wdata,video_rdata;
    wire [15:0] video_wstrb;
    wire video_wlast,video_wvalid,video_wready,video_bvalid,video_rlast,video_rvalid,video_rready;
    wire [1:0] video_bresp;
    wire ddr_test_done,ddr_test_failed;
    wire [2:0] ddr_test_index;
    wire [31:0] ddr_test_cycles;

    wire [10:0] canny_threshold,canny_applied_threshold,canny_result_threshold;
    wire [2:0] canny_mode,canny_applied_mode;
    wire result_contour;wire [404:0] result_shapes,display_shapes,shapes_pixel_cdc;
    wire canny_request,canny_release,canny_pin,canny_compare,canny_result_valid,canny_display_active,canny_fault,canny_busy;
    wire [4:0] canny_a,canny_b;
    wire [31:0] canny_result_base,canny_display_base,canny_completed,canny_total,canny_input,canny_algorithm,canny_hys;
    wire [31:0] canny_ar_wait,canny_read_wait,canny_write_wait;
    wire [15:0] canny_adopted_generation;
 wire [31:0] merged_awaddr;
 wire [7:0] merged_awlen;
 wire [2:0] merged_awsize;
 wire [1:0] merged_awburst;
 wire [0:0] merged_awvalid;
 wire [0:0] merged_awready;
 wire [127:0] merged_wdata;
 wire [15:0] merged_wstrb;
 wire [0:0] merged_wlast;
 wire [0:0] merged_wvalid;
 wire [0:0] merged_wready;
 wire [1:0] merged_bresp;
 wire [0:0] merged_bvalid;
 wire [0:0] merged_bready;
 wire [31:0] merged_araddr;
 wire [7:0] merged_arlen;
 wire [2:0] merged_arsize;
 wire [1:0] merged_arburst;
 wire [0:0] merged_arvalid;
 wire [0:0] merged_arready;
 wire [127:0] merged_rdata;
 wire [1:0] merged_rresp;
 wire [0:0] merged_rlast;
 wire [0:0] merged_rvalid;
 wire [0:0] merged_rready;
    canny_video_port canny_video(
      .clk(clk_sys),.rst_n(rstn_sys),.enable(PROCESS_FROM_DDR && ddr_test_done && !ddr_test_failed && (canny_mode==7 || canny_mode==6)),.contour_mode(canny_mode==6),.result_contour(result_contour),.result_shapes(result_shapes),
      .live_mode(history_mode==2'd0 && history_status[13:12]==2'd0),
      .threshold(canny_threshold),.process_request(canny_request),.process_release(canny_release),
      .process_valid(canny_pin),.process_compare(canny_compare),.process_a(canny_a),.process_b(canny_b),
      .process_generation_a(process_generation_a),.process_generation_b(process_generation_b),
      .result_generation_a(result_generation_a),.result_generation_b(result_generation_b),.result_compare(result_compare),
      .display_active(canny_display_active),.display_base(canny_display_base),
      .result_valid(canny_result_valid),.result_base(canny_result_base),.result_threshold(canny_result_threshold),
      .fault(canny_fault),.busy(canny_busy),.completed_frames(canny_completed),.last_total_cycles(canny_total),
      .last_input_cycles(canny_input),.last_algorithm_cycles(canny_algorithm),.last_hysteresis_cycles(canny_hys),
      .last_ar_wait_cycles(canny_ar_wait),.last_read_wait_cycles(canny_read_wait),.last_write_wait_cycles(canny_write_wait),
      .video_awaddr(video_awaddr),.ddr_awaddr(merged_awaddr),
      .video_awlen(video_awlen),.ddr_awlen(merged_awlen),
      .video_awsize(3'd4),.ddr_awsize(merged_awsize),
      .video_awburst(2'd1),.ddr_awburst(merged_awburst),
      .video_awvalid(video_awvalid),.ddr_awvalid(merged_awvalid),
      .video_awready(video_awready),.ddr_awready(merged_awready),
      .video_wdata(video_wdata),.ddr_wdata(merged_wdata),
      .video_wstrb(video_wstrb),.ddr_wstrb(merged_wstrb),
      .video_wlast(video_wlast),.ddr_wlast(merged_wlast),
      .video_wvalid(video_wvalid),.ddr_wvalid(merged_wvalid),
      .video_wready(video_wready),.ddr_wready(merged_wready),
      .video_bresp(video_bresp),.ddr_bresp(merged_bresp),
      .video_bvalid(video_bvalid),.ddr_bvalid(merged_bvalid),
      .video_bready(1'b1),.ddr_bready(merged_bready),
      .video_araddr(video_araddr),.ddr_araddr(merged_araddr),
      .video_arlen(video_arlen),.ddr_arlen(merged_arlen),
      .video_arsize(3'd4),.ddr_arsize(merged_arsize),
      .video_arburst(2'd1),.ddr_arburst(merged_arburst),
      .video_arvalid(video_arvalid),.ddr_arvalid(merged_arvalid),
      .video_arready(video_arready),.ddr_arready(merged_arready),
      .video_rdata(video_rdata),.ddr_rdata(merged_rdata),
      .video_rresp(),.ddr_rresp(merged_rresp),
      .video_rlast(video_rlast),.ddr_rlast(merged_rlast),
      .video_rvalid(video_rvalid),.ddr_rvalid(merged_rvalid),
      .video_rready(video_rready),.ddr_rready(merged_rready));
    canny_shared_ddr_port shared_port(
        .clk(w_ddr3_ui_clk),.rst_n(rstn_sys),.enable(w_ddr3_cal_done && w_ddr3_cal_pass && ov5640_i2c_config_done),
        .done(ddr_test_done),.failed(ddr_test_failed),.test_index(ddr_test_index),.total_cycles(ddr_test_cycles),
        .video_awaddr(merged_awaddr),.video_araddr(merged_araddr),.video_awlen(merged_awlen),.video_arlen(merged_arlen),
        .video_awsize(merged_awsize),.video_arsize(merged_arsize),.video_awburst(merged_awburst),.video_arburst(merged_arburst),
        .video_awvalid(merged_awvalid),.video_awready(merged_awready),.video_arvalid(merged_arvalid),.video_arready(merged_arready),
        .video_wdata(merged_wdata),.video_wstrb(merged_wstrb),.video_wlast(merged_wlast),.video_wvalid(merged_wvalid),.video_wready(merged_wready),
        .video_bresp(merged_bresp),.video_bvalid(merged_bvalid),.video_bready(merged_bready),
        .video_rdata(merged_rdata),.video_rresp(merged_rresp),.video_rlast(merged_rlast),.video_rvalid(merged_rvalid),.video_rready(merged_rready),
        .ddr_awaddr(w_ddr3_awaddr),.ddr_araddr(w_ddr3_araddr),.ddr_awlen(w_ddr3_awlen),.ddr_arlen(w_ddr3_arlen),
        .ddr_awsize(),.ddr_arsize(),.ddr_awburst(),.ddr_arburst(),
        .ddr_awvalid(w_ddr3_awvalid),.ddr_awready(w_ddr3_awready),.ddr_arvalid(w_ddr3_arvalid),.ddr_arready(w_ddr3_arready),
        .ddr_wdata(w_ddr3_wdata),.ddr_wstrb(w_ddr3_wstrb),.ddr_wlast(w_ddr3_wlast),.ddr_wvalid(w_ddr3_wvalid),.ddr_wready(w_ddr3_wready),
        .ddr_bresp(w_ddr3_bresp),.ddr_bvalid(w_ddr3_bvalid),.ddr_bready(w_ddr3_bready),
        .ddr_rdata(w_ddr3_rdata),.ddr_rresp(w_ddr3_rresp),.ddr_rlast(w_ddr3_rlast),.ddr_rvalid(w_ddr3_rvalid),.ddr_rready(w_ddr3_rready));
	axi4_ctrl #(.HISTORY_ENABLE(PROCESS_FROM_DDR),.CANNY_ENABLE(PROCESS_FROM_DDR),.C_RD_END_ADDR(1280 * 2 * 720), .C_W_WIDTH(16), .C_R_WIDTH(16), .C_ID_LEN(4)) u_axi4_ctrl (

		.axi_clk        (w_ddr3_ui_clk            ),
		.axi_reset      (w_ddr3_ui_rst            ),

		.axi_awaddr     (video_awaddr       ),
		.axi_awlen      (video_awlen        ),
		.axi_awvalid    (video_awvalid      ),
		.axi_awready    (video_awready      ),

		.axi_wdata      (video_wdata        ),
		.axi_wstrb      (video_wstrb        ),
		.axi_wlast      (video_wlast        ),
		.axi_wvalid     (video_wvalid       ),
		.axi_wready     (video_wready       ),

		.axi_bid        (0          ),
		.axi_bresp      (video_bresp),
		.axi_bvalid     (video_bvalid),

		.axi_arid       (w_ddr3_arid         ),
		.axi_araddr     (video_araddr       ),
		.axi_arlen      (video_arlen        ),
		.axi_arvalid    (video_arvalid      ),
		.axi_arready    (video_arready      ),

		.axi_rid        (w_ddr3_rid          ),
		.axi_rdata      (video_rdata        ),
		.axi_rresp      (0        ),
		.axi_rlast      (video_rlast        ),
		.axi_rvalid     (video_rvalid       ),
		.axi_rready     (video_rready       ),

		.wframe_pclk    (w_cmos_pclk          ),
		.wframe_vsync   (PROCESS_FROM_DDR ? raw_frame_level : processed_frame_level),
		.wframe_data_en (PROCESS_FROM_DDR ? valid_w0 : display_valid),
		.wframe_data    (PROCESS_FROM_DDR ? data_w0 : display_data),
		
		.rframe_pclk    (clk_pixel            ),
		.rframe_vsync   (~ddr_read_vs        ), // Read timing must precede processed HDMI timing.
		.rframe_data_en (lcd_request             ),
		.rframe_data    (lcd_data           ),
        .raw_read_empty(raw_read_empty),
		
		.history_mode(history_mode),.history_older(history_older),.history_newer(history_newer),
        .history_status(history_status),
        .process_request(canny_request),.process_release(canny_release),.process_valid(canny_pin),
        .process_a(canny_a),.process_b(canny_b),.process_compare(canny_compare),
        .process_generation_a(process_generation_a),.process_generation_b(process_generation_b),
        .requested_display_mode(canny_mode),.requested_threshold(canny_threshold),
        .result_valid(canny_result_valid),.result_base(canny_result_base),.result_threshold(canny_result_threshold),
        .result_contour(result_contour),.result_shapes(result_shapes),.display_shapes(display_shapes),
        .result_generation_a(result_generation_a),.result_generation_b(result_generation_b),.result_compare(result_compare),
        .display_metadata(display_metadata),
        .adopted_generation_a(canny_adopted_generation),
        .result_display_active(canny_display_active),.result_display_base(canny_display_base),
        .applied_display_mode(canny_applied_mode),.applied_threshold(canny_applied_threshold),
		.tp_o 		(w_axi_tp)
	);
	assign led_o[3:0] = w_axi_tp; 
	
	
	
	
	////////////////////////////////////////////////////////////////
	//  LCD Timing Driver
	wire 	[15:0] 	w_lcd_data_swap = {lcd_data[7:0], lcd_data[15:8]}; 
	wire 	[7:0] 	lcd_data_r = (USE_EDGE_FEATURE && !diagnostic_mode) ? {lcd_data[15:11],lcd_data[15:13]} : lcd_data[7:0];
	wire 	[7:0] 	lcd_data_g = (USE_EDGE_FEATURE && !diagnostic_mode) ? {lcd_data[10:5],lcd_data[10:9]} : lcd_data[7:0];
	wire 	[7:0] 	lcd_data_b = (USE_EDGE_FEATURE && !diagnostic_mode) ? {lcd_data[4:0],lcd_data[4:2]} : lcd_data[7:0];
	
	generate if(PROCESS_FROM_DDR) begin: g_ddr_read_processing
        wire [10:0] threshold_sys,threshold_pixel;
        wire [2:0] mode_sys,mode_pixel;
        wire processing_fault;
        wire filter_gaussian_sys,filter_gaussian_pixel,color_direction_sys,color_direction_pixel;
        history_buttons #(.MODE_COUNT(8),.FILTER_PREVIEW_ENABLE(1),.COLOR_STYLE_ENABLE(1)) buttons(
            .clk(clk_sys),.rst_n(rstn_sys),.key_n(key_n),.threshold(canny_threshold),.mode(canny_mode),
            .history_mode(history_mode),.older(history_older),.newer(history_newer),.filter_gaussian(filter_gaussian_sys),.color_direction(color_direction_sys));
        video_control_cdc controls(
            .source_clk(clk_sys),.source_rst_n(rstn_sys),.threshold_in(canny_applied_threshold),.mode_in(canny_applied_mode),
            .dest_clk(clk_pixel),.dest_rst_n(rstn_pixel),.threshold_out(threshold_pixel),.mode_out(mode_pixel));
        history_status_cdc #(.WIDTH(1)) filter_mailbox(
            .source_clk(clk_sys),.source_rst_n(rstn_sys),.data_in(filter_gaussian_sys),
            .dest_clk(clk_pixel),.dest_rst_n(rstn_pixel),.data_out(filter_gaussian_pixel));
        history_status_cdc #(.WIDTH(1)) color_mailbox(
            .source_clk(clk_sys),.source_rst_n(rstn_sys),.data_in(color_direction_sys),
            .dest_clk(clk_pixel),.dest_rst_n(rstn_pixel),.data_out(color_direction_pixel));
        history_status_cdc #(.WIDTH(405)) shapes_mailbox(
            .source_clk(clk_sys),.source_rst_n(rstn_sys),.data_in(display_shapes),
            .dest_clk(clk_pixel),.dest_rst_n(rstn_pixel),.data_out(shapes_pixel_cdc));
        wire [15:0] history_pixel;
        wire [31:0] metadata_pixel;
        history_status_cdc #(.WIDTH(32)) metadata_mailbox(
            .source_clk(clk_sys),.source_rst_n(rstn_sys),.data_in(display_metadata),
            .dest_clk(clk_pixel),.dest_rst_n(rstn_pixel),.data_out(metadata_pixel));
        history_status_cdc status_mailbox(.source_clk(clk_sys),.source_rst_n(rstn_sys),.data_in(history_status),
            .dest_clk(clk_pixel),.dest_rst_n(rstn_pixel),.data_out(history_pixel));
        ddr_read_edge_display #(.HISTORY_OSD(1),.MEDIAN_ENABLE(MEDIAN_ENABLE),.SHAPE_ENABLE(SHAPE_ENABLE),.FILTER_PREVIEW_ENABLE(1),.EXTERNAL_SHAPES(1),.STRENGTH_COLOR_ENABLE(1),.RGB_STROKE_ENABLE(1)) display(
            .clk(clk_pixel),.rst_n(rstn_pixel),.raw_rgb565(lcd_data),.history_status(history_pixel),
            .raw_empty(raw_read_empty),
            .threshold(threshold_pixel),.mode(mode_pixel),.filter_gaussian(filter_gaussian_pixel),.color_direction(color_direction_pixel),.contour_shapes(shapes_pixel_cdc),.raw_request(lcd_request),.raw_vsync(ddr_read_vs),
            .frame_metadata(metadata_pixel),.display_fps(display_fps),
            .video_hsync(lcd_hs),.video_vsync(lcd_vs),.video_de(lcd_de),
            .video_rgb({lcd_red,lcd_green,lcd_blue}),.fault(processing_fault));
        assign {lcd_red2,lcd_green2,lcd_blue2}=24'd0;
    end else begin: g_original_display
        assign display_fps=0;
        assign canny_threshold=160;assign canny_mode=0;
        assign history_mode=0;assign history_older=0;assign history_newer=0;
        assign ddr_read_vs=lcd_vs;
	lcd_driver u_lcd_driver
	(
	    //  global clock
	    .clk        (clk_pixel   ),
	    .rst_n      (rstn_pixel), 
	    
	    //  lcd interface
	    .lcd_dclk   (               ),
	    .lcd_blank  (               ),
	    .lcd_sync   (               ),
	    .lcd_request(lcd_request    ), 	//	Request data 1 cycle ahead. 
	    .lcd_hs     (lcd_hs         ),
	    .lcd_vs     (lcd_vs         ),
	    .lcd_en     (lcd_de         ),
	    .lcd_rgb    ({lcd_red2,lcd_green2,lcd_blue2, lcd_red,lcd_green,lcd_blue}),
	    
	    //  user interface
	    .lcd_data   ({lcd_data_r, lcd_data_g, lcd_data_b}  )
	);
	end endgenerate
	
	
	
	////////////////////////////////////////////////////////////////
	//	HDMI Interface. 
	
	//	HDMI requires specific timing, thus is not compatible with LCD & LVDS & DSI. Must implement standalone. 
	
	assign hdmi_txd0_rst_o = rst_pixel; 
	assign hdmi_txd1_rst_o = rst_pixel; 
	assign hdmi_txd2_rst_o = rst_pixel; 
	assign hdmi_txc_rst_o = rst_pixel; 
	
	assign hdmi_txd0_oe = 1'b1; 
	assign hdmi_txd1_oe = 1'b1; 
	assign hdmi_txd2_oe = 1'b1; 
	assign hdmi_txc_oe = 1'b1; 
	
	//-------------------------------------
	//Digilent HDMI-TX IP Modified by CB elec.
    wire [23:0] hdmi_video_rgb,diagnostic_video_rgb,camera_debug_rgb,canny_debug_rgb;
    generate if(PRESENTATION_MODE) begin: g_presentation
        assign hdmi_video_rgb=diagnostic_video_rgb;
    end else begin: g_debug_overlays
    camera_timing_overlay camera_measurement(
        .sys_clk(clk_sys),.sys_rst_n(rstn_sys),.pixel_clk(clk_pixel),.pixel_rst_n(rstn_pixel),
        .camera_vsync(cmos_vsync),.video_de(lcd_de),.video_vsync(lcd_vs),
        .video_rgb(diagnostic_video_rgb),.output_rgb(camera_debug_rgb));
    canny_runtime_overlay canny_measurement(
        .sys_clk(clk_sys),.sys_rst_n(rstn_sys),.pixel_clk(clk_pixel),.pixel_rst_n(rstn_pixel),
        .enabled(canny_mode==7),.busy(canny_busy),.failed(canny_fault),.result_valid(canny_result_valid),
        .completed_frames(canny_completed),.input_cycles(canny_input),.hysteresis_cycles(canny_hys),
        .displayed_fps(display_fps),.captured_frames(camera_completed_sys),
        .video_de(lcd_de),.video_vsync(lcd_vs),
        .video_rgb(camera_debug_rgb),.output_rgb(canny_debug_rgb));
    canny_task_observer_overlay canny_observer(
        .sys_clk(clk_sys),.sys_rst_n(rstn_sys),.pixel_clk(clk_pixel),.pixel_rst_n(rstn_pixel),
        .enabled(canny_mode==7),.completed_frames(canny_completed),
        .job_generation(result_generation_a),.adopted_generation(canny_adopted_generation),
        .ar_wait_cycles(canny_ar_wait),.read_wait_cycles(canny_read_wait),.write_wait_cycles(canny_write_wait),
        .video_de(lcd_de),.video_vsync(lcd_vs),.video_rgb(canny_debug_rgb),.output_rgb(hdmi_video_rgb));
    end endgenerate
    generate if(CANNY_SELFTEST) begin: g_canny_selftest
        canny_selftest_overlay #(.HIDE_SUCCESS(PRESENTATION_MODE)) diagnostic(
            .ddr_done(ddr_test_done),.ddr_failed(ddr_test_failed),
            .sys_clk(clk_sys),.sys_rst_n(rstn_sys),.pixel_clk(clk_pixel),.pixel_rst_n(rstn_pixel),
            .video_de(lcd_de),.video_vsync(lcd_vs),.video_rgb({lcd_red,lcd_green,lcd_blue}),.output_rgb(diagnostic_video_rgb));
    end else begin: g_no_canny_selftest
        assign diagnostic_video_rgb={lcd_red,lcd_green,lcd_blue};
    end endgenerate
	rgb2dvi #(.ENABLE_OSERDES(0)) u_rgb2dvi
	(
		//.TMDS_Clk_p		(hdmio_txc_p_o), 	//	w_hdmio_txc), 
		//.TMDS_Clk_n		(hdmio_txc_n_o), 
		//.TMDS_Data_p	(hdmio_txd_p_o), 	//	w_hdmio_txd), 
		//.TMDS_Data_n 	(hdmio_txd_n_o), 
		
		.oe_i 		(1), 			//	Always enable output
		.bitflip_i 		(4'b0000), 		//	Reverse clock & data lanes. 
		
		.aRst			(1'b0), 
		.aRst_n		(1'b1), 
		
		.PixelClk		(clk_pixel        ),//pixel clk = 74.25M
		.SerialClk		(     ),//pixel clk *5 = 371.25M
		
		.vid_pVSync		(lcd_vs), 
		.vid_pHSync		(lcd_hs), 
		.vid_pVDE		(lcd_de), 
		.vid_pData		(hdmi_video_rgb),
		
		.txc_o			(hdmi_txc_o), 
		.txd0_o			(hdmi_txd0_o), 
		.txd1_o			(hdmi_txd1_o), 
		.txd2_o			(hdmi_txd2_o)
	); 
		
	






	
	
endmodule


