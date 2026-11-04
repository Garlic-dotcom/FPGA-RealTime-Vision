// Nine-input odd/even compare-exchange network. Nine parallel phases are
// grouped into five register stages (2+2+2+2+1). One result per clock.
module median9_pipeline #(parameter FILTER_ENABLE=1)(
 input clk,rst_n,sink_valid,sink_sop,sink_eop,
 input [71:0] sink_pixels,
 output source_valid,source_sop,source_eop,
 output [7:0] source_data,
 output [7:0] source_gaussian
);
 wire [71:0] phase_data[0:9];
 assign phase_data[0]=sink_pixels;
 genvar phase,lane;
 generate for(phase=1;phase<=9;phase=phase+1) begin: sort_phase
   wire [71:0] exchanged;
   for(lane=0;lane<9;lane=lane+1) begin: compare_lane
     if((lane%2)==((phase-1)%2) && lane<8) begin: low
       wire [7:0] a=phase_data[phase-1][71-lane*8 -: 8];
       wire [7:0] b=phase_data[phase-1][71-(lane+1)*8 -: 8];
       assign exchanged[71-lane*8 -: 8]=(a<=b)?a:b;
     end else if(lane>0 && ((lane-1)%2)==((phase-1)%2)) begin: high
       wire [7:0] a=phase_data[phase-1][71-(lane-1)*8 -: 8];
       wire [7:0] b=phase_data[phase-1][71-lane*8 -: 8];
       assign exchanged[71-lane*8 -: 8]=(a>=b)?a:b;
     end else begin: unpaired
       assign exchanged[71-lane*8 -: 8]=phase_data[phase-1][71-lane*8 -: 8];
     end
   end
   if(phase%2==0 || phase==9) begin: registered_phase
     reg [71:0] data_reg;
     always @(posedge clk or negedge rst_n)
       if(!rst_n) data_reg<=0; else data_reg<=exchanged;
     assign phase_data[phase]=data_reg;
   end else begin: combinational_phase
     assign phase_data[phase]=exchanged;
   end
 end endgenerate
 reg [4:0] valid_pipe,sop_pipe,eop_pipe;
 reg [7:0] center_pipe[0:4];
 // A second filter shares the SAME input window. This is a parallel
 // Gaussian alternative, not a Gaussian applied to the median result.
 // [1 2 1; 2 4 2; 1 2 1]/16, replicated borders supplied by the caller.
 // Two arithmetic stages plus three delays match the five median stages.
 reg [9:0] gaussian_row0,gaussian_row1,gaussian_row2;
 reg [11:0] gaussian_sum;
 reg [7:0] gaussian_pipe[0:2];
 integer i;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     valid_pipe<=0;sop_pipe<=0;eop_pipe<=0;
     for(i=0;i<5;i=i+1) center_pipe[i]<=0;
     gaussian_row0<=0;gaussian_row1<=0;gaussian_row2<=0;gaussian_sum<=0;
     for(i=0;i<3;i=i+1) gaussian_pipe[i]<=0;
   end else begin
     valid_pipe<={valid_pipe[3:0],sink_valid};
     sop_pipe<={sop_pipe[3:0],sink_sop && sink_valid};
     eop_pipe<={eop_pipe[3:0],sink_eop && sink_valid};
     center_pipe[0]<=sink_pixels[39:32];
     for(i=1;i<5;i=i+1) center_pipe[i]<=center_pipe[i-1];
     gaussian_row0<={2'b0,sink_pixels[71:64]}+{1'b0,sink_pixels[63:56],1'b0}+{2'b0,sink_pixels[55:48]};
     gaussian_row1<={2'b0,sink_pixels[47:40]}+{1'b0,sink_pixels[39:32],1'b0}+{2'b0,sink_pixels[31:24]};
     gaussian_row2<={2'b0,sink_pixels[23:16]}+{1'b0,sink_pixels[15:8],1'b0}+{2'b0,sink_pixels[7:0]};
     gaussian_sum<={2'b0,gaussian_row0}+{1'b0,gaussian_row1,1'b0}+{2'b0,gaussian_row2};
     gaussian_pipe[0]<=gaussian_sum[11:4];
     gaussian_pipe[1]<=gaussian_pipe[0];gaussian_pipe[2]<=gaussian_pipe[1];
   end
 end
 assign source_valid=valid_pipe[4];
 assign source_sop=sop_pipe[4];
 assign source_eop=eop_pipe[4];
 assign source_data=FILTER_ENABLE ? phase_data[9][39:32] : center_pipe[4];
 assign source_gaussian=gaussian_pipe[2];
endmodule
