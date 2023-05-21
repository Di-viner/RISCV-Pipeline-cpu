//=====================================================================
// Description:
// As part of the project of Computer Organization Experiments, Wuhan University
// In spring 2022
// The gadgets.
// ====================================================================

`include "xgriscv_defines.v"

// pc register with write enable
module pcenr (
	input             		clk, reset,
	input             		en,
	input      [`XLEN-1:0]	d, 
	output reg [`XLEN-1:0]	q);
 
	always @(posedge clk, posedge reset)
    if (reset) 
    	q <= `ADDR_SIZE'h00000000 ;
    else if (en)    
    	q <=  d;
endmodule

// adder for address calculation
module addr_adder (
	input  [`ADDR_SIZE-1:0] a, b,
	output [`ADDR_SIZE-1:0] y);

	assign  y = a + b;
endmodule

// flop with reset and clear control
module floprc #(parameter WIDTH = 8)
              (input                  clk, reset, clear,
               input      [WIDTH-1:0] d, 
               output reg [WIDTH-1:0] q);

  always @(posedge clk, posedge reset)
    if (reset)      q <= 0;
    else if (clear) q <= 0;
    else            q <= d;
endmodule

// flop with reset, enable and clear control
module flopenrc #(parameter WIDTH = 8)
                 (input                  clk, reset,
                  input                  en, clear,
                  input      [WIDTH-1:0] d, 
                  output reg [WIDTH-1:0] q);
 
  always @(posedge clk, posedge reset)
    if      (reset) q <= 0;
    else if (clear) q <= 0;
    else if (en)    q <= d;
endmodule

// flop with reset and enable control
module flopenr #(parameter WIDTH = 8)
                (input                  clk, reset,
                 input                  en,
                 input      [WIDTH-1:0] d, 
                 output reg [WIDTH-1:0] q);
 
  always @(posedge clk, posedge reset)
    if      (reset) q <= 0;
    else if (en)    q <=  d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, 
              input              s, 
              output [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, d2,
              input  [1:0]       s, 
              output [WIDTH-1:0] y);

  assign  y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module mux4 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, d2, d3,
              input  [1:0]       s, 
              output reg [WIDTH-1:0] y);

	always @( * )
	begin
      case(s)
         2'b00: y <= d0;
         2'b01: y <= d1;
         2'b10: y <= d2;
         2'b11: y <= d3;
      endcase
	end
endmodule

module mux5 #(parameter WIDTH = 8)
             (input		[WIDTH-1:0] d0, d1, d2, d3, d4,
              input		[2:0]       s, 
              output reg	[WIDTH-1:0] y);

	always @( * )
	begin
      case(s)
	    3'b000: y <= d0;
	    3'b001: y <= d1;
	    3'b010: y <= d2;
	    3'b011: y <= d3;
	    3'b100: y <= d4;
      endcase
    end  
endmodule

module mux6 #(parameter WIDTH = 8)
           (input  [WIDTH-1:0] 	d0, d1, d2, d3, d4, d5,
            input  [2:0] 		s,
         	  output reg [WIDTH-1:0]	y);

	always@( * )
	begin
	  case(s)
		3'b000: y <= d0;
		3'b001: y <= d1;
		3'b010: y <= d2;
		3'b011: y <= d3;
		3'b100: y <= d4;
		3'b101: y <= d5;
	  endcase
	end
endmodule

module imm (
	input	[11:0]			iimm, //instr[31:20], 12 bits
	input	[11:0]			simm, //instr[31:25, 11:7], 12 bits
	input	[11:0]			bimm, //instrD[31], instrD[7], instrD[30:25], instrD[11:8], 12 bits
	input	[19:0]			uimm,
	input	[19:0]			jimm,
	input	[4:0]			 immctrl,

	output	reg [`XLEN-1:0] 	immout);
  always  @(*)
	 case (immctrl)
		`IMM_CTRL_ITYPE:	immout <= {{{`XLEN-12}{iimm[11]}}, iimm[11:0]};
    `IMM_CTRL_STYPE:  immout <= {{{`XLEN-12}{simm[11]}}, simm[11:0]};
    `IMM_CTRL_JTYPE:  immout <= {{{`XLEN-21}{jimm[19]}},jimm[19:0],1'b0};
		`IMM_CTRL_UTYPE:	immout <= {uimm[19:0], 12'b0};
    `IMM_CTRL_BTYPE:  immout <= {{{`XLEN-13}{bimm[11]}},bimm[11:0],1'b0};
		default:			      immout <= `XLEN'b0;
	 endcase
endmodule


// comparator for branch
module cmp(
  input [`XLEN-1:0] a, b,
  input             op_unsigned,
  output            zero,
  output            lt);

  assign zero = (a == b);
  assign lt = (!op_unsigned & ($signed(a) < $signed(b))) | (op_unsigned & (a < b));
endmodule

//access memory pattern
module ampattern (input [1:0] addr, input [1:0] whb, output reg [3:0] amp); 
  always@(*)
  case (whb)
    2'b01: amp <= 4'b1111;
    2'b10: if (addr[1]) amp <= 4'b1100;
           else         amp <= 4'b0011; //addr[0]
    2'b11: case (addr)
              2'b00: amp <= 4'b0001;
              2'b01: amp <= 4'b0010;
              2'b10: amp <= 4'b0100;
              2'b11: amp <= 4'b1000;
           endcase
    default: amp <= 4'b1111;// it shouldn't happen
  endcase
endmodule

module data_process2#(parameter WIDTH = 32)(input [3:0] amp, input [`XLEN-1:0] tempData,input lunsigned,output reg [`XLEN-1:0] rd);
  always@(*)
     if(!lunsigned)
      begin
        if(amp == 4'b0001) rd <= {{{`XLEN-8}{tempData[7]}},tempData[7:0]};
        if(amp == 4'b0010) rd <= {{{(`XLEN-8)}{(tempData[15])}},tempData[15:8]};        
        if(amp == 4'b0100) rd <= {{{(`XLEN-8)}{(tempData[23])}},tempData[23:16]};
        if(amp == 4'b1000) rd <= {{{(`XLEN-8)}{(tempData[31])}},tempData[31:24]};
        if(amp == 4'b0011) rd <= {{{(`XLEN-16)}{(tempData[15])}},tempData[15:0]};
        if(amp == 4'b1100) rd <= {{{(`XLEN-16)}{(tempData[31])}},tempData[31:16]};
        if(amp == 4'b1111) rd <= tempData[31:0];

      end
    else if(lunsigned)
      begin
        if(amp == 4'b0001) rd <= {{{`XLEN-8}{1'b0}},tempData[7:0]};
        if(amp == 4'b0010) rd <= {{{`XLEN-8}{1'b0}},tempData[15:8]};
        if(amp == 4'b0100) rd <= {{{(`XLEN-8)}{1'b0}},tempData[23:16]};
        if(amp == 4'b1000) rd <= {{{(`XLEN-8)}{1'b0}},tempData[31:24]};
        if(amp == 4'b0011) rd <= {{{(`XLEN-16)}{1'b0}},tempData[15:0]};
        if(amp == 4'b1100) rd <= {{{(`XLEN-16)}{1'b0}},tempData[31:16]};
        if(amp == 4'b1111) rd <= tempData[31:0];
      end
endmodule

module forwarding_unit(
  input regwriteM,
  input regwriteW,
  input[4:0] rdM,
  input[4:0] rdW,
  input[4:0] rs1E,
  input[4:0] rs2E,
  input[4:0] rs2M,    
  input memwriteM,
  input btypeD,       
  input[4:0] rs1D,
  input[4:0] rs2D,

  output reg[1:0] ForwardAE,  //MEM-->EX, WB-->EX
  output reg[1:0] ForwardBE,
  output wdMsrc,              //WB-->MEM
  output cmpsrca,             //MEM-->ID
  output cmpsrcb       
);

  assign cmpsrca = (btypeD && rdM == rs1D && rdM != 5'b0)? 1'b1: 1'b0;
  assign cmpsrcb = (btypeD && rdM == rs2D && rdM != 5'b0)? 1'b1: 1'b0;
  assign wdMsrc = (rdW == rs2M && memwriteM && regwriteW)? 1'b0: 1'b1;
  always@(*)
  begin
    if(regwriteM && rdM != 5'b0 && rdM == rs1E)       //ex hazard
      ForwardAE <= 2'b10;
    else if(regwriteW && rdW != 5'b0 && rdW == rs1E)  //mem hazard 
      ForwardAE <= 2'b01;
    else
      ForwardAE <= 2'b00;

    if(regwriteM && rdM != 5'b0 && rdM == rs2E)
      ForwardBE <= 2'b10;
    else if(regwriteW && rdW != 5'b0 && rdW == rs2E)
      ForwardBE <= 2'b01;
    else 
      ForwardBE <= 2'b00;
  end
endmodule

module hazard_detection(
  input memtoregE,
  input memtoregM,
  input regwriteE,
  input memwriteD,
  input btypeD,
  input[4:0] rdE,
  input[4:0] rdM,
  input[4:0] rs1D,
  input[4:0] rs2D,

  output reg notStall   //notStall == 0 means a stall in the pipeline
);
always@(*)
begin
  if(memtoregE && rdE != 5'b0 && (rdE == rs1D || rdE == rs2D) && !memwriteD)
    notStall <= 1'b0;   //load-use
  else if(regwriteE && btypeD && rdE != 5'b0 && (rdE == rs1D || rdE == rs2D))
    notStall <= 1'b0;   //arith-beq
  else if(memtoregM && btypeD && rdM != 5'b0 && (rdM == rs1D || rdM == rs2D))
    notStall <= 1'b0;   //load-beq
  else
    notStall <= 1'b1;   //notStall
end
endmodule