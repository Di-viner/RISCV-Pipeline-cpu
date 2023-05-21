//=====================================================================
// Description:
// As part of the project of Computer Organization Experiments, Wuhan University
// In spring 2022
// The overall of the pipelined xg-riscv implementation.
// ====================================================================

`include "xgriscv_defines.v"

module xgriscv_pipeline(
  input                   clk, reset,
  output[`ADDR_SIZE-1:0]  pcW);
  
  wire [31:0]    instr;
  wire [31:0]    pcF, pcM;
  wire           memwrite;
  wire [3:0]     amp;
  wire [31:0]    addr, writedata, readdata;
  wire [1:0]     whbM;
  wire           lunsignedM;
  //instruction memory
  imem U_imem(pcF, instr);
  //data memory
  dmem U_dmem(clk, memwrite, addr, writedata, pcM, whbM, lunsignedM ,readdata);
  
  xgriscv U_xgriscv(clk, reset, pcF, instr, memwrite, amp, addr, writedata, pcM, pcW, readdata, whbM, lunsignedM, btypeD);
  
endmodule

// xgriscv: a pipelined riscv processor
module xgriscv(input         			        clk, reset,
               output [31:0] 			        pcF,
               input  [`INSTR_SIZE-1:0]   instr,
               output					            memwrite,
               output [3:0]  			        amp,
               output [`ADDR_SIZE-1:0] 	  daddr, 
               output [`XLEN-1:0] 		    writedata,
               output [`ADDR_SIZE-1:0] 	  pcM,
               output [`ADDR_SIZE-1:0] 	  pcW,
               input  [`XLEN-1:0] 		    readdata,
               output [1:0]               whbM,
               output                     lunsignedM,
               output                     btypeD);
	
  wire [6:0]  opD;
 	wire [2:0]  funct3D;
	wire [6:0]  funct7D;
  wire [4:0]  rdD, rs1D;
  wire [11:0] immD;
  wire        zeroD, ltD;
  wire [4:0]  immctrlD;
  wire        itypeD, jalD, jalrD, bunsignedD, pcsrcD;
  wire [3:0]  aluctrlD;
  wire [1:0]  alusrcaD;
  wire        alusrcbD;
  wire        memwriteD, lunsignedD;
  wire [1:0]  swhbD, lwhbD;
  wire        memtoregD, regwriteD;
  
  controller  c(clk, reset, opD, funct3D, funct7D, rdD, rs1D, immD, zeroD, ltD,
              immctrlD, itypeD, jalD, jalrD, bunsignedD, pcsrcD, 
              aluctrlD, alusrcaD, alusrcbD, 
              memwriteD, lunsignedD, lwhbD, swhbD, 
              memtoregD, regwriteD, btypeD);

  
  datapath    dp(clk, reset,
              instr, pcF,
              readdata, daddr, writedata, memwrite, pcM, pcW,
              immctrlD, itypeD, jalD, jalrD, bunsignedD, pcsrcD, 
              aluctrlD, alusrcaD, alusrcbD, 
              memwriteD, lunsignedD, lwhbD, swhbD, 
              memtoregD, regwriteD, 
              opD, funct3D, funct7D, rdD, rs1D, immD, zeroD, ltD,
              whbM, lunsignedM, btypeD);

endmodule
