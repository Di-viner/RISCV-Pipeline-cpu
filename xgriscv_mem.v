//=====================================================================
// Description:
// As part of the project of Computer Organization Experiments, Wuhan University
// In spring 2022
// The instruction memory and data memory.
// ====================================================================

`include "xgriscv_defines.v"

module imem(input  [`ADDR_SIZE-1:0]   a,    //pc
            output [`INSTR_SIZE-1:0]  rd);  //read-data

  reg  [`INSTR_SIZE-1:0] RAM[`IMEM_SIZE-1:0];

  initial
    begin
      //$readmemh("riscv32_sim1.dat", RAM);
    end
  //input address output read-data
  assign rd = RAM[a[11:2]]; // instruction size aligned
endmodule


module dmem(input           	          clk, we, //colck,write-enable
            input  [`XLEN-1:0]          a, wd,    //address, write-data
            input  [`ADDR_SIZE-1:0] 	  pc,
            input [1:0]                 whbM,
            input                       lunsigned,
            output [`XLEN-1:0]          rd);      //read-data

  reg  [31:0] RAM[1023:0];

  assign rd = RAM[a[11:2]]; // word aligned
  wire[3:0] amp;
  ampattern access_memory_pattern(a[1:0], whbM, amp);
  wire [`XLEN-1:0] tempData;
  assign tempData = RAM[a[11:2]];
  always @(posedge clk)
    if (we)
      begin
        if(amp == 4'b0001) RAM[a[11:2]] = {tempData[31:8], wd[7:0]};
        if(amp == 4'b0010) RAM[a[11:2]] = {tempData[31:16],wd[7:0],tempData[7:0]};
        if(amp == 4'b0100) RAM[a[11:2]] = {tempData[31:24],wd[7:0],tempData[15:0]};
        if(amp == 4'b1000) RAM[a[11:2]] = {wd[7:0],tempData[23:0]};
        if(amp == 4'b0011) RAM[a[11:2]] = {tempData[31:16],wd[15:0]};
        if(amp == 4'b1100) RAM[a[11:2]] = {wd[15:0],tempData[15:0]};
        if(amp == 4'b1111) RAM[a[11:2]] = wd;
        // DO NOT CHANGE THIS display LINE!!!
        /**********************************************************************/
        //$display("pc = %h: dataaddr = %h, memdata = %h", pc, {a[31:2],2'b00}, RAM[a[11:2]]);
        /**********************************************************************/
  	  end
    
endmodule

