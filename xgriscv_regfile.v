//=====================================================================
// Description:
// As part of the project of Computer Organization Experiments, Wuhan University
// In spring 2022
// The regfile module implements the core's general purpose registers file.
// ====================================================================

`include "xgriscv_defines.v"

module regfile(
  input                      	clk,        
  input  [`RFIDX_WIDTH-1:0]  	ra1, ra2,   //read-address
  output [`XLEN-1:0]          rd1, rd2,   //read-data

  input                      	we3,        //write-enable
  input  [`RFIDX_WIDTH-1:0]  	wa3,        //write-address
  input  [`XLEN-1:0]          wd3,        //write-data
  
  input  [`ADDR_SIZE-1:0] 	   pc         //pc's value
  );

  reg [`XLEN-1:0] rf[`RFREG_NUM-1:0];

  // three ported register file
  // read two ports combinationally
  // write third port on falling edge of clock
  // register 0 hardwired to 0

  always @(negedge clk)
    if (we3 && wa3!=0)
      begin
        rf[wa3] <= wd3;
        // DO NOT CHANGE THIS display LINE!!!
        /**********************************************************************/
        $display("pc = %h: x%d = %h", pc, wa3, wd3);
        /**********************************************************************/
      end

  assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule
