//=====================================================================
// Description:
// As part of the project of Computer Organization Experiments, Wuhan University
// In spring 2022
// The datapath of the pipeline.
// ====================================================================

`include "xgriscv_defines.v"

module datapath(
	input                    clk, reset,

	input [`INSTR_SIZE-1:0]  instrF, 	 	// from instructon memory
	output[`ADDR_SIZE-1:0] 	 pcF, 		   	// to instruction memory

	input [`XLEN-1:0]	     readdataM, 	// from data memory: read data
  	output[`XLEN-1:0]        aluoutM, 	 	// to data memory: address
 	output[`XLEN-1:0]	     writedataM,	// to data memory: write data
  	output			         memwriteM,		// to data memory: write enable
 	output [`ADDR_SIZE-1:0]  pcM,       	// to data memory: pc of the write instruction
 	
 	output [`ADDR_SIZE-1:0]  pcW,       	// to testbench
  
	
	// from controller
	input [4:0]		            immctrlD,
	input			            itype, jalD, jalrD, bunsignedD, pcsrcD,
	input [3:0]		            aluctrlD,
	input [1:0]		            alusrcaD,
	input			            alusrcbD,
	input			            memwriteD, lunsignedD,
	input [1:0]		          	lwhbD, swhbD,  
	input          		        memtoregD, regwriteD,
	
  	// to controller
	output [6:0]		        opD,
	output [2:0]		        funct3D,
	output [6:0]		        funct7D,
	output [4:0] 		        rdD, rs1D,
	output [11:0]  		        immD,
	output 	       		        zeroD, ltD,

	output [1:0] 				whbM,
	output 						lunsignedM,
	input 						btypeD
	);
	wire pcsrcE;
	wire notStall;	///notStall == 0 means a stall in the pipeline

	// next PC logic (operates in fetch and decode)
	wire [`ADDR_SIZE-1:0]	 pcplus4F, nextpcF, pcbranchE;
	mux2 #(`ADDR_SIZE)	    pcsrcmux(pcplus4F, pcbranchE, pcsrcE, nextpcF);

	// Fetch stage logic
	pcenr      	 	pcreg(clk, reset, notStall, nextpcF, pcF);
	addr_adder  	pcadder1(pcF, `ADDR_SIZE'b100, pcplus4F);

	
	// IF/ID pipeline registers
	wire [`INSTR_SIZE-1:0]	instrD;
	wire [`ADDR_SIZE-1:0]	pcD, pcplus4D;
	wire [1:0] 				whbD = swhbD | lwhbD;
	wire flushD = pcsrcE;

	flopenrc #(`INSTR_SIZE) 	pr1D(clk, reset, notStall ,flushD, instrF, instrD);     // instruction
	flopenrc #(`ADDR_SIZE)	  	pr2D(clk, reset, notStall, flushD, pcF, pcD);           // pc
	flopenrc #(`ADDR_SIZE)	  	pr3D(clk, reset, notStall, flushD, pcplus4F, pcplus4D); // pc+4

	// Decode stage logic
	wire [`RFIDX_WIDTH-1:0] rs2D;
	assign  opD 	= instrD[6:0];
	assign  rdD     = instrD[11:7];
	assign  funct3D = instrD[14:12];
	assign  rs1D    = instrD[19:15];
	assign  rs2D   	= instrD[24:20];
	assign  funct7D = instrD[31:25];
	assign  immD    = instrD[31:20];
  
	// immediate generate
	wire [11:0]  iimmD  = instrD[31:20];
	wire [11:0]	 simmD	= {instrD[31:25], instrD[11:7]};
	wire [11:0]  bimmD	= {instrD[31],instrD[7],instrD[30:25],instrD[11:8]};
	wire [19:0]	 uimmD	= instrD[31:12];
	wire [19:0]  jimmD	= {instrD[31], instrD[19:12], instrD[20], instrD[30:21]};
	wire [`XLEN-1:0]	immoutD, shftimmD;

	imm 	im(iimmD, simmD, bimmD, uimmD, jimmD, immctrlD, immoutD);

	
	wire memtoregE;	
	wire memtoregM;
	wire regwriteE; 
	wire[4:0] 	rdE;
	wire[4:0] 	rdM;
	wire[31:0] 	rdata1_forwardD;	//consider whether use forwarding data to branch_cmp
	wire[31:0] 	rdata2_forwardD;
	wire cmpsrca;					//given by forwarding unit, decide whether use forwarding data from aluoutM
	wire cmpsrcb;
	// register file (operates in decode and writeback)
	wire [`XLEN-1:0]		rdata1D, rdata2D, wdataW;
	wire [`RFIDX_WIDTH-1:0]	waddrW;
	regfile rf(clk, rs1D, rs2D, rdata1D, rdata2D, regwriteW, waddrW, wdataW, pcW);

	//hazard dection unit
	hazard_detection hazard_detect(memtoregE, memtoregM, regwriteE, memwriteD, btypeD, rdE, rdM, rs1D, rs2D, notStall);
	mux2 #(`XLEN) cmpa(rdata1D, aluoutM, cmpsrca, rdata1_forwardD);
	mux2 #(`XLEN) cmpb(rdata2D, aluoutM, cmpsrcb, rdata2_forwardD);
	cmp branch_cmp(rdata1_forwardD, rdata2_forwardD, bunsignedD, zeroD, ltD);
	
	// ID/EX pipeline registers
	// for control signals
	wire memwriteE;
	wire [1:0] alusrcaE;
	wire alusrcbE;	
	wire [3:0] aluctrlE;
	wire flushE = pcsrcE | !notStall;
	wire [1:0] whbE;
	wire lunsignedE;
	wire jalrE;

	floprc #(9) regE(clk, reset, flushE,
                  {regwriteD, memwriteD, alusrcaD, alusrcbD, aluctrlD}, 
                  {regwriteE, memwriteE, alusrcaE, alusrcbE, aluctrlE});
	floprc #(1) reg2E(clk, reset, flushE, memtoregD, memtoregE);
	floprc #(1) reg3E(clk, reset, flushE, pcsrcD, pcsrcE);
	floprc #(2) reg4E(clk, reset, flushE, whbD, whbE);
	floprc #(1) reg5E(clk, reset, flushE, lunsignedD, lunsignedE);
	floprc #(1) reg6E(clk, reset, flushE, jalrD, jalrE);
	
 	// for data
	wire [4:0] rs1E;
	wire [4:0] rs2E;
	wire [`XLEN-1:0]	srca1E, srcb1E, immoutE, srcaE, srcbE, aluoutE;
	wire [`ADDR_SIZE-1:0] 	pcE, pcplus4E;
	wire [`XLEN-1:0] pcaddsrc1E, pcbranch1E;
	
	floprc #(`XLEN) 		pr1E(clk, reset, flushE, rdata1D, srca1E);        	// data from rs1
	floprc #(`XLEN) 		pr2E(clk, reset, flushE, rdata2D, srcb1E);         	// data from rs2
	floprc #(`XLEN) 		pr3E(clk, reset, flushE, immoutD, immoutE);        	// imm output
	floprc #(5) 			pr4E(clk, reset, flushE, rs1D, rs1E);				// rs1
	floprc #(5) 			pr5E(clk, reset, flushE, rs2D, rs2E);				// rs2
 	floprc #(`RFIDX_WIDTH)  pr6E(clk, reset, flushE, rdD, rdE);         		// rd
 	floprc #(`ADDR_SIZE)	pr7E(clk, reset, flushE, pcD, pcE);            		// pc
 	floprc #(`ADDR_SIZE)	pr8E(clk, reset, flushE, pcplus4D, pcplus4E);  		// pc+4	

	// execute stage logic
	wire	  regwriteM;
	wire[4:0] rs2M;
	wire[4:0] rdW;
	wire[1:0] ForwardAE;		//signs given by forwarding unit
	wire[1:0] ForwardBE;
	wire wdMsrc;
	wire[31:0] srca_forwardE;	//consider whether use forwarding data to alu or address adder
	wire[31:0] srcb_forwardE;
	forwarding_unit forward(regwriteM, regwriteW, rdM, rdW, rs1E, rs2E, rs2M, memwriteM, btypeD, rs1D, rs2D, ForwardAE, ForwardBE, wdMsrc, cmpsrca, cmpsrcb);
	mux3 #(`XLEN)  forwarda_mux(srca1E, wdataW, aluoutM, ForwardAE, srca_forwardE);
	mux3 #(`XLEN)  forwardb_mux(srcb1E, wdataW, aluoutM, ForwardBE, srcb_forwardE);
 	mux4 #(`XLEN)  srca_mux(srca_forwardE, 0, pcE, pcplus4E, alusrcaE, srcaE);     	// alu src a mux
	mux2 #(`XLEN)  srcb_mux(srcb_forwardE, immoutE, alusrcbE, srcbE);			 	// alu src b mux
	
	mux2 #(`XLEN) pc_adder_mux(pcE, srca_forwardE, jalrE, pcaddsrc1E);			//jal or jalr?
	addr_adder pcadder2(pcaddsrc1E, immoutE, pcbranch1E);
	mux2 #(`XLEN) pcbranch_mux(pcbranch1E, (pcbranch1E & ~1), jalrE, pcbranchE);//jal or jalr?

	alu alu(srcaE, srcbE, 5'b0, aluctrlE, aluoutE, overflowE, zeroE, ltE, geE);

	// EX/MEM pipeline registers
	// for control signals
	wire [`XLEN-1:0] srcb1M;
	wire 		flushM = 0;
	floprc #(2) 	regM(clk, reset, flushM,
                  	{regwriteE, memwriteE},
                  	{regwriteM, memwriteM});
	floprc #(1) 	reg2M(clk, reset, flushM, memtoregE, memtoregM);
	floprc #(2) 	reg3M(clk, reset, flushE, whbE, whbM);
	floprc #(1) 	reg4M(clk, reset, flushE, lunsignedE, lunsignedM);
	// for data
	floprc #(`XLEN) 	        pr1M(clk, reset, flushM, aluoutE, aluoutM);
	floprc #(`RFIDX_WIDTH) 	 	pr2M(clk, reset, flushM, rdE, rdM);
	floprc #(`ADDR_SIZE)	    pr3M(clk, reset, flushM, pcE, pcM);            // pc
	floprc #(`XLEN)				pr4M(clk, reset, flushM, srcb1E, srcb1M);
	floprc #(5)					pr5M(clk, reset, flushM, rs2E, rs2M);


	// mem stage logic
	wire[3:0] amp;
	wire[`XLEN-1:0] process_data;			//after process, process_data is the real load data
  	ampattern ampat(aluoutM[1:0], whbM, amp);
	data_process2 process_unit(amp, readdataM, lunsignedM, process_data);	 
	mux2 #(`XLEN) writedata_mux(wdataW, srcb1M, wdMsrc, writedataM);	//consider forwarding data from WB stage(WB-->MEM)
  
  	// MEM/WB pipeline registers
  	// for control signals
  	wire flushW = 0;
  	wire memtoregW;
	floprc #(1) regW(clk, reset, flushW, regwriteM, regwriteW);
	floprc #(1) reg2W(clk, reset, flushW, memtoregM, memtoregW);
  	// for data
  	wire[`XLEN-1:0] aluoutW;
  	wire[`XLEN-1:0]	readdataW;

  	floprc #(`XLEN) 	    pr1W(clk, reset, flushW, aluoutM, aluoutW);
  	floprc #(`RFIDX_WIDTH)	pr2W(clk, reset, flushW, rdM, rdW);
  	floprc #(`ADDR_SIZE)	pr3W(clk, reset, flushW, pcM, pcW);            // pc
  	floprc #(`XLEN) 		pr4W(clk, reset, flushW, process_data, readdataW);

	// write-back stage logic
	mux2 #(`XLEN) toreg_mux(aluoutW, readdataW, memtoregW, wdataW);			//choose aluoutW or readdataW to register file
	assign waddrW = rdW;

endmodule