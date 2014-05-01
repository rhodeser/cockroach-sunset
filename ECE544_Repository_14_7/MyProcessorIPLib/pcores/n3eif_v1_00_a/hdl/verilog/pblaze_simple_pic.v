/////////////////////////////////////////////////////////////////////
////                                                             ////
////  pblaze_simple_pic.v - 8 input interrupt controller for the ////
////  Xilinx Picoblaze                                           ////
////                                                             ////
////  Modified by:	Roy Kravitz		                             ////
////          		roy.kravitz@pdx.edu                      	 ////
//// 						                                     ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
////                                                             ////
////  OpenCores         Simple Programmable Interrupt Controller ////
////                                                             ////
////  Author: Richard Herveille                                  ////
////          richard@asics.ws                                   ////
////          www.asics.ws                                       ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2002 Richard Herveille                        ////
////                    richard@asics.ws                         ////
////                                                             ////
//// This source file may be used and distributed without        ////
//// restriction provided that this copyright statement is not   ////
//// removed from the file and that any derivative work contains ////
//// the original copyright notice and the associated disclaimer.////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////


//
// Change History:
//				 Revision 1.4  2005/05/19 royk
//				 Modified bus interface to match Picoblaze
//
//               Revision 1.3  2002/12/24 10:26:51  rherveille
//               Fixed some typos in the documentation.
//
//               Revision 1.2  2002/12/22 16:11:03  rherveille
//               *** empty log message ***
//
//

//
// This is a simple Programmable Interrupt Controller.
// The number of interrupts is depending on the databus size.
// There's one interrupt input per databit (i.e. 16 interrupts for a 16
// bit databus).
// All attached devices share the same CPU priority level.
//
//
//
// Registers:
//
// 0x00: EdgeEnable Register
//       bits 7:0 R/W  Edge Enable '1' = edge triggered interrupt source
//                                 '0' = level triggered interrupt source
// 0x01: PolarityRegister
//       bits 7:0 R/W Polarity     '1' = high level / rising edge
//                                 '0' = low level / falling edge
// 0x02: MaskRegister
//       bits 7:0 R/W Mask         '1' = interrupt masked (disabled)
//                                 '0' = interrupt not masked (enabled)
// 0x03: PendingRegister
//       bits 7:0 R/W Pending      '1' = interrupt pending
//                                 '0' = no interrupt pending
//
// A CPU interrupt is generated when an interrupt is pending and its
// MASK bit is cleared.
//
//
//
// HOWTO:
//
// Clearing pending interrupts:
// Writing a '1' to a bit in the interrupt pending register clears the
// interrupt. Make sure to clear the interrupt at the source before
// writing to the interrupt pending register. Otherwise the interrupt
// will be set again.
//
// Priority based interrupts:
// Upon reception of an interrupt, check the interrupt register and
// determine the highest priority interrupt. Mask all interrupts from the
// current level to the lowest level. This negates the interrupt line, and
// makes sure only interrupts with a higher level are triggered. After
// completion of the interrupt service routine, clear the interrupt source,
// the interrupt bit in the pending register, and restore the MASK register
// to it's previous state.
//
// Adapt the core for fewer interrupt sources:
// If less than 8 interrupt sources are required, than the 'is' parameter
// can be set to the amount of required interrupts. Interrupts are mapped
// starting at the LSBs. So only the 'is' LSBs per register are valid. All
// other bits (i.e. the 8-'is' MSBs) are set to zero '0'.
// Codesize is approximately linear to the amount of interrupts. I.e. using
// 4 instead of 8 interrupt sources reduces the size by approx. half.
//


module pblaze_simple_pic (
	// interface to the picoblaze
	input 				Wr_Strobe,			// Write strobe - asserted to write I/O data
		 				Rd_Strobe,			// Read strobe - asserted to read I/O data
	input [7:0] 		AddrIn,				// I/O port address
	input [7:0] 		DataIn,				// Data to be written to I/O register
	output reg [7:0] 	DataOut,			// I/O register data to picoblaze
	
	input [7:0]			Int_in,				// Interrupt sources
    output				Int,				// Interrupt signal to Picoblaze
	input				Int_ack,			// Interrupt acknowledge from PicoBlaze

	input				enable,				// enables I/O range when set to 1
	input				reset,				// System reset
						clk					// 50Mhz clock signal
 );

 	// Number of interrupt sources
  	parameter is = 8;


  	//
  	//  Module body
  	//
  	reg  [is:1]	 	pol, edgen, pending, mask;	// register bank
  	reg  [is:1]		lirq, dirq;					// latched irqs, delayed latched irqs
  	reg				intff;						// interrupt flip flop
  	wire			clr_intff;					// clear interrupt flip flop
  	wire			write_en;					// write enable for this module
  	  	


	//
	// perform parameter checks
	//
	// synopsys translate_off
  	initial
  	begin
 	     if(is > 8)
	        $display("simple_pic: max. 8 interrupt sources supported.");
	  end
	// synopsys translate_on

  	//
  	// latch interrupt inputs
  	always @(posedge clk)
    	lirq <= #1 Int_in;

  	//
  	// generate delayed latched irqs
  	always @(posedge clk)
    	dirq <= #1 lirq;


  	//
  	// generate actual triggers
  	function trigger;
		input edgen, pol, lirq, dirq;

		reg   edge_irq, level_irq;
		begin
			edge_irq  = pol ? (lirq & ~dirq) : (dirq & ~lirq);
			level_irq = pol ? lirq : ~lirq;

			trigger = edgen ? edge_irq : level_irq;
		end
	endfunction

	reg  [is:1] irq_event;
	integer n;
	always @(posedge clk)
	for(n=1; n<=is; n=n+1)
		irq_event[n] <= #1 trigger(edgen[n], pol[n], lirq[n], dirq[n]);

	// Picoblaze bus interface
	//
	// respond to IO access only if address is within my address range
	assign write_en = Wr_Strobe & enable;

	// write registers
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			edgen <= #1 {{is}{1'b0}};			// clear edge enable register
			pol   <= #1 {{is}{1'b0}};			// clear polarity register
			mask  <= #1 {{is}{1'b1}};			// mask all interrupts
		end
		else begin
			if(write_en) begin
				 case (AddrIn[1:0])
					// output registers
					2'b00 :	edgen <= DataIn;	// write edge enable register
					2'b01 :	pol <= DataIn;		// write polarity register
					2'b10 :	mask <= DataIn;		// write mask register
					2'b11 :	;					// pending register is a special case
				endcase
			end
		end
	end // always - write registers


    // pending register is a special case
    always @(posedge clk or posedge reset)
      if (reset)
          pending <= #1 {{is}{1'b0}};            // clear all pending interrupts
      else if ( write_en & (AddrIn[1:0] == 2'b11) )
          pending <= #1 (pending & ~DataIn[is-1:0]) | irq_event;
      else
          pending <= #1 pending | irq_event;

	// read registers - drives registered values for output ports
	always @* begin
			case (AddrIn[1:0])
				// Input registers
				2'b00 :	DataOut = #1 { {{8-is}{1'b0}}, edgen};
				2'b01 :	DataOut = #1 { {{8-is}{1'b0}}, pol};
				2'b10 :	DataOut = #1 { {{8-is}{1'b0}}, mask};	
				2'b11 :	DataOut = #1 { {{8-is}{1'b0}}, pending};
			endcase
	end // always - read registers


	// Interrupt flip-flop (based on PicoBlaze documentation)	
	assign Int = intff;
	assign clr_intff = reset | (write_en & (AddrIn[1:0] == 2'b11));

	always @(posedge clk) begin
		if (clr_intff)
			intff <= #1 1'b0;
		else if (|(pending & ~mask))
			intff <= #1 1'b1;
		else
			intff <= intff;
	end  // always  for interrupt flip-flop

endmodule

