/****************************************************************/
/* rotary_filter.v - Interface to Rotary Encoder on a PmodENC	*/
/*																*/
/* Translated from Ken Chapman's VHDL code w/ no changes		*/
/****************************************************************/
module rotary_filter (
	input		rotary_a,			// A input from S3E Rotary Encoder
				rotary_b,			// B input from S3E Rotary Encoder
	
	output reg	rotary_event,		// Asserted high when rotary encoder changes position
				rotary_left,		// Asserted high when rotary direction is to the left
				
	input		clk					// input clock
);

	// declare internal variables
	reg				rotary_a_int,		// synchronization flip flops
					rotary_b_int;
				
	reg				rotary_q1,			// state flip-flops 
					rotary_q2,
					delay_rotary_q1;
					
					
	// The rotary switch contacts are filtered using their offset (one-hot) style to  
	// clean them. Circuit concept by Peter Alfke.
	// Note that the clock rate is fast compared with the switch rate.
 
	always @(posedge clk) begin
		// Synchronize inputs to clock domain using flip-flops in input/output blocks.
		rotary_a_int <= rotary_a;
		rotary_b_int <= rotary_b;
		
		case ({rotary_b_int, rotary_a_int})
			2'b00: 	begin
						rotary_q1 <= 0;         
						rotary_q2 <= rotary_q2;
				   	end
			2'b01: 	begin
						rotary_q1 <= rotary_q1;         
						rotary_q2 <= 0;
				   	end
 			2'b10: 	begin
						rotary_q1 <= rotary_q1;         
						rotary_q2 <= 1;
				   	end
 			2'b11: 	begin
						rotary_q1 <= 1;         
						rotary_q2 <= rotary_q2;
				   	end
		endcase
	end //always
	
	// The rising edges of 'rotary_q1' indicate that a rotation has occurred and the 
	// state of 'rotary_q2' at that time will indicate the direction. 
	always @(posedge clk) begin
	    delay_rotary_q1 <= rotary_q1;	// want to only catch the first edge
      	if (rotary_q1 && ~delay_rotary_q1) begin  // rotary position has changed
        	rotary_event <= 1;
       		rotary_left <= rotary_q2;
       	end
       	else begin  // rotary position has not moved
        	rotary_event <= 0;
        	rotary_left <= rotary_left;
      	end
     end //always

endmodule