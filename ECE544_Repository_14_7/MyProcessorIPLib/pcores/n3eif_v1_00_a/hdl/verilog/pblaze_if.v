
/*************************************************/
/* pblaze_if.v - Interface to embedded Picoblaze */
/*************************************************/
module pblaze_if (
	// interface to the picoblaze
	input 				Wr_Strobe,			// Write strobe - asserted to write I/O data
		 				Rd_Strobe,			// Read strobe - asserted to read I/O data
	input [7:0] 		AddrIn,				// I/O port address
	input [7:0] 		DataIn,				// Data to be written to I/O register
	output reg [7:0] 	DataOut,			// I/O register data to picoblaze
   	
	// interface to the external CPU
	// inputs assume registers are instantiated external to this module if necessary	
	input [7:0]			btn_sw_in,			// buttons and switches from S3E board
						leds_in,			// led contents to display
						rotary_ctl,			// rotary control register
						lcd_cmd,			// LCD command register
						lcd_data,			// LCD data register
		
	output reg [7:0]	btn_sw_out,			// debounced pushbuttons and switches
						leds_out,			// led outputs
						rotary_status,		// rotary encoder status
						rotary_count_lo,	// rotary count bits[7:0]
						rotary_count_hi,	// rotary count bits[15:8]
						lcd_status,			// LCD status register
						lcd_ctl,			// LCD control signals 
						lcd_dbus,			// LCD data bus
						
	// local (to N3IF) interface
	input [7:0]			rot_lcd_inputs,		// rotary encoder and lcd data control inputs
	input				enable,				// enables I/O range when set to 1

	// system clock and reset		
	input				reset,				// System reset
						clk					// 50Mhz clock signal
 );
 

	// declare internal signals
	wire		write_en;				// write enable for this module				

					
	// respond to IO writes only if address is within my address range
	assign write_en = Wr_Strobe & enable;

	// read registers - drives registered values to the picoblaze
	always @(posedge clk) begin
		if (enable) begin
			case (AddrIn[2:0])
				// Input registers
				3'b000 :	DataOut = btn_sw_in;		// buttons and switches
				3'b001 :	DataOut = rotary_ctl;		// rotary control register	
				3'b010 :	DataOut = rot_lcd_inputs;	// rotary filter and lcd inputs
				3'b011 : 	DataOut = lcd_cmd;			// lcd command		
				3'b100 : 	DataOut = lcd_data;			// lcd data	
				3'b101 : 	DataOut = 8'd5;				// reserved
				3'b110 :	DataOut = rotary_count_lo;	// least significant byte of rotary count
													// (used for self test)
				3'b111 :	DataOut = leds_in;			// led inputs					
			endcase
		end
	end // always - read registers


	// write registers - drives external CPU registers from picoblaze
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			btn_sw_out <= 8'h00;		// start w/ buttons and switches off
			rotary_status <= 8'h00;		// start w/ rotary encoder not busy
			lcd_status <= 8'h80;		// start w/ LCD display busy
			lcd_ctl <= 8'h00;			// start w/ all LCD control signals at 0
			lcd_dbus <= 8'h00;			// start w/ LCD data bus at 0
			rotary_count_lo <= 8'h00;	// start w/ rotary count = 0
			rotary_count_hi <= 8'h00;	
		end
		else begin
			if(write_en) begin
				 case (AddrIn[2:0])
					// output registers
					3'b000 :	btn_sw_out <= DataIn;
					3'b001 :	rotary_status <= DataIn;
					3'b010 :	lcd_dbus <= DataIn;
					3'b011 :	lcd_status <= DataIn;
					3'b100 :	lcd_ctl <= DataIn;	
					3'b101 :	rotary_count_lo <= DataIn;
					3'b110 :	rotary_count_hi <= DataIn;
					3'b111 :	leds_out <= DataIn;			// led outputs
				endcase
			end
		end
	end // always - write registers
		
endmodule	