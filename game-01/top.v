/*
This game uses a potentiometer input on a Pololu Maestro which sends a serial
UART 8 bit data to the fpga. The FPGA creates an image on the VGA output which
is effected by the 8bit data. It also determines the rotation angle of the
servo using the same data. When the player and wall intercept, audio is played.
*/
module top (
    input CLK,    // 16MHz clock
    input PIN_19, //serial data in
    output PIN_12, //VGA hsync about 31kHz
    output PIN_13, //VGA vsync 60Hz
    output PIN_14, //VGA red
    output PIN_15, //VGA green
    output PIN_16, //VGA blue
    output PIN_20, //audio for speaker
    output PIN_24, //servo_1
    output LED,   // User/boot LED next to power LED
    output USBPU  // USB pull-up resistor
);

    assign USBPU = 0;  // drive USB pull-up resistor to '0' to disable USB
    assign LED = 0; //LED is zero so I know the FPGA is running or bootloader

    wire [2:0] rgb; //color mix for vga signal
    wire hsync; //low for pulse
    wire vsync; //low for pulse
    wire red; //output from vga module high when visible
    wire green; //high when visible
    wire blue; //high when visible
    wire [9:0] x; //x and y coordinates count up during active region
    wire [9:0] y;
    wire [7:0] eightbit_mod_1; //created by the uart_rx module from maestro
    wire servopulse_1; //driven by servo module
    wire cue_music;
    wire wave_form;

    reg serial;

    always @(posedge CLK) begin
      serial <= PIN_19;
      end


    uart_rx uart_rx_1 (
      .CLK (CLK),
      .i_Rx_Serial (serial),
      .o_Rx_Byte (eightbit_mod_1)
      );

    vga_tx vga_tx_1 (
      .CLK (CLK), //input
      .rgb (rgb), //input
      .hsync (hsync), //outputs
      .vsync (vsync),
      .red (red),
      .green (green),
      .blue (blue),
      .x (x),
      .y (y));

    game game_1 (
      .CLK (CLK),
      .eightbit_mod_1 (eightbit_mod_1),
      .x (x),
      .y (y),
      .rgb (rgb),
      .hit (cue_music)
      );

    servo servo_1 (
      .CLK (CLK),
      .eightbit_mod_1 (eightbit_mod_1),
      .o_servopulse (servopulse_1)
      );

    music music_1 (
      .CLK (CLK),
      .start (cue_music),
      .wave_form (wave_form)
      );

    assign PIN_12 = hsync; //use npn as a buffer
    assign PIN_13 = vsync;
    assign PIN_14 = red; //red highest value is .7V at 75 ohm impedence
    assign PIN_15 = green; //green
    assign PIN_16 = blue; //blue
    assign PIN_24 = servopulse_1; //use transistor buffer / amp to +5V
    assign PIN_20 = wave_form;

endmodule //top


module game (
  input CLK,
  input [7:0] eightbit_mod_1,
  input [9:0] x,
  input [9:0] y,
  output [2:0] rgb,
  output hit );

//////////////////////////////////////////////////////////////
/////////Introducing a sweep from the music siren code
reg [27:0] tone;
    always @(posedge CLK) begin
    tone <= tone +1;
    end

wire [7:0] ramp = (tone[25] ? tone[24:17] : ~tone[24:17]);

assign wall = (x == ramp);
assign player = (x > eightbit_mod_1 - 2 ? (x < eightbit_mod_1 + 2 ? 1 : 0) : 0)
  && (y > 25 ? (y < 30 ? 1 : 0) : 0);
assign impact = player && wall;
assign hit = impact;


//Test Pattern with all colors possible in RGB 3'bxxx
assign box1 = (x > 0 ? (x < 116 ? 1 : 0) : 0); //first column
assign box2 = (x > 58 ? (x < 232 ? 1 : 0) : 0); //second column
assign box3 = (x > 174 ? (x < 348 ? 1 : 0) : 0); //third column
assign box4 = (x > 290 ? (x < 348 ? 1 : 0) : 0); //final column

assign rgb [2] = wall; //red
assign rgb [1] = player; //green
assign rgb [0] = impact; //blue

endmodule //game



module servo (
    input CLK,
    input [7:0] eightbit_mod_1, //input an eight bit control
    output o_servopulse   // servo output pulse
    );

parameter ClkDiv = 63; // 16000000/1000/256 = 62.5

reg [5:0] ClkCount; //64 count block, I only need to 63
reg ClkTick;
reg [11:0] PulseCount; //12 bits or 4,096 block
reg [7:0] RCServo_position; //8bit block 256 count
reg RCServo_pulse; //our final servo PWM output

//ClkTick <= 1 when ClkCount equals ClkDiv
  always @(posedge CLK) begin
    ClkTick <= (ClkCount==ClkDiv); //removed -2
    end

  always @(posedge CLK) begin
    if (ClkTick) //if clktick is true, when clkcount = ClkDiv
    ClkCount <= 0; //make clkcount 0
    else
    ClkCount <= ClkCount +1;
    end

  always @(posedge CLK) begin
    if (ClkTick) //if clktick is true, when clkcount = ClkDiv
    PulseCount <= PulseCount +1; //pulse count increases by 1
    end

  always @(posedge CLK) begin
    RCServo_position <= eightbit_mod_1; //control servo here
    end

  always @(posedge CLK) begin
   RCServo_pulse = (PulseCount < {4'b0001, RCServo_position});
   end

assign o_servopulse = RCServo_pulse;

endmodule //servo


module uart_rx
  #(parameter CLKS_PER_BIT = 1667)
  (input CLK,
   input i_Rx_Serial,
   output [7:0] o_Rx_Byte
   );

  parameter s_IDLE         = 3'b000;
  parameter s_RX_START_BIT = 3'b001;
  parameter s_RX_DATA_BITS = 3'b010;
  parameter s_RX_STOP_BIT  = 3'b011;
  parameter s_CLEANUP      = 3'b100;

  reg          r_Rx_Data_R = 1'b1;
  reg          r_Rx_Data   = 1'b1;

  reg [10:0] r_Clock_Count = 0;
  reg [2:0]  r_Bit_Index   = 0; //8 bits total
  reg [7:0]  r_Rx_Byte     = 0;
  reg        r_Rx_DV       = 0;
  reg [2:0]  r_SM_Main     = 0;

  // Purpose: Double-register the incoming data.
  // This allows it to be used in the UART RX Clock Domain.
  // (It removes problems caused by metastability)
  always @(posedge CLK) begin
      r_Rx_Data_R <= i_Rx_Serial;
      r_Rx_Data   <= r_Rx_Data_R;
     end


  // Purpose: Control RX state machine
  always @(posedge CLK)
    begin
      case (r_SM_Main)

        s_IDLE :
          begin
            r_Rx_DV       <= 1'b0;
            r_Clock_Count <= 0;
            r_Bit_Index   <= 0;

            if (r_Rx_Data == 1'b0)          // Start bit detected
              r_SM_Main <= s_RX_START_BIT;
            else
              r_SM_Main <= s_IDLE;
          end

        // Check middle of start bit to make sure it's still low
        s_RX_START_BIT :
          begin
            if (r_Clock_Count == (CLKS_PER_BIT-1)/2)
              begin
                if (r_Rx_Data == 1'b0)
                  begin
                    r_Clock_Count <= 0;  // reset counter, found the middle
                    r_SM_Main     <= s_RX_DATA_BITS;
                  end
                else
                  r_SM_Main <= s_IDLE;
              end
            else
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_START_BIT;
              end
          end // case: s_RX_START_BIT


        // Wait CLKS_PER_BIT-1 clock cycles to sample serial data
        s_RX_DATA_BITS :
          begin
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_DATA_BITS;
              end
            else
              begin
                r_Clock_Count          <= 0;
                r_Rx_Byte[r_Bit_Index] <= r_Rx_Data;

                // Check if we have received all bits
                if (r_Bit_Index < 7)
                  begin
                    r_Bit_Index <= r_Bit_Index + 1;
                    r_SM_Main   <= s_RX_DATA_BITS;
                  end
                else
                  begin
                    r_Bit_Index <= 0;
                    r_SM_Main   <= s_RX_STOP_BIT;
                  end
              end
          end // case: s_RX_DATA_BITS


        // Receive Stop bit.  Stop bit = 1
        s_RX_STOP_BIT :
          begin
            // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_STOP_BIT;
              end
            else
              begin
                r_Rx_DV       <= 1'b1;
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
              end
          end // case: s_RX_STOP_BIT


        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_SM_Main <= s_IDLE;
            r_Rx_DV   <= 1'b0;
          end


        default :
          r_SM_Main <= s_IDLE;

      endcase
    end

assign o_Rx_Byte = r_Rx_Byte;

endmodule // uart_rx




module vga_tx (
  input CLK,
  input [2:0] rgb,
  output hsync,
  output vsync,
  output reg red,
  output reg green,
  output reg blue,
  output reg [9:0] x,
  output reg [9:0] y );

  ///Video timing parameters, should appear as 640 X 480, but is really
  //407 X 480 @ 60HZ.

  parameter active_h_video = 407; //calculated from 16MHz clk
  parameter active_v_video = 480;
  parameter h_FP = 10; //front porch
  parameter h_pulse = 61; //horizontal sync pulse
  parameter h_BP = 31; //horizontal back porch
  parameter v_FP = 10; //veritcal front porch
  parameter v_pulse = 2; //vertical sync pulse
  parameter v_BP = 33; //vertical back porch
  parameter blank_h = h_FP + h_pulse + h_BP; //adds up all blanking time
  parameter blank_v = v_FP + v_pulse + v_BP;
  parameter h_total = blank_h + active_h_video; //adds up all pixel time
  parameter v_total = blank_v + active_v_video;

  reg [8:0] h_counter = 0;
  reg [9:0] v_counter = 0;

  wire active_video;


  always @(posedge CLK) begin
    if (h_counter < h_total -1) //counter below total pixels
     h_counter <= h_counter +1; //increment counter
    else
     begin
      h_counter <= 0; //reset counter
      if (v_counter < v_total -1) //check vertical counter value
       v_counter <= v_counter +1; //increment vertical counter
      else
       v_counter <= 0;  //reset vertical counter
     end
    end


  //This generates our active low sync pulses by using this assign comparison
  assign hsync = (h_counter >= h_FP && h_counter < h_FP + h_pulse) ? 0:1;
  assign vsync = (v_counter >= v_FP && v_counter < v_FP + v_pulse) ? 0:1;
  assign active_video = (h_counter >= h_FP + h_pulse + h_BP
    && v_counter >= v_FP + v_pulse + v_BP) ? 1:0;
    //If I read this correctly, the active_video is high when the two counters
    //are both greater than the sum of the blanking regions, which doesn't
    //make sense to me, because that could be arbitray value after the visible
    //region has started. But x and y have these values removed, so they are
    //correct for screen display.

  always @(posedge CLK) begin
   if (active_video == 1) //looks like its actve when in the blanking region
    begin
     red <= rgb [2];
     green <= rgb [1];
     blue <= rgb [0];
     x <= h_counter - h_FP - h_pulse - h_BP; //x is counter minus the blanking
     y <= v_counter - v_FP - v_pulse - v_BP; //y is counter minus the blanking
    end
   else //active_video is low
    begin
     red <= 0; //everything is zero
     green <= 0;
     blue <= 0;
     x <= 0;
     y <= 0;
    end
  end

  endmodule //vga_tx



//////////////////////////////////////////////////////////////////////////////
  module music (
      input CLK,    // 16MHz clock
      input start,
      output wave_form // Song
      );

  reg [30:0] tone;

   always @(posedge CLK) begin
   if (start == 1)
    tone <= tone + 1;
   else
    tone <= 0;
    end

  wire [7:0] fullnote;

  music_ROM get_fullnote (
    .CLK (CLK),
    .address (tone[27:26]),
    .note (fullnote)
    );

  wire [2:0] octave;
  wire [3:0] note;

  divide_by12 get_octave_and_note (
    .numer (fullnote[5:0]),
    .quotient (octave),
    .remain (note)
    );

reg [8:0] clkdivider;

  always @(note) begin
    case (note)
     0: clkdivider = 512-1; //A
     1: clkdivider = 483-1;
     2: clkdivider = 456-1;
     3: clkdivider = 431-1;
     4: clkdivider = 406-1;
     5: clkdivider = 384-1;
     6: clkdivider = 362-1;
     7: clkdivider = 342-1;
     8: clkdivider = 323-1;
     9: clkdivider = 304-1;
    10: clkdivider = 287-1;
    11: clkdivider = 271-1;
    12: clkdivider = 0;
    13: clkdivider = 0;
    14: clkdivider = 0;
    15: clkdivider = 0;
    endcase
    end

reg [8:0] counter_note;

 always @(posedge CLK) begin
  if (counter_note==0)
    counter_note <= clkdivider;
  else
    counter_note <= counter_note - 1;
    end

reg [7:0] counter_octave;

  always @(posedge CLK) begin
    if (counter_note==0)
    begin
      if(counter_octave==0)
       counter_octave <= (octave==0 ? 255 : octave==1 ? 127 : octave==2 ?
        63 : octave==3 ? 31 : octave==4 ? 15 : 7);
      else
      counter_octave <= counter_octave - 1;
      end
    end

reg wave_form;

  always @(posedge CLK) begin
    if (counter_note==0 && counter_octave==0 && tone[30]==0 && fullnote!=0)
    wave_form <= ~wave_form;
    end

endmodule //music


////////////////////////////////////////////////////////////////////////////
module divide_by12 (
    input [5:0] numer,
    output reg [2:0] quotient,
    output [3:0] remain
    );

  reg [1:0] remain_bit3_bit2;

  always @(numer[5:2]) //and just do a divide by "3" on the remaining bits
    case(numer[5:2])
       0: begin quotient=0; remain_bit3_bit2=0; end
       1: begin quotient=0; remain_bit3_bit2=1; end
       2: begin quotient=0; remain_bit3_bit2=2; end
       3: begin quotient=1; remain_bit3_bit2=0; end
       4: begin quotient=1; remain_bit3_bit2=1; end
       5: begin quotient=1; remain_bit3_bit2=2; end
       6: begin quotient=2; remain_bit3_bit2=0; end
       7: begin quotient=2; remain_bit3_bit2=1; end
       8: begin quotient=2; remain_bit3_bit2=2; end
       9: begin quotient=3; remain_bit3_bit2=0; end
      10: begin quotient=3; remain_bit3_bit2=1; end
      11: begin quotient=3; remain_bit3_bit2=2; end
      12: begin quotient=4; remain_bit3_bit2=0; end
      13: begin quotient=4; remain_bit3_bit2=1; end
      14: begin quotient=4; remain_bit3_bit2=2; end
      15: begin quotient=5; remain_bit3_bit2=0; end
      endcase

      assign remain[1:0] = numer[1:0];  // the first 2 bits are copied through
      assign remain[3:2] = remain_bit3_bit2;  // and the last 2 bits come from
                                              //the case statement
    endmodule //divide_by12


  //////////////////////////////////////////////////////////////////////////////
      module music_ROM (
      	input CLK,
      	input [1:0] address,
      	output reg [7:0] note
        );

      always @(posedge CLK)
      case(address)
      	  0: note <= 8'd25;
      	  1: note <= 8'd30;
      	  2: note <= 8'd35;
      	  3: note <= 8'd20;
      	default: note <= 8'd0;
      endcase

    endmodule //music_ROM
