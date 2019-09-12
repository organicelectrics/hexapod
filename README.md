# hexapod
This is the collection of code projects which involve an ongoing effort to build an insectoid hexapod robot.

I am starting with Verilog and sharing some preliminary projects.
The Game folder contains the verilog code and synthesized files for uploading to a TinyFPGA BX. I use Atom with APIO. The TinyFPGA BX github contains more information about working with that board. I created this project to test analog to digital input control and output to VGA and servo. The Pololu Maestro Board converts the analog value of a potentiometer to an 8bit digital value, serializes it into a UART signal at 9600 baud, and the Tinyfpga receives it and works with that 8bit value. I used verilog code from FPGA4Fun.com, nandland.com, and the TinyFPGA discourse forum.
