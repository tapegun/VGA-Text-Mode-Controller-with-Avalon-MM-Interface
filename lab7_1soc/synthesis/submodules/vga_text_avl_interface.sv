/************************************************************************
Avalon-MM Interface VGA Text mode display

Register Map:
0x000-0x0257 : VRAM, 80x30 (2400 byte, 600 word) raster order (first column then row)
0x258        : control register

VRAM Format:
X->
[ 31  30-24][ 23  22-16][ 15  14-8 ][ 7    6-0 ]
[IV3][CODE3][IV2][CODE2][IV1][CODE1][IV0][CODE0]

IVn = Draw inverse glyph
CODEn = Glyph code from IBM codepage 437

Control Register Format:
[[31-25][24-21][20-17][16-13][ 12-9][ 8-5 ][ 4-1 ][   0    ] 
[[RSVD ][FGD_R][FGD_G][FGD_B][BKG_R][BKG_G][BKG_B][RESERVED]

VSYNC signal = bit which flips on every Vsync (time for new frame), used to synchronize software
BKG_R/G/B = Background color, flipped with foreground when IVn bit is set
FGD_R/G/B = Foreground color, flipped with background when Inv bit is set

************************************************************************/
`define NUM_REGS 601 //80*30 characters / 4 characters per register
`define CTRL_REG 600 //index of control register

module vga_text_avl_interface (
	// Avalon Clock Input, note this clock is also used for VGA, so this must be 50Mhz
	// We can put a clock divider here in the future to make this IP more generalizable
	input logic CLK,
	
	// Avalon Reset Input
	input logic RESET,
	
	// Avalon-MM Slave Signals
	input  logic AVL_READ,					// Avalon-MM Read
	input  logic AVL_WRITE,					// Avalon-MM Write
	input  logic AVL_CS,					// Avalon-MM Chip Select
	input  logic [3:0] AVL_BYTE_EN,			// Avalon-MM Byte Enable
	input  logic [9:0] AVL_ADDR,			// Avalon-MM Address
	input  logic [31:0] AVL_WRITEDATA,		// Avalon-MM Write Data
	output logic [31:0] AVL_READDATA,		// Avalon-MM Read Data
	
	// Exported Conduit (mapped to VGA port - make sure you export in Platform Designer)
	output logic [3:0]  red, green, blue,	// VGA color channels (mapped to output pins in top-level)
	output logic hs, vs						// VGA HS/VS
);

logic [31:0] LOCAL_REG       [`NUM_REGS]; // Registers
//put other local variables here
assign LOCAL_REG [CTRL_REG] = {7'b0000000, 4'b1111, 4'b1111, 4'b1111, 4'b0000, 4'b0000, 4'b0000, 1'b0};
							//   UNUSED															UNUSED
logic [9:0] drawX;
logic [9:0] drawy;
logic [10:0] index; // look at IBM code page 437 to find out the indexing
logic [7:0] keybin;
logic [6:0] posx;
logic [4:0] posy;

//Declare submodules..e.g. VGA controller, ROMS, etc
vga_controller (.*, .Clk(CLK), .Reset(RESET), .DrawX(drawX), .DrawY(drawy));  //FILL IN
font_rom (.addr(index), .data(keybin)); //FILL IN
// Read and write from AVL interface to register block, note that READ waitstate = 1, so this should be in always_ff
always_ff @(posedge CLK) begin
	if(AVL_CS) begin
		if(AVL_READ) begin
			AVL_READDATA = LOCAL_REG[AVL_ADDR];
		end
		if(AVL_WRITE) begin // to optimize in the future remove this if statement and replace with else.. I am specifying the flow very hard because I want to minimize error
			case (AVL_BYTE_EN)
				4'b1000 :
					LOCAL_REG[AVL_ADDR][31:24] <= AVL_WRITEDATA[31:24];

				4'b0100 :
					LOCAL_REG[AVL_ADDR][23:16] <= AVL_WRITEDATA[23:16];
				4'b0010 :
					LOCAL_REG[AVL_ADDR][15:8] <= AVL_WRITEDATA[15:8];
				4'b0001 :
					LOCAL_REG[AVL_ADDR][7:0] <= AVL_WRITEDATA[7:0];
				default: 
					LOCAL_REG[AVL_ADDR] <= AVL_WRITEDATA;
			endcase
		end

	end
end


// 80 columns, 30 rows 

// 8 columns by 16 rows for each character where each spot is a pixel

//handle drawing (may either be combinational or sequential - or both).
always_comb begin
	xpos = drawX/8;
	ypos = drawY/16;
	// ((ypos) * 80 + xpos)  // number of the block we are on 
	// ypos*20 + (xpos/4)	// address we are dereferencing
	// xpos%4

	// INVERSION
//	LOCAL_REG[ypos*20 + (xpos/4)]
	unique case (xpos%4)
		0:	
			index = LOCAL_REG[ypos*20 + (xpos/4)][6:0] + drawY%16;   // index is the input to our FONT ROM and sets keybin to an 8 bit array which we need to
																						// find our bit that represents drawX and drawY
			red = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][7])) ^ 4'(signed'(keybin[drawX%8]));
			green = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][7])) ^ 4'(signed'(keybin[drawX%8]));
			blue = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][7])) ^ 4'(signed'(keybin[drawX%8]));											
		1:	
			index = LOCAL_REG[ypos*20 + (xpos/4)][14:8] + drawY%16;
			red = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][15])) ^ 4'(signed'(keybin[drawX%8]));	
			green = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][15])) ^ 4'(signed'(keybin[drawX%8]));	
			blue = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][15])) ^ 4'(signed'(keybin[drawX%8]));	

		2:	
			index = LOCAL_REG[ypos*20 + (xpos/4)][22:16] + drawY%16;
			red = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][23])) ^ 4'(signed'(keybin[drawX%8]));
			green = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][23])) ^ 4'(signed'(keybin[drawX%8]));
			blue = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][23])) ^ 4'(signed'(keybin[drawX%8]));

		3:	
			index = LOCAL_REG[ypos*20 + (xpos/4)][30:24] + drawY%16;
			red = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][31])) ^ 4'(signed'(keybin[drawX%8]));
			green = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][31])) ^ 4'(signed'(keybin[drawX%8]));
			blue = 4'(signed'(LOCAL_REG[ypos*20 + (xpos/4)][31])) ^ 4'(signed'(keybin[drawX%8]));
	endcase

end

endmodule

