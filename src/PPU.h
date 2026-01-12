#pragma once

#include <array>
#include <cstdint>
#include <vector>

class MemoryMapper;

static const uint16_t PPUCTRL_ADDRESS = 0x2000;
static const uint16_t PPUMASK_ADDRESS = 0x2001;
static const uint16_t PPUSTATUS_ADDRESS = 0x2002;
static const uint16_t OAMADDR_ADDRESS = 0x2003;
static const uint16_t PPU_SCROLL_ADDR_LATCH_ADDRESS = 0x2004;
static const uint16_t PPUSCROLL_ADDRESS = 0x2005;
static const uint16_t PPUADDR_ADDRESS = 0x2006;
static const uint16_t PPUDATA_ADDRESS = 0x2007;

// Occurs at dot one of scanline 241
// Triggers EPPUCTRL's VLANK_NMI_ENABLE flag.
// Enabling while PPUSTATUS's VBLANK flag is enabled immediately triggers NMI.
static const uint16_t VBLANK_SCANLINE = 241;

enum class EAddressMap {
	Pattern_Table_0,
	Pattern_Table_1,
	Name_Table_0,
	Attribute_Table_0,
	Name_Table_1,
	Attribute_Table_1,
	Name_Table_2,
	Attribute_Table_2,
	Name_Table_3,
	Attribute_Table_3,
	Pallete_Ram_Indexes,
	Pallete_Ram_Indexes_Mirror
};

// Writes to this register are ignored until first pre-render scanline
enum class EPPUCTRL {
	// 2 bit combination -  0 = 0x2000, 1 = 0x2400, 2 = 0x2800, 3 = 0x2C00
	// Most significant bits for scroll coordiantes (9 bits). The other 8 bits come from PPUSCROLL.
	// Both these two bits and PPUSCROLL must be written at the same time.
	// Ex. bit 0 adds 256 to X, and bit 1 adds 240 to Y.
	BASE_NAMETABLE_ADDRESS_0 = 1 << 0,
	BASE_NAMETABLE_ADDRESS_1 = 1 << 1,

	// Per CPU Read/Write of PPUDATA
	// 0: Add 1 going across, 1: add 32, going down
	VRAM_ADDRESS_INCREMENT = 1 << 2,

	// 0: 0x0000, 1: 0x1000, ignored in 8x16 mode
	SPRITE_PATTERN_ADDRESS = 1 << 3,
	
	// 0: 0x0000, 1: 0x1000
	BACKGROUND_PATTERN_ADDRESS = 1 << 4,

	// 0: 8x8, 1: 8x16. Reference: PPU OAM#Byte 1
	SPRITE_SIZE = 1 << 5,

	// 0: Read Backdrop from EXT Pins, 1: Output color on EXT Pins.
	PPU_MASTER_SLAVE_SELECT = 1 << 6,

	// 0: off, 1: on
	VBLANK_NMI_ENABLE = 1 << 7
};

// Writes to this register are ignored until first pre-render scanline
// Commonly 0x00 outside of gameplay, and 0x1E during gameplay
enum class EPPUMASK {
	// 0: Normal Color, 1: Greyscale.
	// Gray Scale is computed via ANDing the color with 0x30.
	GREYSCALE = 1 << 0,

	// 0: Hide, 1: Show
	BACKGROUND_LEFTMOST_8PX = 1 << 1,

	// 0: Hide, 1: Show
	SPRITES_LEFTMOST_8PX = 1 << 2,

	ENABLE_BACKGROUND_RENDERING = 1 << 3,
	ENABLE_SPRITE_RENDERING = 1 << 4,

	// Green on PAL/Dendy
	EMPHASIZE_RED = 1 << 5,

	// Red on PAL/Dendy
	EMPHASIZE_GREEN = 1 << 6,

	EMPHASIZE_BLUE = 1 << 7
};

// PPUSTATUS Register
const uint8_t OPEN_BUS_MASK = 0b00011111;

enum class EPPUSTATUS {
	OPEN_BUS_0 = 1 << 0,
	OPEN_BUS_1 = 1 << 1,
	OPEN_BUS_2 = 1 << 2,
	OPEN_BUS_3 = 1 << 3,
	OPEN_BUS_4 = 1 << 4,

	// Triggered when 9 Sprites are attempted to be rendered on 1 scanline, instead of the 8 intended.
	// Used by games for timing when more than one timing source is wanted.
	SPRITE_OVERFLOW_FLAG = 1 << 5,

	// Set to 1 when collision is detected between OAM Sprite 0 and background, where two non-transparent pixels overlap (two pattern bits are %00, whatever that means).
	// Stays set until dot 1 of the pre-render scanline.
	// Cannot detect at X=255
	// Used for timing by games by placing sprite 0 and waiting for this flag.
	// ^ Games using this generally crash when not implemented.
	SPRITE_0_HIT_FLAG = 1 << 6,

	// Cleared on read, unreliable
	// Set at scanline 241, dot 1
	// Cleared on read, and cleared at dot 1 of pre-render scanline
	VBLANK_FLAG = 1 << 7
};

class PPU {
	std::vector<char> mMemory;

	// 4 Palletes, first 16 are background tiles, while last 16 are sprites.
	// Entry 0 of pallete 0 is the backdrop color.
	// Entry 0 of each pallete is transparent, so color values of these is ignored.
	// Backdrop color is displayed when both background and sprites at this pixel are transparent.
	std::array<char, 32> mPalleteMemory;

	// Display list of 64 Sprites, with 4 bytes of information each.
	// Uses Dynamic Memory which decays to 0 if PPU is not rendering.
	// 
	// Byte 0 - Y Pos (Top of sprite). 
	//			Sprite data is delayed by one scanline, need to subtract one before writting here. 
	//			0xEF - 0xFF are off-screen. 
	//			Sprites on first scanline are never displayed.
	//			Sprites cannot be partially off the top of the screen.
	// 
	// Byte 1 - Tile/Index
	//			8x8 Sprites  - Tile number within the pattern table selected in bit 3 of PPUCTRL (0x2000 CPU Memory).
	//			8x16 Sprites - (bit 5 of PPUCTRL set) PPU ignores pattern table selection, and selects a pattern table from bit 0 of this number.
	// 76543210
	// ||||||||
	// |||||||+ Bank($0000 or $1000) of tiles
	// ++++++- Tile number of top of sprite(0 to 254; bottom half gets the next tile)
	//
	// Byte 2 - Attributes 
	//			Flipping flips the pixels read, drawing to the same space as normal.
	//			For 8x16 sprites, each sub-sprite is flipped and the order is flipped so the bottom 8x8 sprite is drawn first at the top.
	//			The unimplemented bits are always zero, can be done by ANDing with 0xE3.
	//			Decayed bits read as 0.
	//			Some PPU revisions allow reading OAM data through OAMDATA (0x2004).
	// 76543210
	//||||||||
	//||||||++- Palette(4 to 7) of sprite
	//|||+++--- Unimplemented(read 0)
	//||+------ Priority(0: in front of background; 1: behind background)
	//|+------- Flip sprite horizontally
	//+ --------Flip sprite vertically
	// 
	// Byte 3 - X Pos (Left side of sprite). 0xF9 - 0xFF are beyond the right edge of the screen. Sprites further left then 0 are not visible.
	std::array<char, 256> mObjectAttributeMemory;

	// All 12 memory regions stored (Address Begin, size)
	std::array<std::pair<uint16_t, uint16_t>, 13> mMemoryMap;

	// CPU Address space, the PPU's Registers exist in the memory range 0x2000 to 0x2007, mirrored up to 0x3FFF.
	// 
	// PPU ignores writes (always 00?) for registers 0x2000, 0x2001, 0x2005, and 0x2006) 
	// until reaching the first pre-render scanline of next frame, aka ~29658 NTSC CPU or 33132 PAL CPU Cycles.
	MemoryMapper* mRAM;

	const std::vector<char>* mChrRomMemory;

public:
	PPU(MemoryMapper* RAM);

	void Init(const std::vector<char>* ChrRomMemory);

	void ExecuteCycle();

	void ExecuteRendering(const bool bIsRenderingBackdrop);
};
