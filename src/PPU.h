#pragma once

#include <array>
#include <vector>

class MemoryMapper;

static const uint16_t PPUCTRL = 0x2000;
static const uint16_t PPUMASK = 0x2001;
static const uint16_t PPUSTATUS = 0x2002;
static const uint16_t OAMADDR = 0x2003;
static const uint16_t PPU_SCROLL_ADDR_LATCH = 0x2004;
static const uint16_t PPUSCROLL = 0x2005;
static const uint16_t PPUADDR = 0x2006;
static const uint16_t PPUDATA = 0x2007;

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
};