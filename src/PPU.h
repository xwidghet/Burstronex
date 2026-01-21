#pragma once

#include <array>
#include <cstdint>
#include <vector>

class MemoryMapper;
class Renderer;
struct ROMData;

static const uint16_t PPUCTRL_ADDRESS = 0x2000;
static const uint16_t PPUMASK_ADDRESS = 0x2001;
static const uint16_t PPUSTATUS_ADDRESS = 0x2002;
static const uint16_t OAMADDR_ADDRESS = 0x2003;
static const uint16_t OAMDATA_ADDRESS = 0x2004;
static const uint16_t PPUSCROLL_ADDRESS = 0x2005;
static const uint16_t PPUADDR_ADDRESS = 0x2006;

// Writes to this address adds 1 or 32 to register v depending on VRAM incremement bit set in PPUCTRL_ADDRESS
// During rendering (pre-render lines and visibible lines 0-239), triggers both coarse X increment and Y increment (with wrapping).
static const uint16_t PPUDATA_ADDRESS = 0x2007;

static const uint16_t PPU_SCANLINE_COUNT = 262;

// Cycles spent per scanline.
// Each clock produces 1 pixel.
static const uint16_t PPU_SCANLINE_CYCLES = 341;

// CPU should not access PPU memory during this time unless rendering is turned off.
static const std::pair<uint16_t, uint16_t> VISIBLE_SCANLINE_RANGE = {0, 239};

// Idle scanline, nothing happens here.
static const uint16_t POST_RENDER_SCANLINE = 240;

// Occurs at dot one of scanline 241
// Triggers EPPUCTRL's VLANK_NMI_ENABLE flag.
// Enabling while PPUSTATUS's VBLANK flag is enabled immediately triggers NMI.
// PPU doesn't access memory during these scanlines, allowing the CPU to read PPU memory safely.
static const std::pair<uint16_t, uint16_t> VBLANK_SCANLINE_RANGE = {241, 260};

// -1 or 261, 261 is easier to implement.
// Dendy has 51 post-render scanlines instead of 1.
// PAL has 70 VBLANK scanlines instead of 20, but runs 3.2 PPU cycles per CPU cycle.
// For odd frames the last cycle of this scanline is skipped by jumping from (339, 261) to (0,0)
static const uint16_t PPU_PRE_RENDER_SCANLINE = 261;

// If Rendering is enabled
static const uint16_t INCREMENT_V_SCANLINE_DOT = 256;

// If Rendering is enabled, copies all horizontal position bits from register t to register v.
static const uint16_t COPY_T_TO_V_HPOS_SCANLINE_DOT = 257;

// If Rendering is enabled,
// Shortly after VBLANK and the horizontal bits have been copied from t to v at dot 257,
// PPU repeatedly copies vertical bits from t to v during dots 280 to 304, completing full initialization of v from t.
static const std::pair<uint16_t, uint16_t> COPY_T_TO_V_VPOS_SCANLINE_DOT_RANGE = {280, 304};

// If Rendering is enabled,
// Increments the horizontal position in v every 8 dots until dot 256 of the next scanline.
// Across the scanline the coarse X scroll coordiante is incremented repeatedly, wrapping to the next nametable.
static const std::pair<uint16_t, uint16_t> HORIZONTAL_POS_INCREMENT_DOT_RANGE = {328, 256};

// Cycles spent each time a memory fetch is done.
static const uint8_t MEMORY_FETCH_CYCLES = 2;

static const uint16_t IDLE_PHASE_CYCLE = 0;

// Fetches the following per tile, taking 2 cycles per read:
// 1. Nametable byte
// 2, Attribute table byte
// 3. Pattern table tile low
// 4. Pattern table tile high (8 bytes above pattern table tile low address)
//
// Data stored in internal latches, then fed to shift registers every 8 cycles.
// Since it only fetches an attribute byte every 8 cycles, every 8 pixels has the same pallete attribute.
// Shifters are reloaded every 8 cycles, starting at cycle 9.
// Sprite 0 acts as if the image starts at cycle 2, and first pixel is output during cycle 4.
//
// During this time, Sprite Evaluation for the next scanline is taking place independently.
static const std::pair<uint16_t, uint16_t> TILE_FETCH_PHASE_CYCLES = {1, 256};

// Fetches the following per tile, taking 2 cycles per read, 4 for each of the 8 sprites:
// 1. Garbage Nametable byte
// 2, Garbage Attribute table byte
// 3. Pattern table tile low
// 4. Pattern table tile high (8 bytes above pattern table tile low address)
//
// If there are less than 8 sprites on the next scanline, dummy fetches to 0xFF occur for the empty sprite slots.
// This data is discarded, and replaced with transparent values.
static const std::pair<uint16_t, uint16_t> NEXT_SCANLINE_SPRITE_FETCH_PHASE_CYCLES = {257, 320};

// Fetches the following per tile, for the next scanline's first to tiles, taking 2 cycles per read:
// 1. Nametable byte
// 2, Attribute table byte
// 3. Pattern table tile low
// 4. Pattern table tile high (8 bytes above pattern table tile low address)
static const std::pair<uint16_t, uint16_t> NEXT_SCANLINE_FIRST_TWO_TILES_FETCH_PHASE_CYCLES = {321, 336};

// Two bytes fetched, 2 PPU cycles each:
// 1. Nametable byte
// 2. Nametable byte
// Purpose unknown but Mapper MMC5 uses these fetches for clocking a scanline counter.
static const std::pair<uint16_t, uint16_t> ENDING_FETCH_PHASE_CYCLES = {337, 340};

// SCROLLING
// two fine offsets specify the part of an 8x8 tile each pixel falls on.
// two coarse offsets specify which tiles.

// High 5 bits of X and Y scroll settings are sent to PPUSCROLL_ADDRESS, combined with 2 nametable select bits in PPUCTRL_ADDRESS,
// make the 12 bitaddress for the next tile to be fetched within the nametable address space 0x2000-0x2FFF.
// If set before end of VBLANK, the 12 bit address is loaded into register v when needed to fetch the tile for the top left pixel to render.

// Low 3 bits of X sent to PPUSCROLL_ADDRESS (First write) controls the fine pixel offset within the 8x8 tile.
// These bits go into the x register, which selects one of the 8 pixels coming out of a set of shift registers.
// This value does not change during rendering, only the first PPUSCROLL_ADDRESS write.

// Low 3 bits of Y sent to PPUSCROLL_ADDRESS (Second write) controls the vertical offset within the 8x8 tile.
// The low 3 bits goes into the high 3 bits of the v register.
// During rendering the 3 bits in the v register count the lines until the course Y memory address needs to be incremented (and wrapped if needed)

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

const uint8_t BASE_NAMETABLE_ADDRESS_MASK = 0b00000011;

// Writes to this register are ignored until first pre-render scanline
// Ram Address 0x2000 (Write)
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

// Background or Sprite Rendering enabled.
const uint8_t PPUMASK_RENDERING_MASK = 0b11000;

// Writes to this register are ignored until first pre-render scanline
// Commonly 0x00 outside of gameplay, and 0x1E during gameplay
// Ram Address 0x2001 (Write)
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

// Ram Address 0x2002 (Read)
// Reading this address also clears the PPU's w register.
enum class EPPUSTATUS : uint8_t {
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

// Ram Address 0x2003 (Write)
enum class EOAMADDR {
	// Address at the OAM the CPU wants to access. Most games write 0x00 and use OAMDMA
	// Set to 0 during ticks 257-320 (sprite tile loading interval) of the pre-render and visible scanlines.
	// If rendering is enabled mid-scanline then this value likely doesn't correctly point to the correct address, misaligning all reads.
	ADDRESS = 0b11111111
};

// Ram Address 0x2004 (Read/Write)
enum class EOAMDATA {
	// Writes increment OAMADDR, reads do not.
	// Reads during vertical or forced blanking return the OAM at the current OAMADDR(?)
	ADDRESS = 0b11111111
};

// Ram Address 0x2005 (Write)
enum class EPPUSCROLL {
	// First Write
	// X scroll Byte 7-0, (bit 8 in PPUCTRL bit 0)
	//
	// Second Write
	// Y scroll Byte 7-0, (bit 8 in PPUCTRL bit 1)
	//
	// Controls which pixels of the nametable selected through PPUCTRL is at the top left corner of the rendered screen.
	// Typically written during VBLANK, but sometimes mid-render for split-screen.
	// Vertical scroll changes only take effect next frame.
	// Scroll components (X, Y) are 9 bits total when combined with the nametable bits in PPUCTRL.
	SCROLLBITS = 0b11111111
};

// Ram Address 0x2006 (Write)
enum class EPPUADDR {
	// First Write
	// VRAM Address bits 0-7
	VRAM_ADDRESS_Write_1 = 0b11111111,
	// Second Write
	// VRAM Address bits 8-13?
	VRAM_ADDRESS_Write_2 = 0b00111111,
};

// Ram Address 0x2007 (Read/Write)
enum class EPPUDATA {
	VRAM_DATA = 0b11111111
};

// Ram Address 0x4014 (Write)
enum class EOAMDMA {
	// Source page (High Byte of source address)
	SPRITE_DMA = 0b11111111
};

// Times three to avoid needing to read CPU clock count.
const uint32_t REGISTER_IGNORE_CYCLES = 29658 * 3;


class PPU {
	struct PPUREGISTERS {
		// During Rendering, used for scroll position.
		// Outside Rendering, used for VRAM address.
		// 15 bits
		uint16_t v;

		// During Rendering, starting coarse x-scroll for next scanline and starting y scroll for the screen.
		// Outside Rendering, holds the scroll or VRAM addres before transfering to v register.
		// 15 bits
		uint16_t t;

		// Fine x-position of the current scroll, used along side v during rendering.
		// 3 bits
		uint8_t x;

		// Toggles on each write to PPUSCROLL or PPUADDR, indicating if it is the first or second write.
		// Clears on reads of PPUSTATUS. Refered to as 'write latch' or 'write toggle'.
		// 1 bit
		bool w;
	} mRegisters;

	// Registers memory mapped to CPU RAM
	uint8_t mPPUCTRL;
	uint8_t mPPUMASK;
	uint8_t mPPUSTATUS;
	uint8_t mOAMADDR;
	uint8_t mPPUSCROLL;
	uint16_t mPPUADDR;

	uint16_t mNextPPUADDR = 0;

	uint8_t mPPUDataReadBuffer = 0;

	// 16KB address space, 0x0000 - 0x3FFF. Accesed by PPU or CPU via memory mapped registers 0x2006 and 0x2007.
	// 0x0000 - 0x1FFF - CHR ROM / CHR RAM, often with bank switching.
	// 0x2000 - 0x2FFF - mapped to NES VRAM, 2 nametables with cartrige controlled mirroring. Can be remaped to ROM or RAM for up to 4 nametables.
	// 0x3000 - 0x3EFF - mirror of 0x2000 - 0x2FFF. PPU doesn't render from this address range.
	// 0x3F00 - 0x3FFF - Not configurable, mapped to internal pallete control.
	std::array<uint8_t, 16384> mMemory;

	// 4 Palletes, first 16 are background tiles, while last 16 are sprites.
	// Entry 0 of pallete 0 is the backdrop color.
	// Entry 0 of each pallete is transparent, so color values of these is ignored.
	// Backdrop color is displayed when both background and sprites at this pixel are transparent.
	std::array<uint8_t, 32> mPalleteMemory;

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
	std::array<uint8_t, 256> mObjectAttributeMemory;

	// All 12 memory regions stored (Address Begin, size)
	std::array<std::pair<uint16_t, uint16_t>, 13> mMemoryMap;

	uint16_t mCurrentScanline = PPU_PRE_RENDER_SCANLINE;
	uint16_t mCurrentDot = 0;

	// Toggled every frame regardless of if rendering is enabled.
	bool mbIsEvenFrame = true;

	bool mbNMIOutputFlag = false;

	bool mbNMIState = false;
	bool mbOldNMIState = false;

	bool mbIsVBlank = false;

	bool mbPostFirstPreRenderScanline = false;

	uint64_t mClockCount = 0;

	// CPU Address space, the PPU's Registers exist in the memory range 0x2000 to 0x2007, mirrored up to 0x3FFF.
	// 
	// PPU ignores writes (always 00?) for registers 0x2000, 0x2001, 0x2005, and 0x2006) 
	// until reaching the first pre-render scanline of next frame, aka ~29658 NTSC CPU or 33132 PAL CPU Cycles.
	MemoryMapper* mRAM = nullptr;

	const ROMData* mRomData;

	Renderer* mRenderer = nullptr;

	void ExecuteCycle();

	void ExecuteRendering(const bool bIsRenderingEnabled);

public:
	PPU();

	void Init(MemoryMapper* RAM, Renderer* RendererPtr, const ROMData* RomDataPtr);

	void Execute(const uint8_t CPUCycles);

	bool ReadNMIOutput();

	void WritePPUCTRL(const uint8_t Data);

	uint8_t ReadPPUSTATUS();

	uint8_t ReadOAMDATA();

	void WriteOAMADDR(const uint8_t Data);

	void WriteOAMDATA(const uint8_t Data);

	void WritePPUADDR(const uint8_t Data);

	uint8_t ReadPPUData();

	void WritePPUData(const uint8_t Data);

	void ClearWRegister();

	void ToggleWRegister();

	uint64_t GetCycleCount() const;
};
