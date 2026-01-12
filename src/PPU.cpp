#include "PPU.h"

#include "MemoryMapper.h"

#include <cassert>
#include <cstring>
#include <iostream>
#include <format>

PPU::PPU(MemoryMapper* RAM)
{
	// 16KB address space, 0x0000 - 0x3FFF. Accesed by PPU or CPU via memory mapped registers 0x2006 and 0x2007.
	// 0x0000 - 0x1FFF - CHR ROM / CHR RAM, often with bank switching.
	// 0x2000 - 0x2FFF - mapped to NES VRAM, 2 nametables with cartrige controlled mirroring. Can be remaped to ROM or RAM for up to 4 nametables.
	// 0x3000 - 0x3EFF - mirror of 0x2000 - 0x2FFF. PPU doesn't render from this address range.
	// 0x3F00 - 0x3FFF - Not configurable, mapped to internal pallete control.
	mMemory = std::vector<char>(16384);

	mPalleteMemory = {};
	mObjectAttributeMemory = {};

	mMemoryMap = std::array<std::pair<uint16_t, uint16_t>, 13>{{
		{0x0000, 0x1000},
		{0x1000, 0x1000},
		{0x2000, 0x03C0},
		{0x23C0, 0x0040},
		{0x2400, 0x03C0},
		{0x27C0, 0x0040},
		{0x2800, 0x03C0},
		{0x2BC0, 0x0040},
		{0x2C00, 0x03C0},
		{0x2FC0, 0x0040},
		{0x3000, 0x3EFF},
		{0x3F00, 0x0020},
		{0x3F20, 0x00E0}
	}};

	mRAM = RAM;

	mChrRomMemory = nullptr;
}

void PPU::Init(const std::vector<char>* ChrRomMemory)
{
	// What are the real initial values of these?
	mRegisters.v = 0;
	mRegisters.t = 0;
	mRegisters.x = 0;
	mRegisters.w = 0;

	mCurrentScanline = PPU_PRE_RENDER_SCANLINE;
	mCurrentDot = 0;

	mChrRomMemory = ChrRomMemory;
	assert(mChrRomMemory != nullptr && ChrRomMemory->size() <= 8192);
	
	std::memcpy(mMemory.data(), mChrRomMemory->data(), 8192);

}

void PPU::ExecuteCycle()
{
	uint8_t PPUMASK = mRAM->Read8Bit(PPUMASK_ADDRESS);

	// Should take effect 4 dots or more after write, otherwise a crash may occur.
	bool bIsRenderingEnabled = (PPUMASK & PPUMASK_RENDERING_MASK) != 0;

	ExecuteRendering(bIsRenderingEnabled);
}

void PPU::ExecuteRendering(const bool bIsRenderingEnabled)
{

}
