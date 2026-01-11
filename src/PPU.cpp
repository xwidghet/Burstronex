#include "PPU.h"

#include "MemoryMapper.h"

#include <cassert>
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

	mRAM = RAM;
}

void PPU::Init(const std::vector<char>* ChrRomMemory)
{
	assert(mChrRomMemory != nullptr && ChrRomMemory->size() <= 8192);
	mChrRomMemory = ChrRomMemory;
	
	std::memcpy(mMemory.data(), mChrRomMemory->data(), 8192);

}

void PPU::ExecuteCycle()
{

}