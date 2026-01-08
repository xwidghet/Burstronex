#include "RomLoader.h"

#include <fstream>
#include <iostream>
#include <vector>

// Initial version written with INES 1.0 specification. TODO: Update to 2.0 Specification after detecting.
void RomLoader::Load(const std::string& PathToRom)
{
	std::ifstream RomFile(PathToRom, std::ios::binary | std::ios::ate);

	if (!RomFile)
	{
		std::cout << "ROM not found at " + PathToRom << std::endl;
		return;
	}

	std::streamsize Size = RomFile.tellg();
	RomFile.seekg(0, std::ios::beg);

	std::vector<char> FileBuffer(Size);
	if (!RomFile.read(FileBuffer.data(), Size))
	{
		std::cout << "Failed to read ROM file at " + PathToRom << std::endl;
		return;
	}

	std::cout << "ROM file size " + std::to_string(Size) << std::endl;

	std::string RomIdentifier(FileBuffer.data(), 3);
	std::cout << "ROM Header Identifier: " + RomIdentifier << std::endl;

	// 16 KB Units
	int32_t PRG_ROM_SIZE = FileBuffer[4] * 16384;

	// 8 KB Units
	int32_t CHR_ROM_SIZE = FileBuffer[5] * 8192;

	unsigned char Flags6 = FileBuffer[6];
	unsigned char Flags7 = FileBuffer[7];
	unsigned char Flags8 = FileBuffer[8];
	unsigned char Flags9 = FileBuffer[9];
	unsigned char Flags10 = FileBuffer[10];

	// Should be zero, but some rippers may have left identifiers here.
	// Supposedly if bIsINES20Format is false I should refuse to load the ROM if anything is here (Last 4 Bytes).
	std::string HeaderPadding(FileBuffer.data()+10, 5);
	std::cout << "Last Header Bytes: " + HeaderPadding << std::endl;
	
	// TODO: Add Enums for these when implementing the Memory Mapper.

	// 0 == Vertical "Horizontally Mirrored" (CIRAM A10 = PPU A11), 1 == Horizontal "Vertically Mirrored" (CIRAM A10 = PPU A10)
	// bUsesAlternativeNametableLayout overrides this, need to check implementation when this occurs.
	bool bNametableIsVerticle = Flags6 & 0b1;

	// Cartridge contains battery-backed PRG RAM ($6000-7FFF) or other persistent memory
	// Exceptions: UNROM 512 and GTROM use flash memory to store their game state by rewriting the PRG-ROM area
	bool bContainsPersistentMemory = (Flags6 & 0b10) >> 1;

	// 512-byte trainer at $7000-$71FF (stored before PRG data) for Mapper register translation and CHR-RAM caching code. Should be unused on unmodified ROM dumps.
	bool bContains512ByteTrainer = (Flags6 & 0b100) >> 2;
	bool bUsesAlternativeNametableLayout = (Flags6 & 0b1000) >> 3;
	unsigned char MapperNumberLowerNibble = (Flags6 & 0b11110000) >> 4;

	bool bIsVsUnisystem = Flags7 & 0b1;
	bool bIsPlayChoice10 = (Flags7 & 0b10) >> 1;

	bool bIsINES20Format = ((Flags7 & 0b1100) >> 2) == 2;

	// 8KB Units, not widely used. Should override with INES 2.0 data.
	int32_t PrgRamSize = Flags8 * 8192;

	// Other bits unused, most roms don't bother to set this.
	bool bIsNTSC = (Flags9 & 0b1) == 0;

	// PAL == 2, NTSC == 0, Dual Compatability == 1,3
	bIsNTSC = (Flags10 & 0b11) != 2;

	// PRG RAM ($6000-$7FFF), Recent addition so not many roms contain this.
	bool bContainsPrgRAM = (Flags10 & 0b1000) >> 3;

	// Refer to the Bus Conflics page.
	bool bHasBusConflicts = (Flags10 & 0b100000) >> 5;

	std::string ROMFormatString = bIsINES20Format ? "NES 2.0" : "NES 1.0";
	std::cout << "ROM Format: " + ROMFormatString << std::endl;
}