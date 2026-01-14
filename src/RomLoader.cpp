#include "RomLoader.h"

#include "Logger.h"
#include "RomParameters.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

// Currently only fully supports 2.0 specification, and readiing 1.0 header.
ROMData RomLoader::Load(const std::string& PathToRom)
{
	std::ifstream RomFile(PathToRom, std::ios::binary | std::ios::ate);

	if (!RomFile)
	{
		mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::ERROR, "ROM not found at {0}\n", PathToRom);
		return ROMData();
	}

	std::streamsize Size = RomFile.tellg();
	RomFile.seekg(0, std::ios::beg);

	std::vector<char> FileBuffer(Size);
	if (!RomFile.read(FileBuffer.data(), Size))
	{
		mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::ERROR, "Failed to read ROM file at {0}\n", PathToRom);
		return ROMData();
	}

	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "ROM file size {0}\n", std::to_string(Size));

	std::string RomIdentifier(FileBuffer.data(), 3);
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "ROM Header Identifier: {0}\n", RomIdentifier);

	// Copy pasted NES Format detection from https://www.nesdev.org/wiki/NES_2.0
	bool biNESFormat=false;
	if (FileBuffer[0]=='N' && FileBuffer[1]=='E' && FileBuffer[2]=='S' && FileBuffer[3]==0x1A)
		biNESFormat=true;

	bool bNES20Format=false;
	if (biNESFormat==true && (FileBuffer[7]&0x0C)==0x08)
		bNES20Format=true;

	std::string ROMFormatString = bNES20Format ? "NES 2.0" : "NES 1.0";
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "ROM Format: {0}\n", ROMFormatString);

	ROMData RomData;

	auto LastForwardsSlash = PathToRom.find_last_of("/");
	auto FileExtentionPos = PathToRom.find_last_of(".");
	RomData.Name = PathToRom.substr(LastForwardsSlash + 1, FileExtentionPos - LastForwardsSlash - 1);

	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "Loading ROM: {0}\n", RomData.Name);

	if (bNES20Format)
	{
		RomData = ParseNES20ROM(FileBuffer);
	}
	else if (biNESFormat)
	{
		RomData = ParseNES10ROM(FileBuffer);
	}
	else
	{
		mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::ERROR, "ROM file at {0} is not a supported ROM type\n", PathToRom);
	}

	return RomData;
}

ROMData RomLoader::ParseNES10ROM(const std::vector<char>& FileBuffer)
{
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
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "Last Header Bytes: {0}\n", HeaderPadding);

	int32_t RomParserLocation = 16;

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

	std::vector<char> TrainerArea(0);
	if (bContains512ByteTrainer)
	{
		TrainerArea = std::vector<char>(512);
		std::memcpy(TrainerArea.data(), FileBuffer.data() + RomParserLocation, 512);

		RomParserLocation += 512;
	}

	std::vector<char> PrgRomMemory(PRG_ROM_SIZE);
	if (PRG_ROM_SIZE > 0)
		std::memcpy(PrgRomMemory.data(), FileBuffer.data() + RomParserLocation, PRG_ROM_SIZE);

	RomParserLocation += PRG_ROM_SIZE;

	std::vector<char> ChrRomMemory(CHR_ROM_SIZE);
	if (CHR_ROM_SIZE > 0)
		std::memcpy(ChrRomMemory.data(), FileBuffer.data() + RomParserLocation, CHR_ROM_SIZE);

	RomParserLocation += PRG_ROM_SIZE;

	const int32_t INST_ROM_SIZE = bIsPlayChoice10 ? 8192 : 0;
	std::vector<char> PlayChoiceInstRomMemory(INST_ROM_SIZE);
	if (INST_ROM_SIZE > 0)
		std::memcpy(ChrRomMemory.data(), FileBuffer.data() + RomParserLocation, INST_ROM_SIZE);

	RomParserLocation += PRG_ROM_SIZE;

	uint16_t PlayChoice10PROMData = 0;
	uint16_t PlayChoice10PROMCounterOutput= 0;
	if (INST_ROM_SIZE > 0)
	{
		PlayChoice10PROMData = FileBuffer[RomParserLocation++];
		PlayChoice10PROMData = static_cast<uint16_t>(FileBuffer[RomParserLocation++]) << 8;

		PlayChoice10PROMCounterOutput = FileBuffer[RomParserLocation++];
		PlayChoice10PROMCounterOutput = static_cast<uint16_t>(FileBuffer[RomParserLocation++]) << 8;
	}

	ROMData ROM;
	ROM.CPUTimingMode = bIsNTSC ? ECPU_TIMING::NTSC : ECPU_TIMING::PAL;
	ROM.NameTableLayout = bNametableIsVerticle ? ENameTableLayout::Vertical : ENameTableLayout::Horizontal;
	ROM.NameTableLayout = bUsesAlternativeNametableLayout ? ENameTableLayout::Alternative : ROM.NameTableLayout;

	ROM.PrgRomMemory = PrgRomMemory;
	ROM.ChrRomMemory = ChrRomMemory;
	ROM.PlayChoiceInstRomMemory = PlayChoiceInstRomMemory;
	ROM.Playchoice10PromData = PlayChoice10PROMData;
	ROM.Playchoice10PromCounterOut = PlayChoice10PROMCounterOutput;

	return ROM;
}

ROMData RomLoader::ParseNES20ROM(const std::vector<char>& FileBuffer)
{
	int32_t PrgRomSizeLSB = FileBuffer[4];
	int32_t ChrRomSizeLSB = FileBuffer[5];

	unsigned char Flags6 = FileBuffer[6];
	unsigned char Flags7 = FileBuffer[7];
	unsigned char Flags8 = FileBuffer[8];
	unsigned char Flags9 = FileBuffer[9];
	unsigned char Flags10 = FileBuffer[10];
	unsigned char Flags11 = FileBuffer[11];
	unsigned char Flags12 = FileBuffer[12];
	unsigned char Flags13 = FileBuffer[13];
	unsigned char Flags14 = FileBuffer[14];
	unsigned char Flags15 = FileBuffer[15];

	int32_t RomParserLocation = 16;

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

	// Todo: Console Type Enum
	bool bIsNintendoEntertainmentSystem = false;
	bool bIsVsUnisystem = false;
	bool bIsPlayChoice10 = false;
	bool bIsExtendedConsoleType = false;

	int32_t ConsoleTypeFlags = Flags7 & 0b11;
	switch (ConsoleTypeFlags)
	{
		case 0:
			bIsNintendoEntertainmentSystem = true;
			break;
		case 1:
			bIsVsUnisystem = true;
			break;
		case 2:
			bIsPlayChoice10 = true;
			break;
		case 3:
			bIsExtendedConsoleType = true;
			break;
	}

	bool bIsINES20Format = ((Flags7 & 0b1100) >> 2) == 2;

	// NES 2.0 Changes begin
	int32_t MapperMiddleNibble = (Flags7 & 0b11110000) >> 4;

	int32_t MapperHighNibble = (Flags8 & 0b1111);
	int32_t SubMapperNumber = (Flags8 & 0b11110000) >> 4;

	int32_t PrgRomSizeMSB = (Flags9 & 0b1111);
	int32_t ChrRomSizeMSB = (Flags9 & 0b11110000) >> 4;

	// If the shift count is zero, there is no PRG-(NV)RAM.
	// If the shift count is non-zero, the actual size is
	// "64 << shift count" bytes, i.e. 8192 bytes for a shift count of 7.
	int32_t PrgRamShiftCount = (Flags10 & 0b1111);
	int32_t PrgNvramRamShiftCount = (Flags10 & 0b1111) >> 4;

	// If the shift count is zero, there is no CHR-(NV)RAM.
	// If the shift count is non-zero, the actual size is
	// "64 << shift count" bytes, i.e. 8192 bytes for a shift count of 7.
	int32_t ChrRamShiftCount = (Flags11 & 0b1111);
	int32_t ChrNvramRamShiftCount = (Flags11 & 0b1111) >> 4;

	ECPU_TIMING CPUTimingMode;
	int32_t RomTimingMode = (Flags12 & 0b11);
	switch (RomTimingMode)
	{
		case 0:
			CPUTimingMode = ECPU_TIMING::NTSC;
			break;
		case 1:
			CPUTimingMode = ECPU_TIMING::PAL;
			break;
		case 2:
			// Multiple-region, assuming NTSC
			CPUTimingMode = ECPU_TIMING::NTSC;
			break;
		case 3:
			CPUTimingMode = ECPU_TIMING::DENDY;
			break;
	}

	int32_t VsPPUType = 0;
	int32_t VsHardwareType = 0;
	if (bIsVsUnisystem)
	{
		VsPPUType = (Flags13 & 0b1111);
		VsHardwareType = (Flags13 & 0b11110000) >> 4;
	}

	int32_t ExtendedConsoleType = 0;
	if (bIsExtendedConsoleType)
	{
		ExtendedConsoleType = (Flags13 & 0b1111);
	}

	int32_t MiscellaneousRomCount = (Flags14 & 0b11);

	int32_t DefaultExpansionDevice = (Flags15 & 0b00111111);

	// Contains data to be loaded into CPU memory at $7000.
	std::vector<char> TrainerArea(0);
	if (bContains512ByteTrainer)
	{
		TrainerArea = std::vector<char>(512);
		std::memcpy(TrainerArea.data(), FileBuffer.data() + RomParserLocation, 512);

		RomParserLocation += 512;
	}
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "Trainer Area Size: {0}\n", std::to_string(TrainerArea.size()));

	int32_t PrgRomSize = 0;
	if (const bool bPrgRomSizeExponential = PrgRomSizeMSB == 0xF)
	{
		int32_t PrgRomSizeMultiplier = (PrgRomSizeLSB & 0b11) * 2 + 1;
		int32_t PrgRomSizeExponent = (PrgRomSizeLSB & 0b11111100) >> 2;

		PrgRomSize = std::pow(2, PrgRomSizeExponent) * PrgRomSizeMultiplier;
	}
	else
	{
		PrgRomSize = 16384 * ((PrgRomSizeLSB) | (PrgRomSizeMSB << 8));
	}
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "PRG Rom Size: {0}\n", std::to_string(PrgRomSize));

	// Where does this go? For now...just save it I guess.
	// Todo:: Support Vs. Dual System ROM images.
	std::vector<char> PrgRomMemory(PrgRomSize);
	if (PrgRomSize > 0)
		std::memcpy(PrgRomMemory.data(), FileBuffer.data() + RomParserLocation, PrgRomSize);

	RomParserLocation += PrgRomSize;

	// Where does this go? For now...just save it I guess.
	// Todo:: Support Vs. Dual System ROM images.
	int32_t ChrRomSize = 0;
	if (const bool bChrRomSizeExponential = ChrRomSizeMSB == 0xF)
	{
		int32_t ChrRomSizeMultiplier = (ChrRomSizeLSB & 0b11) * 2 + 1;
		int32_t ChrRomSizeExponent = (ChrRomSizeLSB & 0b11111100) >> 2;

		ChrRomSize = std::pow(2, ChrRomSizeExponent) * ChrRomSizeMultiplier;
	}
	else
	{
		ChrRomSize = 8192 * ((ChrRomSizeLSB) | (ChrRomSizeMSB << 8));
	}
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "CHR Rom Size: {0}\n", std::to_string(ChrRomSize));

	std::vector<char> ChrRomMemory(ChrRomSize);
	if (ChrRomSize > 0)
		std::memcpy(ChrRomMemory.data(), FileBuffer.data() + RomParserLocation, ChrRomSize);

	RomParserLocation += ChrRomSize;

	int32_t MiscellaneousRomAreaSize = FileBuffer.size() - 16 - TrainerArea.size() - PrgRomMemory.size() - ChrRomMemory.size();
	mLog->Log(ELOGGING_SOURCES::ROM_LOADER, ELOGGING_MODE::INFO, "Miscellaneous RomArea Memory size: {0}\n", std::to_string(MiscellaneousRomAreaSize));

	std::vector<char> MiscellaneousRomAreaMemory(MiscellaneousRomAreaSize);

	if (MiscellaneousRomAreaSize > 0)
		std::memcpy(MiscellaneousRomAreaMemory.data(), FileBuffer.data() + RomParserLocation, MiscellaneousRomAreaSize);

	ROMData ROM;
	ROM.CPUTimingMode = CPUTimingMode;

	// TODO: Support other name table layouts once Horizontal/Vertical are implemented.
	ROM.NameTableLayout = bNametableIsVerticle ? ENameTableLayout::Vertical : ENameTableLayout::Horizontal;
	ROM.NameTableLayout = bUsesAlternativeNametableLayout ? ENameTableLayout::Alternative : ROM.NameTableLayout;

	ROM.VsSystemType = static_cast<EVsSystemType>(VsPPUType);
	ROM.VsHardwareType = static_cast<EVsHardwareType>(VsHardwareType);
	ROM.ExtendedConsoleType = static_cast<EExtendedConsoleType>(ExtendedConsoleType);
	ROM.DefaultExpansionDevice = static_cast<EDefaultExpansionDevice>(DefaultExpansionDevice);

	ROM.MiscelaneousROMCount = MiscellaneousRomCount;

	ROM.PrgRomMemory = PrgRomMemory;
	ROM.ChrRomMemory = ChrRomMemory;
	ROM.MiscelaneousRomMemory = MiscellaneousRomAreaMemory;

	return ROM;
}
