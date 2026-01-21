#include "PPU.h"

#include "Logger.h"
#include "MemoryMapper.h"
#include "Renderer.h"

#include <cassert>
#include <cstring>

PPU::PPU()
{
	mMemory = {};
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

	mRAM = nullptr;

	mChrRomMemory = nullptr;
}

void PPU::Init(MemoryMapper* RAM, Renderer* RendererPtr, const std::vector<char>* ChrRomMemory)
{
	// What are the real initial values of these?
	mRegisters.v = 0;
	mRegisters.t = 0;
	mRegisters.x = 0;
	mRegisters.w = 0;

	mPPUCTRL = 0;
	mPPUMASK = 0;
	mPPUSTATUS = 0;
	mOAMADDR = 0;
	mPPUSCROLL = 0;
	mPPUADDR = 0;

	mCurrentScanline = PPU_PRE_RENDER_SCANLINE;
	mCurrentDot = 0;

	mbOldNMIRequestFlag = false;

	mbPostFirstPreRenderScanline = false;

	mRAM = RAM;
	mRenderer = RendererPtr;

	mChrRomMemory = ChrRomMemory;
	assert(mChrRomMemory != nullptr && ChrRomMemory->size() <= 8192);

	// Loading CHR into the pattern table range (0x0000 - 0x1FFF)
	// Will need to rework this when Bank Switching is implemented.
	std::memcpy(mMemory.data(), mChrRomMemory->data(), mChrRomMemory->size());
}

void PPU::Execute(const uint8_t CPUCycles)
{
	for (int i = 0; i < CPUCycles; i++)
	{
		// PPU ticks 3x as fast as the CPU.
		ExecuteCycle();
		ExecuteCycle();
		ExecuteCycle();
	}
}

void PPU::ExecuteCycle()
{
	// ??
	// If I don't do this, the CPU never sets VBlank.
	if (mClockCount == REGISTER_IGNORE_CYCLES + 1)
		mbNMIOutputFlag = true;

	// Wasteful to do all these reads, but I feel like it will make it nicer to program
	if (mClockCount > REGISTER_IGNORE_CYCLES)
	{
		mPPUCTRL = mRAM->ReadRegister(PPUCTRL_ADDRESS);
		mPPUMASK = mRAM->ReadRegister(PPUMASK_ADDRESS);
		mPPUSCROLL = mRAM->ReadRegister(PPUSCROLL_ADDRESS);
	}

	mPPUSTATUS = mRAM->ReadRegister(PPUSTATUS_ADDRESS);

	bool bShouldTriggerNMI = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VBLANK_NMI_ENABLE)) != 0;

	bool bVblankFlag = (mPPUSTATUS & static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG)) != 0;
	if (bVblankFlag && bShouldTriggerNMI && mbOldNMIRequestFlag == false)
	{
		mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Triggered NMI!\n");
		// When does this go false?
		mbNMIOutputFlag = true;
	}
	mbOldNMIRequestFlag = bShouldTriggerNMI;

	// Should take effect 4 dots or more after write, otherwise a crash may occur.
	bool bIsRenderingEnabled = (mPPUMASK & PPUMASK_RENDERING_MASK) != 0;

	bool bIsVBLANK = mCurrentScanline >= VBLANK_SCANLINE_RANGE.first && mCurrentScanline < PPU_PRE_RENDER_SCANLINE;

	if (mCurrentScanline == VBLANK_SCANLINE_RANGE.first && mCurrentDot == 1)
	{
		//mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Entered VBlank phase!!\n");

		mPPUSTATUS |= static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG);
		mRAM->WriteRegister(PPUSTATUS_ADDRESS, mPPUSTATUS);

		if (bShouldTriggerNMI)
		{
			mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Triggered NMI!\n");
			mbNMIOutputFlag = true;
		}

		// Upload data to GPU for rendering
		mRenderer->CopyPPUMemory(mPPUCTRL, mMemory, mPalleteMemory, mObjectAttributeMemory);
	}
	else if (mCurrentScanline == PPU_PRE_RENDER_SCANLINE && mCurrentDot == 1)
	{
		mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG);
		mRAM->WriteRegister(PPUSTATUS_ADDRESS, mPPUSTATUS);
	}

	if (bIsVBLANK == false)
	{
		ExecuteRendering(bIsRenderingEnabled);
	}

	uint16_t LastFrameCycle = ENDING_FETCH_PHASE_CYCLES.second;
	bool bIsLastScanline = mCurrentScanline == PPU_PRE_RENDER_SCANLINE;

	// One cycle is skipped for odd frames.
	if (bIsRenderingEnabled && !mbIsEvenFrame && bIsLastScanline)
	{
		LastFrameCycle -= 1;
	}

	mCurrentDot += 1;
	if (mCurrentDot > LastFrameCycle)
	{
		mCurrentDot = 0;
		mCurrentScanline += 1;

		if (mCurrentScanline > PPU_PRE_RENDER_SCANLINE)
		{
			mCurrentScanline = 0;
			mbIsEvenFrame = !mbIsEvenFrame;
		}

		if (mCurrentScanline == PPU_PRE_RENDER_SCANLINE)
		{
			mbPostFirstPreRenderScanline = true;
		}
	}

	mClockCount++;
}

void PPU::ExecuteRendering(const bool bIsRenderingEnabled)
{

}

bool PPU::ReadNMIOutput()
{
	bool bIsNMIEnabled = mbNMIOutputFlag;
	if (mbNMIOutputFlag)
	{
		mbNMIOutputFlag = false;
	}

	return bIsNMIEnabled;
}

uint8_t PPU::ReadOAMDATA()
{
	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU Read PPU OAM DATA! {0:X}, {1:X}\n", mPPUADDR, mObjectAttributeMemory[mOAMADDR]);
	return mObjectAttributeMemory[mOAMADDR];
}

void PPU::WriteOAMADDR(const uint8_t Data)
{
	mOAMADDR = Data;
	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU OAM ADDR! {0:X}\n", Data);
}


void PPU::WriteOAMDATA(const uint8_t Data)
{
	mObjectAttributeMemory[mOAMADDR] = Data;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU OAM DATA! {0:X}, {1:X}\n", mPPUADDR, Data);
	mOAMADDR++;
}

void PPU::WritePPUADDR(const uint8_t Data)
{
	// High first, low second
	const uint8_t WriteData = mRegisters.w ?  Data : uint16_t(Data << 8);
	const uint8_t ClearMask = 0xFFFF << (mRegisters.w ? 0 : 8);

	mPPUADDR &= ~ClearMask;
	mPPUADDR |= WriteData;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU Address! {0:X}\n", mPPUADDR);

	ToggleWRegister();
}

void PPU::WriteData(const uint8_t Data)
{
	mMemory[mPPUADDR] = Data;

	bool bIncrementMode = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VRAM_ADDRESS_INCREMENT)) != 0;

	// 32 Down...Where?
	uint16_t Offset = bIncrementMode ? 32 : 1;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU Data! {0}\n", Offset);

	mPPUADDR += Offset;
}

void PPU::ClearWRegister()
{
	mRegisters.w = false;
}

void PPU::ToggleWRegister()
{
	mRegisters.w = !mRegisters.w;
}
