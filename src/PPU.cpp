#include "PPU.h"

#include "Logger.h"
#include "MemoryMapper.h"
#include "Renderer.h"
#include "RomParameters.h"

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
}

void PPU::Init(MemoryMapper* RAM, Renderer* RendererPtr, const ROMData* RomDataPtr)
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

	mPPUDataReadBuffer = 0;

	mCurrentScanline = PPU_PRE_RENDER_SCANLINE;
	mCurrentDot = 0;
	mbIsEvenFrame = false;

	mbIsVBlank = false;

	mbOldNMIState = false;
	mbNMIOutputFlag = false;

	mbPostFirstPreRenderScanline = false;

	mRAM = RAM;
	mRenderer = RendererPtr;
	mRomData = RomDataPtr;

	assert(RomDataPtr->ChrRomMemory.size() <= 8192);

	// Loading CHR into the pattern table range (0x0000 - 0x1FFF)
	// Will need to rework this when Bank Switching is implemented.
	std::memcpy(mMemory.data(), RomDataPtr->ChrRomMemory.data(), RomDataPtr->ChrRomMemory.size());
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
	// Should take effect 4 dots or more after write, otherwise a crash may occur.
	bool bIsRenderingEnabled = (mPPUMASK & PPUMASK_RENDERING_MASK) != 0;
	bool bShouldTriggerNMI = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VBLANK_NMI_ENABLE)) != 0;

	if (mCurrentScanline == VBLANK_SCANLINE_RANGE.first && mCurrentDot == 1)
	{
		mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Entered VBlank phase!!\n");
		mbIsVBlank = true;
		mPPUSTATUS |= static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG);

		if (bShouldTriggerNMI)
			mbNMIOutputFlag = true;

		// Upload data to GPU for rendering
		mRenderer->CopyPPUMemory(mPPUCTRL, mMemory, mPalleteMemory, mObjectAttributeMemory);
	}
	else if (mCurrentScanline == PPU_PRE_RENDER_SCANLINE && mCurrentDot == 1)
	{
		mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Exit VBlank phase!!\n");
		mbIsVBlank = false;
		mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG);
		mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::SPRITE_0_HIT_FLAG);
		mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::SPRITE_OVERFLOW_FLAG);
	}

	if (mCurrentScanline <= VISIBLE_SCANLINE_RANGE.second)
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
	// Hack until Sprite 0 hit is implemented, where the PPU checks if an opaque pixel of a sprite overlaps an opaque pixel of a background.
	// Which I imagine I can basically copy paste from my shader.
	if (bIsRenderingEnabled)
	{
		mPPUSTATUS |= static_cast<uint8_t>(EPPUSTATUS::SPRITE_0_HIT_FLAG);
	}
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

void PPU::WritePPUCTRL(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	bool bOldShouldTriggerNMI = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VBLANK_NMI_ENABLE)) != 0;
	bool bNewShouldTriggerNMI = (Data & static_cast<uint8_t>(EPPUCTRL::VBLANK_NMI_ENABLE)) != 0;

	if (bOldShouldTriggerNMI == false && bNewShouldTriggerNMI == true && mbIsVBlank)
	{
		mbNMIOutputFlag = true;
	}

	mPPUCTRL = Data;
}

uint8_t PPU::ReadPPUSTATUS()
{
	uint8_t Value = mPPUSTATUS;

	//if (mPPUSTATUS != 0x40)
		mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::VERBOSE, "CPU Read Status with VBlank! {0:X}\n", mPPUSTATUS);

	mPPUSTATUS &= (~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG));
	mbIsVBlank = false;
	ClearWRegister();

	return Value;
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

void PPU::WriteOAMDMA(const uint8_t Data)
{
	// Read Address is always page-aligned, making the low byte 0x00.
	uint16_t MemoryAddress = uint16_t(Data) << 8;
	uint8_t* RawMemoryAddress = mRAM->GetMemoryPtr(MemoryAddress);

	// TODO: CPU is paused while write happens, which takes ~512-513 cycles. So there may be cases where APU/PPU are supposed to be doing other work in this time.
	std::memcpy(mObjectAttributeMemory.data(), RawMemoryAddress, mObjectAttributeMemory.size());
	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU OAM DMA! {0:X}\n", MemoryAddress);
}

void PPU::WritePPUADDR(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	// High first, low second
	if (!mRegisters.w)
	{
		mNextPPUADDR = uint16_t(Data << 8);
	}
	else
	{
		mPPUADDR = (mNextPPUADDR | Data);
		mRegisters.t = mPPUADDR;
	}

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU Address! {0:X}\n", mPPUADDR);

	ToggleWRegister();
}

uint8_t PPU::ReadPPUData()
{
	uint8_t OldBufferData = mPPUDataReadBuffer;

	if (mPPUADDR < 0x2000)
	{
		mPPUDataReadBuffer = mMemory[mPPUADDR];
	}
	else if (mPPUADDR < 0x3F00)
	{
		if (mRomData->NameTableLayout == ENameTableLayout::Horizontal)
		{
			uint16_t Temp = (mPPUADDR & 0x3FF) | (mPPUADDR & 0x800) >> 1;
			Temp += 0x2000;

			mPPUDataReadBuffer = mMemory[Temp];
		}
		else
		{
			uint16_t Temp = mPPUADDR & 0x7FF;
			Temp += 0x2000;

			mPPUDataReadBuffer = mMemory[Temp];
		}
	}
	else
	{
		uint16_t PalleteMask = (mPPUADDR & 3) == 0 ? 0x0F : 0x1F;
		mPPUDataReadBuffer = mPalleteMemory[mPPUADDR & PalleteMask];
	}

	bool bIncrement32Mode = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VRAM_ADDRESS_INCREMENT)) != 0;

	// 32 Down...Where?
	uint16_t Offset = bIncrement32Mode ? 32 : 1;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU read PPU Data! {0}\n", Offset);

	// PPUADDR is limited to 14 bits.
	mPPUADDR += Offset;
	mPPUADDR &= 0x3FFF;

	return OldBufferData;
}

void PPU::WritePPUData(const uint8_t Data)
{
	if (mPPUADDR < 0x2000)
	{
		if (mRomData->ChrRomMemory.size() == 0)
		{
			mMemory[mPPUADDR] = Data;
		}
	}
	else if (mPPUADDR < 0x3F00)
	{
		if (mRomData->NameTableLayout == ENameTableLayout::Horizontal)
		{
			uint16_t Temp = (mPPUADDR & 0x3FF) | (mPPUADDR & 0x800) >> 1;
			Temp += 0x2000;

			mMemory[Temp] = Data;
		}
		else
		{
			uint16_t Temp = mPPUADDR & 0x7FF;
			Temp += 0x2000;

			mMemory[Temp] = Data;
		}
	}
	else
	{
		uint16_t PalleteMask = (mPPUADDR & 3) == 0 ? 0x0F : 0x1F;
		mPalleteMemory[mPPUADDR & PalleteMask] = Data;
	}

	bool bIncrement32Mode = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VRAM_ADDRESS_INCREMENT)) != 0;

	// 32 Down...Where?
	uint16_t Offset = bIncrement32Mode ? 32 : 1;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU Data! {0}\n", Offset);

	// PPUADDR is limited to 14 bits.
	mPPUADDR += Offset;
	mPPUADDR &= 0x3FFF;
}

void PPU::WritePPUSCROLL(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	mPPUSCROLL = Data;
	ToggleWRegister();
}

void PPU::WritePPUMASK(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	mPPUMASK = Data;
}

void PPU::ClearWRegister()
{
	mRegisters.w = false;
}

void PPU::ToggleWRegister()
{
	mRegisters.w = !mRegisters.w;
}

uint64_t PPU::GetCycleCount() const
{
	return mClockCount;
}
