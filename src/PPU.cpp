#include "PPU.h"

#include "Logger.h"
#include "MemoryMapper.h"
#include "Renderer.h"
#include "RomParameters.h"
#include "StatisticsManager.h"

#include <cassert>
#include <cstring>

PPU::PPU()
{
	mMemory = {};
	mChrMemory = {};
	mPalleteMemory = {};
	mObjectAttributeMemory = {};
	mSecondaryObjectAttributeMemory = {};
	mBackgroundDrawData = {};

	mPPUStatistics = {};

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

	mPPUDataReadBuffer = 0;

	mCurrentScanline = PPU_PRE_RENDER_SCANLINE;
	mCurrentDot = 0;
	mbIsEvenFrame = false;

	mbIsVBlank = false;

	mRAM = RAM;
	mRenderer = RendererPtr;
	mRomData = RomDataPtr;

	assert(RomDataPtr->ChrRomMemory.size() <= 8192);

	// Loading CHR into the pattern table range (0x0000 - 0x1FFF)
	// Will need to rework this when Bank Switching is implemented.
	std::memcpy(mChrMemory.data(), RomDataPtr->ChrRomMemory.data(), RomDataPtr->ChrRomMemory.size());

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

	ExecuteScanlineLogic();
	ExecuteDotLogic(bIsRenderingEnabled);

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
	}

	mClockCount++;
}

void PPU::ExecuteScanlineLogic()
{
	// Technically this occurs on dot 0, but the effects are delayed to dot 1.
	if (const bool bScanlineTransitioned = mCurrentDot == 1)
	{
		if (mCurrentScanline == VBLANK_SCANLINE_RANGE.first)
		{
			mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Entered VBlank phase!!\n");
			mbIsVBlank = true;
			mPPUSTATUS |= static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG);

			// Upload data to GPU for rendering
			mRenderer->CopyPPUMemory(mPPUCTRL, mBackgroundDrawData, mChrMemory, mMemory, mPalleteMemory, mObjectAttributeMemory);

			mStatisticsManager->UpdatePPUStatistics(mPPUStatistics);
			mPPUStatistics.mStatusCallsSinceVBlank = 0;
		}
		else if (mCurrentScanline == PPU_PRE_RENDER_SCANLINE)
		{
			mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "PPU: Exit VBlank phase!!\n");
			mbIsVBlank = false;
			mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG);
			mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::SPRITE_0_HIT_FLAG);
			mPPUSTATUS &= ~static_cast<uint8_t>(EPPUSTATUS::SPRITE_OVERFLOW_FLAG);
		}
	}
}

void PPU::ExecuteDotLogic(const bool bIsRenderingEnabled)
{
	if (mCurrentDot == IDLE_PHASE_CYCLE)
		return;

	if (bIsRenderingEnabled && IsInScanlineRange(VISIBLE_SCANLINE_RANGE))
	{
		SpriteEvaluation();
	}

	if (IsInScanlineRange(VISIBLE_SCANLINE_RANGE) || (mCurrentScanline == PPU_PRE_RENDER_SCANLINE))
	{
		PrepareNextStrip();
	}

	if (mCurrentScanline <= VISIBLE_SCANLINE_RANGE.second)
	{
		ExecuteRendering(bIsRenderingEnabled);
	}
}

void PPU::SpriteEvaluation()
{
	if (mCurrentDot > 0 && mCurrentDot <= 64)
	{
		if (mCurrentDot == 1)
		{
			mSpriteEvaluationIndex = 0;
			mbSecondaryObjectAttributeMemoryFull = false;
			mbSpriteEvaluationOverflowed = false;
		}

		if ((mCurrentDot & 1) == 1)
		{
			mSpriteEvaluationTemp = 0xFF;
		}
		else
		{
			mSecondaryObjectAttributeMemory[mSpriteEvaluationIndex] = mSpriteEvaluationTemp;
			mSpriteEvaluationIndex = (mSpriteEvaluationIndex + 1) & 0x1F;
		}
	}
	else if (mCurrentDot > 64 && mCurrentDot <= 256)
	{
		if ((mCurrentDot & 1) == 1)
		{
			mSpriteEvaluationTemp = mObjectAttributeMemory[mOAMADDR];
		}
		else
		{
			if (mbSpriteEvaluationOverflowed == false)
			{
				if (mbSecondaryObjectAttributeMemoryFull == false)
				{
					mSecondaryObjectAttributeMemory[mSpriteEvaluationIndex] = mSpriteEvaluationTemp;
				}

				if (mSpriteEvaluationStep == 0)
				{
					uint16_t SpriteSize = (mPPUCTRL & (1 << 5)) != 0 ? 16 : 8;
					if (((mCurrentScanline - mSpriteEvaluationTemp) >= 0) && ((mCurrentScanline - mSpriteEvaluationTemp) < SpriteSize))
					{
						if (mbSecondaryObjectAttributeMemoryFull == false)
						{
							mSpriteEvaluationIndex++;
							mOAMADDR++;
							if (mCurrentDot == 66)
							{
								mbScanlineContainsSprite0 = true;
							}
						}
						else
						{
							mPPUSTATUS |= static_cast<uint8_t>(EPPUSTATUS::SPRITE_OVERFLOW_FLAG);
						}

						mSpriteEvaluationStep = (mSpriteEvaluationStep + 1) & 3;
					}
					else
					{
						mOAMADDR += 4;
					}
				}
				else
				{
					mSpriteEvaluationIndex++;
					mOAMADDR++;
					if (mSpriteEvaluationIndex == 0x20)
					{
						mbSecondaryObjectAttributeMemoryFull = true;
					}
					mSpriteEvaluationStep = (mSpriteEvaluationStep + 1) & 3;
				}

				if (mOAMADDR == 0)
				{
					mbSpriteEvaluationOverflowed = true;
				}
			}
		}
	}
	else if (mCurrentDot > 256 && mCurrentDot <= 320)
	{
		mOAMADDR = 0;

		if (mCurrentDot == 257)
		{
			mSecondaryObjectAttributeMemoryCount = mSpriteEvaluationIndex;
			mSpriteEvaluationIndex = 0;
			mSpriteEvaluationStep = 0;
		}

		switch (mSpriteEvaluationStep)
		{
			case 0:
				mSpriteShiftRegisters.SpriteYPosition[mSpriteEvaluationIndex/4] = mSecondaryObjectAttributeMemory[mSpriteEvaluationIndex];
				mSpriteEvaluationIndex++;
				break;
			case 1:
				mSpriteShiftRegisters.SpritePattern[mSpriteEvaluationIndex/4] = mSecondaryObjectAttributeMemory[mSpriteEvaluationIndex];
				mSpriteEvaluationIndex++;
				break;
			case 2:
				mSpriteShiftRegisters.SpriteAttribute[mSpriteEvaluationIndex/4] = mSecondaryObjectAttributeMemory[mSpriteEvaluationIndex];
				mSpriteEvaluationIndex++;
				break;
			case 3:
				mSpriteShiftRegisters.SpriteXPosition[mSpriteEvaluationIndex/4] = mSecondaryObjectAttributeMemory[mSpriteEvaluationIndex];
				break;
			case 4:
				mSpriteStripAddress = CalculateSpriteAddress(mSpriteEvaluationIndex/4);
				break;
			case 5:
				mSpriteStripTemp = ReadPPUMemory(mSpriteStripAddress);
				if (mCurrentScanline == 261)
				{
					mSpriteStripTemp = 0;
				}
				if (const bool bFlipHorizontal = ((mSpriteShiftRegisters.SpriteAttribute[mSpriteEvaluationIndex/4] >> 6) & 1) == 1)
				{
					// Reverse bit order
					mSpriteStripTemp = ((mSpriteStripTemp & 0xF0) >> 4) | ((mSpriteStripTemp & 0xF) << 4);
					mSpriteStripTemp = ((mSpriteStripTemp & 0xCC) >> 2) | ((mSpriteStripTemp & 0x33) << 2);
					mSpriteStripTemp = ((mSpriteStripTemp & 0xAA) >> 1) | ((mSpriteStripTemp & 0x55) << 1);
				}
				mSpriteShiftRegisters.SpriteShiftRegisterLow[mSpriteEvaluationIndex/4] = mSpriteStripTemp;
				break;
			case 6:
				mSpriteStripAddress += 8;
				break;
			case 7:
				mSpriteStripTemp = ReadPPUMemory(mSpriteStripAddress);
				if (mCurrentScanline == 261)
				{
					mSpriteStripTemp = 0;
				}
				if (const bool bFlipHorizontal = ((mSpriteShiftRegisters.SpriteAttribute[mSpriteEvaluationIndex/4] >> 6) & 1) == 1)
				{
					// Reverse bit order
					mSpriteStripTemp = ((mSpriteStripTemp & 0xF0) >> 4) | ((mSpriteStripTemp & 0xF) << 4);
					mSpriteStripTemp = ((mSpriteStripTemp & 0xCC) >> 2) | ((mSpriteStripTemp & 0x33) << 2);
					mSpriteStripTemp = ((mSpriteStripTemp & 0xAA) >> 1) | ((mSpriteStripTemp & 0x55) << 1);
				}
				mSpriteShiftRegisters.SpriteShiftRegisterHigh[mSpriteEvaluationIndex/4] = mSpriteStripTemp;
				mSpriteEvaluationIndex++;
				break;
		}
		mSpriteEvaluationStep = (mSpriteEvaluationStep + 1) & 7;
	}
}

uint16_t PPU::CalculateSpriteAddress(uint8_t Index) const
{
	if (const bool bIs8x8SpriteMode = (mPPUCTRL & (1 << 5)) == 0)
	{
		uint16_t SpritePatternTableAddress = 0x1000 * uint16_t((mPPUCTRL & (1 << 3)) != 0);
		uint16_t SpritePatternAddress = (mSpriteShiftRegisters.SpritePattern[Index] << 4);
		uint16_t ScanlineOffset = (mCurrentScanline - mSpriteShiftRegisters.SpriteYPosition[Index]);

		if (const bool bIsFlipped = ((mSpriteShiftRegisters.SpriteAttribute[Index] >> 7) & 1) != 0)
		{
			ScanlineOffset = (7-ScanlineOffset) & 7;
		}

		return SpritePatternTableAddress + SpritePatternAddress + ScanlineOffset;
	}
	else
	{
		if (const bool bIsNotFlipped = ((mSpriteShiftRegisters.SpriteAttribute[Index] >> 7) & 1) == 0)
		{
			uint16_t SpritePatternTableAddress = (mSpriteShiftRegisters.SpritePattern[Index] & 1) == 1 ? 0x1000 : 0;
			uint16_t SpritePatternAddress = ((mSpriteShiftRegisters.SpritePattern[Index] & 0xFE) << 4);
			uint16_t ScanlineOffset = (mCurrentScanline - mSpriteShiftRegisters.SpriteYPosition[Index]);

			SpritePatternAddress += ScanlineOffset < 8 ? 0 : 16;
			ScanlineOffset = ScanlineOffset < 8 ? ScanlineOffset : (ScanlineOffset & 7);

			return (SpritePatternTableAddress | SpritePatternAddress) - ScanlineOffset;
		}
		else
		{
			uint16_t SpritePatternTableAddress = (mSpriteShiftRegisters.SpritePattern[Index] & 1) == 1 ? 0x1000 : 0;
			uint16_t SpritePatternAddress = ((mSpriteShiftRegisters.SpritePattern[Index] & 0xFE) << 4);
			uint16_t ScanlineOffset = (mCurrentScanline - mSpriteShiftRegisters.SpriteYPosition[Index]);

			SpritePatternAddress += ScanlineOffset < 8 ? 16 : 7;
			ScanlineOffset = ScanlineOffset < 8 ? ((ScanlineOffset & 7) + 7) : (ScanlineOffset & 7);

			return (SpritePatternTableAddress | SpritePatternAddress) - ScanlineOffset;
		}
	}
}

void PPU::PrepareNextStrip()
{
	if (IsInDotRange(TILE_FETCH_PHASE_CYCLES) || IsInDotRange(NEXT_SCANLINE_FIRST_TWO_TILES_FETCH_PHASE_CYCLES))
	{
		bool bBackgroundRendering = (mPPUMASK & static_cast<uint8_t>(EPPUMASK::ENABLE_BACKGROUND_RENDERING)) != 0;
		bool bRenderSprites = (mPPUMASK & static_cast<uint8_t>(EPPUMASK::ENABLE_SPRITE_RENDERING)) != 0;

		if (bBackgroundRendering || bRenderSprites)
		{
			if (bBackgroundRendering)
			{
				mShiftRegisters.mPatternLow <<= 1;
				mShiftRegisters.mPatternHigh <<= 1;
				mShiftRegisters.mAttributeLow <<= 1;
				mShiftRegisters.mAttributeHigh <<= 1;
			}
			if (bBackgroundRendering || bRenderSprites)
			{
				if (mCurrentDot > 1 && mCurrentDot <= 256)
				{
					for (int i = 0; i < 8; i++)
					{
						if (mSpriteShiftRegisters.SpriteXPosition[i] > 0)
						{
							mSpriteShiftRegisters.SpriteXPosition[i]--;
						}
						else
						{
							mSpriteShiftRegisters.SpriteShiftRegisterLow[i] <<= 1;
							mSpriteShiftRegisters.SpriteShiftRegisterHigh[i] <<= 1;
						}
					}
				}
			}

			uint16_t BackgroundPatternTableAddress = 0x1000 * ((mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::BACKGROUND_PATTERN_ADDRESS)) != 0);

			switch ((mCurrentDot-1) & 7)
			{
				case 0:
					mShiftRegisters.mPatternLow = (mShiftRegisters.mPatternLow & 0xFF00) | mPreparedStrip.mPatternLow;
					mShiftRegisters.mPatternHigh = (mShiftRegisters.mPatternHigh & 0xFF00) | mPreparedStrip.mPatternHigh;
					mShiftRegisters.mAttributeLow = (mShiftRegisters.mAttributeLow & 0xFF00) | ((mPreparedStrip.mAttribute & 1) == 1 ? 0xFF : 0);
					mShiftRegisters.mAttributeHigh = (mShiftRegisters.mAttributeHigh & 0xFF00) | ((mPreparedStrip.mAttribute & 2) == 2 ? 0xFF : 0);


					mStripAddress = (0x2000 + (mRegisters.v & 0x0FFF));
					mStripDataTemp = ReadPPUMemory(mStripAddress);
					break;
				case 1:
					mStripDataNext = mStripDataTemp;
					break;
				case 2:
					mStripAddress = ((0x23C0 | (mRegisters.v & 0x0C00) | ((mRegisters.v >> 4) & 0x38) | ((mRegisters.v  >> 2) & 0x07)));
					mStripDataTemp = ReadPPUMemory(mStripAddress);
					break;
				case 3:
					mPreparedStrip.mAttribute = mStripDataTemp;

					// Right Tile
					if ((mRegisters.v & 3) >= 2)
					{
						mPreparedStrip.mAttribute >>= 2;
					}
					// Bottom Tile
					if ((((mRegisters.v & 0b0000001111100000) >> 5) & 3) >= 2)
					{
						mPreparedStrip.mAttribute >>= 4;
					}
					mPreparedStrip.mAttribute &= 3;
					break;
				case 4:
					mStripAddress = (((mRegisters.v & 0b0111000000000000) >> 12) | (mStripDataNext*16) | (BackgroundPatternTableAddress));
					mStripDataTemp = ReadPPUMemory(mStripAddress);
					break;
				case 5:
					mPreparedStrip.mPatternLow = mStripDataTemp;
					mStripAddress += 8;
					break;
				case 6:
					mStripDataTemp = ReadPPUMemory(mStripAddress);
					break;
				case 7:
					mPreparedStrip.mPatternHigh = mStripDataTemp;

					// If we've exited the current nametable block
					if ((mRegisters.v & 0x001F) == 31)
					{
						// Reset Scroll
						mRegisters.v &= 0xFFE0;

						// Jump Nametable
						mRegisters.v ^= 0x0400;
					}
					else
					{
						mRegisters.v++;
					}
					break;
			}
		}
	}
}

void PPU::ExecuteRendering(const bool bIsRenderingEnabled)
{
	if (bIsRenderingEnabled)
	{
		if (IsInDotRange(INCREMENT_V_SCANLINE_DOT))
		{
			IncrementScrollY();
		}
		else if (IsInDotRange(COPY_T_TO_V_HPOS_SCANLINE_DOT))
		{
			ResetScrollX();
		}
		if (IsInDotRange(COPY_T_TO_V_VPOS_SCANLINE_DOT_RANGE) && (mCurrentScanline == PPU_PRE_RENDER_SCANLINE))
		{
			ResetScrollY();
		}
	}

	if ((mCurrentScanline < VBLANK_SCANLINE_RANGE.first) && IsInDotRange(TILE_FETCH_PHASE_CYCLES))
	{
		uint8_t PalleteLow = 0;
		uint8_t PalleteHigh = 0;

		bool bRenderBackground = (mPPUMASK & static_cast<uint8_t>(EPPUMASK::ENABLE_BACKGROUND_RENDERING)) != 0;
		bool bRenderBackgroundL8px = (mPPUMASK & static_cast<uint8_t>(EPPUMASK::BACKGROUND_LEFTMOST_8PX)) != 0;
		if (bRenderBackground && (mCurrentDot > 8 || bRenderBackgroundL8px))
		{
			uint8_t Column0 = ((mShiftRegisters.mPatternLow >> (15 - mRegisters.x)) & 1);
			uint8_t Column1 = ((mShiftRegisters.mPatternHigh >> (15 - mRegisters.x)) & 1);
			PalleteLow = (Column1 << 1) | Column0;

			uint8_t Pallete0 = ((mShiftRegisters.mAttributeLow >> (15 - mRegisters.x)) & 1);
			uint8_t Pallete1 = ((mShiftRegisters.mAttributeHigh >> (15 - mRegisters.x)) & 1);
			PalleteHigh = (Pallete1 << 1) | Pallete0;

			// Index 0 of all color palletes is a mirror of the background color which is the first color.
			if ((PalleteLow == 0) && (PalleteHigh != 0))
			{
				PalleteHigh = 0;
			}
		}

		bool bRenderSprites = (mPPUMASK & static_cast<uint8_t>(EPPUMASK::ENABLE_SPRITE_RENDERING)) != 0;
		bool bRenderSpriteL8px = (mPPUMASK & static_cast<uint8_t>(EPPUMASK::SPRITES_LEFTMOST_8PX)) != 0;
		uint8_t SpritePalleteLow = 0;
		uint8_t SpritePalleteHigh = 0;
		bool bSpritePriority = false;

		if (bRenderSprites && (mCurrentDot > 8 || bRenderSpriteL8px))
		{
			for (int i = 0; i < 8; i++)
			{
				if (mSpriteShiftRegisters.SpriteXPosition[i] == 0 && i < (mSecondaryObjectAttributeMemoryCount / 4))
				{
					SpritePalleteLow = (mSpriteShiftRegisters.SpriteShiftRegisterLow[i] & 0x80) != 0;
					SpritePalleteLow |= 2*((mSpriteShiftRegisters.SpriteShiftRegisterHigh[i] & 0x80) != 0);

					SpritePalleteHigh = (mSpriteShiftRegisters.SpriteAttribute[i] & 0x3) | 0x4;
					bSpritePriority = ((mSpriteShiftRegisters.SpriteAttribute[i] >> 5) & 1) == 0;
				}
				else
				{
					continue;
				}
				if (SpritePalleteLow != 0)
				{
					if (mCurrentDot < 256)
					{
						const bool bOverlappingBackground = i == 0 && mbScanlineContainsSprite0 && SpritePalleteLow != 0 && PalleteLow != 0 && bRenderBackground;
						if (bOverlappingBackground)
						{
							mPPUSTATUS |= static_cast<uint8_t>(EPPUSTATUS::SPRITE_0_HIT_FLAG);
						}
					}
					break;
				}
			}
		}

		if ((bSpritePriority && SpritePalleteLow != 0) || PalleteLow == 0)
		{
			PalleteLow = SpritePalleteLow;
			PalleteHigh = SpritePalleteHigh;
			if (PalleteLow == 0)
			{
				PalleteHigh = 0;
			}
		}

		// Shove into array, copy to GPU, and should be able to directly read this -> Read pallete color
		// May need attributes, but I guess if the tile is 0, we know it's a transparent tile pixel?
		// Since this is 4 bits I could shove it into one, but for simplicity for now I'll leave the variables packed into uint16_t
		int32_t X = mCurrentDot-1;
		int32_t Y = int32_t(mCurrentScanline)*256;
		int32_t WritePos = X + Y;

		mBackgroundDrawData[WritePos] = (uint16_t(PalleteHigh) << 8) | uint16_t(PalleteLow);
	}
}

bool PPU::IsInScanlineRange(const std::pair<uint16_t, uint16_t>& Range) const
{
	return (mCurrentScanline >= Range.first) && (mCurrentScanline <= Range.second);
}

bool PPU::IsInDotRange(const std::pair<uint16_t, uint16_t>& Range) const
{
	return (mCurrentDot >= Range.first) && (mCurrentDot <= Range.second);
}

uint8_t PPU::ReadPPUMemory(uint16_t Address)
{
	if (Address < 0x2000)
	{
		return mChrMemory[Address];
	}
	else if (Address < 0x3F00)
	{
		if (mRomData->NameTableLayout == ENameTableLayout::Horizontal)
		{
			return mMemory[(Address & 0x3FF) | (Address & 0x800) >> 1];
		}
		else
		{
			return mMemory[Address & 0x7FF];
		}
	}
	else
	{
		uint16_t PalleteMask = (Address & 3) == 0 ? 0x0F : 0x1F;
		return mPalleteMemory[Address & PalleteMask];
	}
}

void PPU::IncrementScrollY()
{
	if ((mRegisters.v & 0x7000) != 0x7000)
	{
		mRegisters.v += 0x1000;
	}
	else
	{
		mRegisters.v &= 0x0FFF;
		uint16_t Y = (mRegisters.v & 0x03E0) >> 5;
		if (Y == 29)
		{
			Y = 0;
			mRegisters.v ^= 0x0800;
		}
		else
		{
			Y++;
			Y &= 0x1F;
		}

		mRegisters.v = ((mRegisters.v & 0xFC1F) | (Y << 5));
	}
}

void PPU::ResetScrollY()
{
	mRegisters.v = (mRegisters.v & 0b0000010000011111) | (mRegisters.t & 0b0111101111100000);
}

void PPU::ResetScrollX()
{
	mRegisters.v = (mRegisters.v & 0b0111101111100000) | (mRegisters.t & 0b0000010000011111);
}

bool PPU::ReadNMIOutput()
{
	bool bIsNMIEnabled = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VBLANK_NMI_ENABLE)) != 0;

	return bIsNMIEnabled && mbIsVBlank;
}

void PPU::WritePPUCTRL(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	mPPUCTRL = Data;
}

uint8_t PPU::ReadPPUSTATUS()
{
	uint8_t Value = mPPUSTATUS;

	//if (mPPUSTATUS != 0x40)
		mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU Read Status with VBlank! {0:X}\n", mPPUSTATUS);

	mPPUSTATUS &= (~static_cast<uint8_t>(EPPUSTATUS::VBLANK_FLAG));
	mbIsVBlank = false;
	ClearWRegister();

	mPPUStatistics.mStatusCallsSinceVBlank++;

	return Value;
}

void PPU::WriteOAMADDR(const uint8_t Data)
{
	mOAMADDR = Data;
	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU OAM ADDR! {0:X}\n", Data);
}


uint8_t PPU::ReadOAMDATA()
{
	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU Read PPU OAM DATA! {0:X}, {1:X}\n", mRegisters.v, mObjectAttributeMemory[mOAMADDR]);
	return mObjectAttributeMemory[mOAMADDR];
}

void PPU::WriteOAMDATA(const uint8_t Data)
{
	mObjectAttributeMemory[mOAMADDR] = Data;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU OAM DATA! {0:X}, {1:X}\n", mRegisters.v, Data);
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

void PPU::WritePPUSCROLL(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	// High first, low second
	if (!mRegisters.w)
	{
		mRegisters.x = Data & 7;
		mTempTransferAddress = uint16_t(mTempTransferAddress & 0b0111111111100000) | (Data >> 3);
	}
	else
	{
		mRegisters.t = uint16_t(mTempTransferAddress & 0b0000110000011111) | (((Data & 0xF8) << 2) | (uint16_t(Data & 7) << 12));
	}

	ToggleWRegister();
}

void PPU::WritePPUADDR(const uint8_t Data)
{
	if (mClockCount < REGISTER_IGNORE_CYCLES)
		return;

	// High first, low second
	if (!mRegisters.w)
	{
		mTempTransferAddress = uint16_t((Data & 0x3F) << 8);
	}
	else
	{
		mRegisters.v = (mTempTransferAddress | Data);
		mRegisters.t = mRegisters.v;
	}

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU Address! {0:X}\n", Data);

	ToggleWRegister();
}

uint8_t PPU::ReadPPUData()
{
	uint8_t OldBufferData = mPPUDataReadBuffer;

	if (mRegisters.v > 0x3F00)
	{
		OldBufferData = ReadPPUMemory(mRegisters.v);
	}
	else
	{
		mPPUDataReadBuffer = ReadPPUMemory(mRegisters.v);
	}

	bool bIncrement32Mode = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VRAM_ADDRESS_INCREMENT)) != 0;

	// 32 Down...Where?
	uint16_t Offset = bIncrement32Mode ? 32 : 1;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU read PPU Data! {0}\n", Offset);

	// VRAM Address is limited to 14 bits.
	mRegisters.v += Offset;
	mRegisters.v &= 0x3FFF;

	return OldBufferData;
}

void PPU::WritePPUData(const uint8_t Data)
{
	if (mRegisters.v < 0x2000)
	{
		if (mRomData->ChrRomMemory.size() == 0)
		{
			mChrMemory[mRegisters.v] = Data;
		}
	}
	else if (mRegisters.v < 0x3F00)
	{
		if (mRomData->NameTableLayout == ENameTableLayout::Horizontal)
		{
			uint16_t Temp = (mRegisters.v & 0x3FF) | (mRegisters.v & 0x800) >> 1;

			mMemory[Temp] = Data;
		}
		else
		{
			uint16_t Temp = mRegisters.v & 0x7FF;

			mMemory[Temp] = Data;
		}
	}
	else
	{
		uint16_t PalleteMask = (mRegisters.v & 3) == 0 ? 0x0F : 0x1F;
		mPalleteMemory[mRegisters.v & PalleteMask] = Data;
	}

	bool bIncrement32Mode = (mPPUCTRL & static_cast<uint8_t>(EPPUCTRL::VRAM_ADDRESS_INCREMENT)) != 0;

	// 32 Down...Where?
	uint16_t Offset = bIncrement32Mode ? 32 : 1;

	mLog->Log(ELOGGING_SOURCES::PPU, ELOGGING_MODE::INFO, "CPU wrote PPU Data! {0}\n", Offset);

	// VRAM Address is limited to 14 bits.
	mRegisters.v += Offset;
	mRegisters.v &= 0x3FFF;
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
