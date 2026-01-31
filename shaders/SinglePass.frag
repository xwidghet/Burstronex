#version 460 core
in vec2 QuadCoordinate;
out vec4 FragColor;

layout(std430, binding = 0) buffer PPUMemory
{
	int mPPUCTRL;
	int mPPUMemory[16384];
	int mBackgroundDrawData[61696];
	int mPalleteMemory[32];
	int mObjectAttributeMemory[256];
};

// RGB pairs of SRGB colors
const float mPallete[] = {
	0.333, 0.333, 0.333, 0.0, 0.067, 0.427, 0.012, 0.0, 0.494, 0.145, 0.0, 0.443, 0.255, 0.0, 0.286, 0.31, 0.0, 0.071, 0.294, 0.0, 0.0, 0.216, 0.039, 0.0, 0.09, 0.114, 0.0, 0.0, 0.173, 0.0, 0.0, 0.2, 0.0, 0.0, 0.184, 0.035, 0.0, 0.137, 0.259, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.647, 0.647, 0.647, 0.035, 0.306, 0.737, 0.192, 0.204, 0.855, 0.369, 0.122, 0.812, 0.522, 0.082, 0.627, 0.608, 0.09, 0.345, 0.604, 0.149, 0.047, 0.51, 0.239, 0.0, 0.353, 0.341, 0.0, 0.176, 0.424, 0.0, 0.024, 0.467, 0.0, 0.0, 0.455, 0.2, 0.0, 0.396, 0.498, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.996, 1.0, 1.0, 0.333, 0.631, 1.0, 0.42, 0.471, 1.0, 0.576, 0.404, 1.0, 0.831, 0.42, 1.0, 0.949, 0.427, 0.745, 0.961, 0.478, 0.435, 0.878, 0.565, 0.176, 0.729, 0.671, 0.035, 0.549, 0.757, 0.051, 0.388, 0.808, 0.22, 0.286, 0.808, 0.49, 0.275, 0.757, 0.8, 0.235, 0.235, 0.235, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.996, 1.0, 1.0, 0.718, 0.859, 1.0, 0.725, 0.769, 1.0, 0.792, 0.729, 1.0, 0.906, 0.749, 1.0, 0.973, 0.761, 0.914, 0.984, 0.78, 0.788, 0.957, 0.812, 0.675, 0.898, 0.855, 0.608, 0.824, 0.894, 0.604, 0.757, 0.918, 0.663, 0.71, 0.922, 0.769, 0.698, 0.902, 0.894, 0.686, 0.686, 0.686, 0.0, 0.0, 0.0, 0.0, 0.0
};

bool RenderSprite(vec2 QuadCoordinate, int SpritePatternTableAddress, out int SpriteAttributes, out int SpriteTileData)
{
	ivec2 ScanlinePos = ivec2(int(QuadCoordinate.x * 256.0), int(QuadCoordinate.y * 256.0));
	ScanlinePos.y -= 1;

	// 0: 8x8, 1: 8x16
	bool SpriteSizeMode16 = (mPPUCTRL & (1 << 5)) != 0;
	// Number of rows from scanline 0 of the sprite.
	int HeightSize = SpriteSizeMode16 ? 15 : 7;

	// TODO: Once PPU implements rendering, add an acceleration structure to avoid needing to loop all of OAM.
	int SpriteCount = 0;
	for (int i = 0; i < 64; i++)
	{
		// Y Position is one lower due do sprite rendering being delayed by one scanline.
		ivec2 SpritePos = ivec2(mObjectAttributeMemory[i*4 + 3], mObjectAttributeMemory[i*4]);

		// Only Render/Count sprites that could have been rendered up to this scanline position.
		if (((SpritePos.y + HeightSize) < ScanlinePos.y) || (SpritePos.y > ScanlinePos.y))
			continue;

		if (SpritePos.x > ScanlinePos.x)
			continue;

		SpriteCount++;
		if (SpriteCount == 9)
			break;

		// After sprite counting since we need to know how many sprites are on this scanline.
		if ((SpritePos.x + 7) < ScanlinePos.x)
			continue;

		int SpritePatternTableOffset = SpritePatternTableAddress;
		int SpriteIndex = mObjectAttributeMemory[i*4 + 1];
		int SpriteTileNumber = SpriteIndex;

		// 8x16 sprites are stored in alternating tables, using the first bit of the Index as the toggle.
		if (SpriteSizeMode16)
		{
			SpriteTileNumber = (SpriteIndex & 0xFE);
			SpritePatternTableOffset += (0x1000 * (SpriteIndex & 1));
		}

		SpriteAttributes = mObjectAttributeMemory[i*4 + 2];

		ivec2 PixelInSpritePos = ScanlinePos - SpritePos;
		PixelInSpritePos.x = 7 - PixelInSpritePos.x;

		bool bFlipHorizontally = (SpriteAttributes & 0x40) != 0;
		if (bFlipHorizontally)
		{
			PixelInSpritePos.x = 7 - PixelInSpritePos.x;
		}

		bool bFlipVertically = (SpriteAttributes & 0x80) != 0;
		if (bFlipVertically)
		{
			PixelInSpritePos.y = HeightSize - PixelInSpritePos.y;
		}

		// Update Attributes/Pallete
		int PatternByte0 = mPPUMemory[SpritePatternTableOffset + SpriteTileNumber*16 + PixelInSpritePos.y];
		int PatternByte1 = mPPUMemory[SpritePatternTableOffset + SpriteTileNumber*16 + PixelInSpritePos.y + 8];

		SpriteTileData = ((PatternByte0 >> PixelInSpritePos.x) & 1);
		SpriteTileData |= ((PatternByte1 >> PixelInSpritePos.x) & 1) << 1;

		bool bIsTransparent = SpriteTileData == 0;
		if (bIsTransparent)
			continue;

		// We've got our target!
		return true;
	}

	return false;
}

void main()
{
	// First two bits of PPUCTRL contain the offset.
	int NameTableAddress = 0x2000 + 0x400 * (mPPUCTRL & 0x3);
	int SpritePatternTableAddress = 0x1000 * int((mPPUCTRL & (1 << 3)) != 0);
	int BackgroundPatternTableAddress = 0x1000 * int((mPPUCTRL & (1 << 4)) != 0);

	// 32x30 Grid, 1 byte per cell
	// Why does making Y a 32 grid fix it? Is it the boundary area, and it's 30 with that removed??
	const int NameCellX = int(QuadCoordinate.x * 32.0);
	const int NameCellY = int(QuadCoordinate.y * 32.0);

	// Cells < 1 and 30+ are in the vblank area.
	if (NameCellY < 1 || NameCellY > 29)
		discard;

	const int CellData = mPPUMemory[NameTableAddress + NameCellX + NameCellY*32];

	int TileCorner = BackgroundPatternTableAddress + CellData*16;

	// Get Tile Top Left Corner
	int XPixel = int(QuadCoordinate.x * 256.0) & 7;
	int YPixel = int(QuadCoordinate.y * 256.0) & 7;

	// Each Tile is 16 Bytes, where X coordinate is in the bits stored accross two bytes.
	int Byte0 = mPPUMemory[TileCorner + YPixel];
	int Byte1 = mPPUMemory[TileCorner + YPixel + 8];

	// Read specific pixel in this tile.
	// Pixels are stored in reverse bit order.
	int PixelIndex = 7 - XPixel;
	int TileData = ((Byte0 >> PixelIndex) & 1) * 1;
	TileData += ((Byte1 >> PixelIndex) & 1) * 2;

	// Calculate Attributes
	int AttributeOffset = (NameCellX >> 2) + (NameCellY >> 2)*8;
	int AttributeTable = mPPUMemory[NameTableAddress + AttributeOffset + 0x3C0];
	int AttributeCell = ((NameCellX & 2) + (NameCellY & 2)*2) >> 1;
	int AttributeTileData = ((AttributeTable >> (AttributeCell << 1)) & 3);

	int PalleteIndex = TileData > 0 ? (TileData + (AttributeTileData << 2)) : 0;

	int SpriteAttributes = 0;
	int SpritePatternData = 0;
	if (RenderSprite(QuadCoordinate, SpritePatternTableAddress, SpriteAttributes, SpritePatternData))
	{
		bool bBackgroundIsOpaque = TileData > 0;
		bool bSpriteBehindBackground = (SpriteAttributes & 0x20) != 0;
		bool bSkipSprite = ((bSpriteBehindBackground == true) && bBackgroundIsOpaque);
		if (!bSkipSprite)
		{
			PalleteIndex = 0;

			// IsSprite
			PalleteIndex |= (1 << 4);

			PalleteIndex &= ~(2 << 2);
			PalleteIndex |= (SpriteAttributes & 3) << 2;

			PalleteIndex &= ~3;
			PalleteIndex |= SpritePatternData;
		}
	}

	// Pallete is RGB floats, so need to navigate to the 3 float pair for this pallete color.
	int PalleteValue = mPalleteMemory[PalleteIndex] * 3;

	vec3 PalleteColor = vec3(mPallete[PalleteValue], mPallete[PalleteValue+1], mPallete[PalleteValue+2]);

	FragColor = vec4(PalleteColor, 1.0);
}
