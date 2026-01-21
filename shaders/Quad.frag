#version 460 core
in vec2 QuadCoordinate;
out vec4 FragColor;

layout(std430, binding = 0) buffer PPUMemory
{
	int mPPUCTRL;
	int mPPUMemory[16384];
	int mPalleteMemory[32];
	int mObjectAttributeMemory[256];
};

void main()
{
	vec2 PatternCoordinates = QuadCoordinate;

	// First two bits of PPUCTRL contain the offset.
	int NameTableAddress = 0x2000 + 0x400 * (mPPUCTRL & 0x3);
	int SpritePatternTableAddress = 0x1000 * int((mPPUCTRL & (1 << 4)) != 0);
	int BackgroundPatternTableAddress = 0x1000 * int((mPPUCTRL & (1 << 5)) != 0);

	// 0: 8x8, 1: 8x16
	bool SpriteSizeMode = (mPPUCTRL & (1 << 6)) != 0;

	// 32x30 Grid, 1 byte per cell
	const int NameCellX = int(PatternCoordinates.x * 32.0);
	const int NameCellY = int((PatternCoordinates.y) * 30.0);

	const int CellData = mPPUMemory[NameTableAddress + NameCellX + NameCellY*32];

	// 8x8 Character tiles filling 256x240 pixels
	const int X = int(PatternCoordinates.x * 32.0);
	const int Y = int((PatternCoordinates.y) * 30.0);

	int TileCorner = BackgroundPatternTableAddress + CellData;

	// Get Tile Top Left Corner
	int XPixel = int(PatternCoordinates.x * 128.0) % 8;
	int YPixel = int(PatternCoordinates.y * 128.0) % 8;

	// Each Tile is 16 Bytes, where X coordinate is in the bits stored accross two bytes.
	int Byte0 = mPPUMemory[TileCorner + YPixel];
	int Byte1 = mPPUMemory[TileCorner + YPixel + 8];

	// Read specific pixel in this tile.
	// Pixels are stored in reverse bit order.
	int PixelIndex = 7 - XPixel;
	bool TileData0 = (Byte0 & (1 << PixelIndex)) != 0;
	bool TileData1 = (Byte1 & (1 << PixelIndex)) != 0;

	vec4 Color = vec4(0);
	if (TileData0 && !TileData1)
		Color = vec4(1, 0, 0, 1);
	else if (!TileData0 && TileData1)
		Color = vec4(0, 1, 0, 1);
	else if (!TileData0 && !TileData1)
		Color = vec4(0, 0, 1, 1);
	else if (TileData0 && TileData1)
		Color = vec4(1, 1, 0, 1);

	FragColor = Color;
}
