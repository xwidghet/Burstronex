#version 460 core
in vec2 QuadCoordinate;
out vec4 FragColor;

layout(std430, binding = 0) buffer PPUMemory
{
	int mPPUCTRL;
	int mPPUMemory[16384];
	int mBackgroundDrawData[61696];
	int mChrMemory[8192];
	int mPalleteMemory[32];
	int mObjectAttributeMemory[256];
};

void main()
{
	const int Stride = 16 * 16;

	int PatternTableOffset = QuadCoordinate.x > 0.5 ? 4096 : 0;

	vec2 PatternCoordinates = QuadCoordinate;
	PatternCoordinates.x = fract(PatternCoordinates.x * 2);

	// 256 Tile Grid, 16x16
	const int X = int(PatternCoordinates.x * 16.0);
	const int Y = int((PatternCoordinates.y) * 16.0);

	// Get Tile Top Left Corner
	int TileCorner = X*16 + Y*Stride + PatternTableOffset;
	int XPixel = int(PatternCoordinates.x * 128.0) % 16;
	int YPixel = int(PatternCoordinates.y * 128.0) % 8;

	// Each Tile is 16 Bytes, where X coordinate is in the bits stored accross two bytes.
	int Byte0 = mChrMemory[TileCorner + YPixel];
	int Byte1 = mChrMemory[TileCorner + YPixel + 8];

	// Read specific pixel in this tile.
	// Pixels are stored in reverse bit order.
	int PixelIndex = 7 - int(PatternCoordinates.x * 128.0) % 8;
	bool TileData0 = (Byte0 & (1 << PixelIndex)) != 0;
	bool TileData1 = (Byte1 & (1 << PixelIndex)) != 0;

	vec4 Color = vec4(0);
	Color.rgb = (((float(TileData0) * 1) + float(TileData1) * 2) / 3.0).xxx;

	//FragColor = vec4(QuadCoordinate.x, QuadCoordinate.y, 1.0f, 1.0f);
	FragColor = Color;
}
