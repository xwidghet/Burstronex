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

// RGB pairs of SRGB colors
const float mPallete[] = {
	0.333, 0.333, 0.333, 0.0, 0.067, 0.427, 0.012, 0.0, 0.494, 0.145, 0.0, 0.443, 0.255, 0.0, 0.286, 0.31, 0.0, 0.071, 0.294, 0.0, 0.0, 0.216, 0.039, 0.0, 0.09, 0.114, 0.0, 0.0, 0.173, 0.0, 0.0, 0.2, 0.0, 0.0, 0.184, 0.035, 0.0, 0.137, 0.259, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.647, 0.647, 0.647, 0.035, 0.306, 0.737, 0.192, 0.204, 0.855, 0.369, 0.122, 0.812, 0.522, 0.082, 0.627, 0.608, 0.09, 0.345, 0.604, 0.149, 0.047, 0.51, 0.239, 0.0, 0.353, 0.341, 0.0, 0.176, 0.424, 0.0, 0.024, 0.467, 0.0, 0.0, 0.455, 0.2, 0.0, 0.396, 0.498, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.996, 1.0, 1.0, 0.333, 0.631, 1.0, 0.42, 0.471, 1.0, 0.576, 0.404, 1.0, 0.831, 0.42, 1.0, 0.949, 0.427, 0.745, 0.961, 0.478, 0.435, 0.878, 0.565, 0.176, 0.729, 0.671, 0.035, 0.549, 0.757, 0.051, 0.388, 0.808, 0.22, 0.286, 0.808, 0.49, 0.275, 0.757, 0.8, 0.235, 0.235, 0.235, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.996, 1.0, 1.0, 0.718, 0.859, 1.0, 0.725, 0.769, 1.0, 0.792, 0.729, 1.0, 0.906, 0.749, 1.0, 0.973, 0.761, 0.914, 0.984, 0.78, 0.788, 0.957, 0.812, 0.675, 0.898, 0.855, 0.608, 0.824, 0.894, 0.604, 0.757, 0.918, 0.663, 0.71, 0.922, 0.769, 0.698, 0.902, 0.894, 0.686, 0.686, 0.686, 0.0, 0.0, 0.0, 0.0, 0.0
};

void main()
{
	// First two bits of PPUCTRL contain the offset.
	int SpritePatternTableAddress = 0x1000 * int((mPPUCTRL & (1 << 3)) != 0);

	ivec2 PixelPos = ivec2(QuadCoordinate.x*255, QuadCoordinate.y*240);
	int DataIndex = PixelPos.x + PixelPos.y*256;
	int DrawData = mBackgroundDrawData[DataIndex];
	int Pallete = DrawData >> 8;
	int PalleteOffset = DrawData & 0x00FF;

	int PalleteIndex = (Pallete*4 + PalleteOffset);

	// Pallete is RGB floats, so need to navigate to the 3 float pair for this pallete color.
	int PalleteValue = mPalleteMemory[PalleteIndex] * 3;

	vec3 PalleteColor = vec3(mPallete[PalleteValue], mPallete[PalleteValue+1], mPallete[PalleteValue+2]);

	FragColor = vec4(PalleteColor, 1.0);

}
