#pragma once

#include <string>
#include <vector>

struct ROMData;

class RomLoader {

	static ROMData ParseNES10ROM(const std::vector<char>& FileBuffer);

	static ROMData ParseNES20ROM(const std::vector<char>& FileBuffer);
public:
	static ROMData Load(const std::string& PathToRom);
};
