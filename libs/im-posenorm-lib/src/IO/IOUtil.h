#pragma once

#include <cstddef>
#include <climits>

namespace IMPoseNorm::IO {

	bool NeedsEndiannessSwap(
		bool useLittleEndian);
	
    template <typename T>
    inline T SwapEndianness(
        T value) {

        static_assert (
            CHAR_BIT == 8,
            "CHAR_BIT != 8");

        union {
            T value;
            unsigned char byteValue[sizeof(T)];
        } source, destination;

        source.value = value;

        for (std::size_t i = 0; i < sizeof(T); i++) {

            destination.byteValue[i] = source.byteValue[sizeof(T) - i - 1];
        }

        return destination.value;
    }
}