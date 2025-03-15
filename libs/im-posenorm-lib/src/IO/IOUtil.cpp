#include "IOUtil.h"

#include <bit>

namespace IMPoseNorm::IO {

    bool NeedsEndiannessSwap(
            bool useLittleEndian) {

        return (useLittleEndian
                && std::endian::native != std::endian::little)
            || (!useLittleEndian
                && std::endian::native != std::endian::big);
    }
}