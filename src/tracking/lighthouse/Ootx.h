#pragma once
// L2: OOTX bit-stream decoder. A base station sends one bit per rotation (in the LH2 sync); packets are
//   preamble (17 zero bits + 1) | u16 length | payload (padded to even) | u32 CRC32, with a stuffed '1' sync
//   bit after every 16 data bits.
// Ported from libsurvive (MIT) src/ootx_decoder.c (Joshua Allen).
#include <cstdint>
#include <optional>
#include <vector>

namespace lhp::tracking::lighthouse {

struct OotxStats {
    uint32_t bits = 0, preambles = 0, badSyncBits = 0, badCrc = 0, packets = 0, guessedBits = 0;
    uint32_t progressBytes = 0;   // bytes collected towards the current packet
    uint32_t expectedBytes = 0;   // payload length of the current packet (0 = not yet known)
};

class OotxDecoder {
public:
    /// Feeds one bit (0/1, or -1 = unknown because a rotation was missed).
    /// Returns the payload when a packet completes with a valid CRC.
    std::optional<std::vector<uint8_t>> Push(int bit);
    const OotxStats& Stats() const { return stats_; }

private:
    void Reset();
    static constexpr int kMax = 64;
    uint8_t buf_[kMax]{};
    int offset_ = 0;
    int bitsWritten_ = 0;
    uint32_t preamble_ = 0xFFFFFFFF;
    int bitsProcessed_ = 0;
    bool found_ = false;
    OotxStats stats_;
};

uint32_t Crc32(const uint8_t* data, size_t length);

}  // namespace lhp::tracking::lighthouse
