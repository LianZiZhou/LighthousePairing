#include "tracking/lighthouse/Ootx.h"

#include <cstring>

namespace lhp::tracking::lighthouse {

uint32_t Crc32(const uint8_t* data, size_t n) {
    static uint32_t table[256];
    static bool init = [] {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        return true;
    }();
    (void)init;
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

void OotxDecoder::Reset() {
    offset_ = 0;
    bitsWritten_ = 0;
    found_ = false;   // buffer kept: packets repeat, so stale bits are good guesses for missed ones
    stats_.progressBytes = 0;
    stats_.expectedBytes = 0;
}

std::optional<std::vector<uint8_t>> OotxDecoder::Push(int bit) {
    ++stats_.bits;
    ++bitsProcessed_;
    // bit < 0: unknown (a rotation was missed). Guess like libsurvive: inside a packet assume 1 for the
    // preamble detector and keep the buffered bit from the previous (identical) packet.
    int pre = bit;
    if (pre < 0) pre = found_ ? 1 : ((preamble_ & 0x1FFFF) == 0 ? 1 : 0);
    preamble_ = (preamble_ << 1) | (uint32_t)(pre & 1);
    if ((preamble_ & 0x3FFFF) == 0x00001) {   // 17 zeros then a one
        ++stats_.preambles;
        Reset();
        bitsProcessed_ = 0;
        found_ = true;
        return std::nullopt;
    }
    if (bitsProcessed_ > 16) {   // every 17th bit is a stuffed '1'
        if (bit == 0) {   // unknown (-1) sync bits are accepted
            if (found_) ++stats_.badSyncBits;
            Reset();
        }
        bitsProcessed_ = 0;
        return std::nullopt;
    }
    if (!found_) return std::nullopt;

    uint8_t mask = (uint8_t)(1u << (7 - bitsWritten_));
    if (bit > 0) buf_[offset_] |= mask;
    else if (bit == 0) buf_[offset_] &= (uint8_t)~mask;
    else ++stats_.guessedBits;
    if (++bitsWritten_ > 7) {
        bitsWritten_ = 0;
        if (++offset_ >= kMax) {   // overflow: corrupt, wait for the next preamble
            Reset();
            return std::nullopt;
        }
    }
    uint16_t len = (uint16_t)(buf_[0] | (buf_[1] << 8));   // little endian length (first two bytes)
    uint16_t padded = (uint16_t)(len + (len & 1));
    stats_.progressBytes = (uint32_t)offset_;
    stats_.expectedBytes = offset_ >= 2 ? (uint32_t)padded + 6 : 0;
    if (offset_ >= 2 && padded + 6 > kMax) {
        Reset();
        return std::nullopt;
    }
    if (offset_ >= 2 && offset_ >= padded + 6) {
        const uint8_t* payload = buf_ + 2;
        uint32_t crc;
        std::memcpy(&crc, payload + padded, 4);
        std::optional<std::vector<uint8_t>> out;
        if (Crc32(payload, len) == crc) {
            ++stats_.packets;
            out = std::vector<uint8_t>(payload, payload + len);
        } else {
            ++stats_.badCrc;
        }
        Reset();
        return out;
    }
    return std::nullopt;
}

}  // namespace lhp::tracking::lighthouse
