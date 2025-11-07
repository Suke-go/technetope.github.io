#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace acoustics::osc {

class OscEncryptor {
public:
    using Key256 = std::array<std::uint8_t, 32>;
    using Iv128 = std::array<std::uint8_t, 16>;

    OscEncryptor() = default;

    void setKey(const Key256& key, const Iv128& iv);
    void clear();

    bool enabled() const { return enabled_; }

    Iv128 deriveIv(std::uint64_t counter) const;
    std::vector<std::uint8_t> encrypt(const std::vector<std::uint8_t>& plaintext,
                                      const Iv128& iv) const;

private:
    Key256 key_{};
    Iv128 iv_{};
    bool enabled_{false};
};

}  // namespace acoustics::osc
