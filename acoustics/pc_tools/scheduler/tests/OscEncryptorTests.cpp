#include <catch2/catch_test_macros.hpp>

#include "acoustics/osc/OscEncryptor.h"

#include <vector>

TEST_CASE("OscEncryptor round trip equals identity when decrypting with same key", "[osc][crypto]") {
    acoustics::osc::OscEncryptor encryptor;
    acoustics::osc::OscEncryptor::Key256 key{};
    acoustics::osc::OscEncryptor::Iv128 iv{};

    for (std::size_t i = 0; i < key.size(); ++i) {
        key[i] = static_cast<std::uint8_t>(i);
    }
    for (std::size_t i = 0; i < iv.size(); ++i) {
        iv[i] = static_cast<std::uint8_t>(0xF0 + i);
    }

    encryptor.setKey(key, iv);
    const std::vector<std::uint8_t> plaintext{0x10, 0x20, 0x30, 0x40, 0x50};
    const auto derivedIv = encryptor.deriveIv(1);
    const auto ciphertext = encryptor.encrypt(plaintext, derivedIv);

    acoustics::osc::OscEncryptor decryptor;
    decryptor.setKey(key, iv);
    const auto decrypted = decryptor.encrypt(ciphertext, decryptor.deriveIv(1));

    REQUIRE(ciphertext != plaintext);
    REQUIRE(decrypted == plaintext);
}
