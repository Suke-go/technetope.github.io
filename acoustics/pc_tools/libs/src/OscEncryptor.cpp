#include "acoustics/osc/OscEncryptor.h"

#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <stdexcept>
#include <limits>

namespace acoustics::osc {

void OscEncryptor::setKey(const Key256& key, const Iv128& iv) {
    key_ = key;
    iv_ = iv;
    enabled_ = true;
}

void OscEncryptor::clear() {
    if (enabled_) {
        OPENSSL_cleanse(key_.data(), key_.size());
        OPENSSL_cleanse(iv_.data(), iv_.size());
    }
    enabled_ = false;
}

OscEncryptor::Iv128 OscEncryptor::deriveIv(std::uint64_t counter) const {
    if (!enabled_) {
        throw std::runtime_error("Encryption not enabled");
    }

    Iv128 derived = iv_;
    std::uint64_t high = 0;
    std::uint64_t low = 0;
    for (int i = 0; i < 8; ++i) {
        high = (high << 8) | derived[static_cast<std::size_t>(i)];
    }
    for (int i = 8; i < 16; ++i) {
        low = (low << 8) | derived[static_cast<std::size_t>(i)];
    }

    const std::uint64_t previousLow = low;
    low += counter;
    if (low < previousLow) {
        if (high == std::numeric_limits<std::uint64_t>::max()) {
            throw std::runtime_error("Derived IV overflow");
        }
        ++high;
    }

    for (int i = 15; i >= 8; --i) {
        derived[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(low & 0xFFU);
        low >>= 8U;
    }
    for (int i = 7; i >= 0; --i) {
        derived[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(high & 0xFFU);
        high >>= 8U;
    }
    return derived;
}

std::vector<std::uint8_t> OscEncryptor::encrypt(const std::vector<std::uint8_t>& plaintext,
                                                const Iv128& iv) const {
    if (!enabled_) {
        return plaintext;
    }

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("Failed to create EVP_CIPHER_CTX");
    }

    const EVP_CIPHER* cipher = EVP_aes_256_ctr();
    if (EVP_EncryptInit_ex(ctx, cipher, nullptr, key_.data(), iv.data()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_EncryptInit_ex failed");
    }

    std::vector<std::uint8_t> output(plaintext.size());
    int outLen = 0;
    if (!plaintext.empty()) {
        if (EVP_EncryptUpdate(ctx,
                              output.data(),
                              &outLen,
                              plaintext.data(),
                              static_cast<int>(plaintext.size())) != 1) {
            EVP_CIPHER_CTX_free(ctx);
            throw std::runtime_error("EVP_EncryptUpdate failed");
        }
    }

    int finalLen = 0;
    if (EVP_EncryptFinal_ex(ctx, output.data() + outLen, &finalLen) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP_EncryptFinal_ex failed");
    }

    EVP_CIPHER_CTX_free(ctx);
    return output;
}

}  // namespace acoustics::osc
