#include "setup_code.h"

#include <array>
#include <cstdint>
#include <cstdio>

#include <crypto/CHIPCryptoPAL.h>
#include <esp_log.h>
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#include <platform/CommissionableDataProvider.h>
#include <sdkconfig.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/SetupPayload.h>

namespace {

const char *const TAG = "setup_code";

// Matter Core spec 5.1.7.1: passcodes 1..99999998 except these trivial ones.
constexpr bool is_valid_passcode(uint32_t passcode)
{
    constexpr std::array<uint32_t, 11> trivial = {11111111, 22222222, 33333333, 44444444, 55555555, 66666666,
                                                  77777777, 88888888, 99999999, 12345678, 87654321};
    if (passcode < 1 || passcode > chip::kSetupPINCodeMaximumValue) {
        return false;
    }
    for (const uint32_t t : trivial) {
        if (passcode == t) {
            return false;
        }
    }
    return true;
}

static_assert(is_valid_passcode(CONFIG_GATE_SETUP_PASSCODE),
              "Set CONFIG_GATE_SETUP_PASSCODE in this unit's settings file, see firmware/units/unit.defaults.example");

constexpr uint32_t passcode = CONFIG_GATE_SETUP_PASSCODE;
constexpr uint16_t discriminator = CONFIG_GATE_SETUP_DISCRIMINATOR;
constexpr uint32_t spake2p_iterations = chip::Crypto::kSpake2p_Min_PBKDF_Iterations;

// The Spake2+ verifier is derived from the passcode at the first request, with a random salt per boot:
// commissioners read the salt from the device in every PASE session, so it need not be stored.
// Called only from the Matter task.
class unit_commissionable_data final : public chip::DeviceLayer::CommissionableDataProvider {
public:
    CHIP_ERROR GetSetupDiscriminator(uint16_t &value) override
    {
        value = discriminator;
        return CHIP_NO_ERROR;
    }
    CHIP_ERROR SetSetupDiscriminator(uint16_t) override { return CHIP_ERROR_NOT_IMPLEMENTED; }

    CHIP_ERROR GetSpake2pIterationCount(uint32_t &value) override
    {
        value = spake2p_iterations;
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetSpake2pSalt(chip::MutableByteSpan &out) override
    {
        ReturnErrorOnFailure(prepare());
        return chip::CopySpanToMutableSpan(chip::ByteSpan(salt_), out);
    }

    CHIP_ERROR GetSpake2pVerifier(chip::MutableByteSpan &out, size_t &length) override
    {
        ReturnErrorOnFailure(prepare());
        length = verifier_.size();
        return chip::CopySpanToMutableSpan(chip::ByteSpan(verifier_), out);
    }

    CHIP_ERROR GetSetupPasscode(uint32_t &value) override
    {
        value = passcode;
        return CHIP_NO_ERROR;
    }
    CHIP_ERROR SetSetupPasscode(uint32_t) override { return CHIP_ERROR_NOT_IMPLEMENTED; }

private:
    CHIP_ERROR prepare()
    {
        if (ready_) {
            return CHIP_NO_ERROR;
        }
        ReturnErrorOnFailure(chip::Crypto::DRBG_get_bytes(salt_.data(), salt_.size()));
        chip::Crypto::Spake2pVerifier verifier;
        ReturnErrorOnFailure(verifier.Generate(spake2p_iterations, chip::ByteSpan(salt_), passcode));
        chip::MutableByteSpan serialized(verifier_);
        ReturnErrorOnFailure(verifier.Serialize(serialized));
        ready_ = true;
        return CHIP_NO_ERROR;
    }

    std::array<uint8_t, chip::Crypto::kSpake2p_Max_PBKDF_Salt_Length> salt_{};
    std::array<uint8_t, chip::Crypto::kSpake2p_VerifierSerialized_Length> verifier_{};
    bool ready_ = false;
};

unit_commissionable_data s_provider;

}  // namespace

void setup_code_install()
{
    esp_matter::set_custom_commissionable_data_provider(&s_provider);
}

void setup_code_print()
{
    const chip::RendezvousInformationFlags rendezvous(chip::RendezvousInformationFlag::kBLE);
    std::array<char, 64> qr_buffer{};
    std::array<char, 32> manual_buffer{};
    std::array<char, 160> url{};
    chip::MutableCharSpan qr(qr_buffer.data(), qr_buffer.size());
    chip::MutableCharSpan manual(manual_buffer.data(), manual_buffer.size());
    if (GetQRCode(qr, rendezvous) != CHIP_NO_ERROR || GetManualPairingCode(manual, rendezvous) != CHIP_NO_ERROR ||
        GetQRCodeUrl(url.data(), url.size(), qr) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "setup code could not be encoded");
        return;
    }
    std::printf("Matter setup code: %.*s\nManual pairing code: %.*s\nQR code: %s\n", static_cast<int>(qr.size()),
                qr.data(), static_cast<int>(manual.size()), manual.data(), url.data());
}
