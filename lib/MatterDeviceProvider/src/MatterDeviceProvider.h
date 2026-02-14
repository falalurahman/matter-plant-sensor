#pragma once

#include <platform/CommissionableDataProvider.h>
#include <platform/DeviceInstanceInfoProvider.h>
#include <crypto/CHIPCryptoPAL.h>
#include <lib/core/CHIPError.h>
#include <lib/support/Span.h>

// ============================================================================
// Device Info Configuration
// All values can be overridden via build flags, e.g.:
//   -D MATTER_DEVICE_SERIAL_NUMBER=\"SF-2026-003\"
// ============================================================================
#ifndef MATTER_DEVICE_VENDOR_NAME
#define MATTER_DEVICE_VENDOR_NAME       "Falalu's DIY"
#endif
#ifndef MATTER_DEVICE_VENDOR_ID
#define MATTER_DEVICE_VENDOR_ID         0xFFF1          // Test VID (matches ExampleDAC)
#endif
#ifndef MATTER_DEVICE_PRODUCT_NAME
#define MATTER_DEVICE_PRODUCT_NAME      "Matter Device"
#endif
#ifndef MATTER_DEVICE_PRODUCT_ID
#define MATTER_DEVICE_PRODUCT_ID        0x8000
#endif
#ifndef MATTER_DEVICE_SERIAL_NUMBER
#define MATTER_DEVICE_SERIAL_NUMBER     "MD-0000-001"
#endif
#ifndef MATTER_DEVICE_HW_VERSION
#define MATTER_DEVICE_HW_VERSION        1
#endif
#ifndef MATTER_DEVICE_HW_VERSION_STRING
#define MATTER_DEVICE_HW_VERSION_STRING "v1.0"
#endif
#ifndef MATTER_DEVICE_PRODUCT_URL
#define MATTER_DEVICE_PRODUCT_URL       ""
#endif
#ifndef MATTER_DEVICE_PRODUCT_LABEL
#define MATTER_DEVICE_PRODUCT_LABEL     "Matter Device"
#endif
#ifndef MATTER_DEVICE_PART_NUMBER
#define MATTER_DEVICE_PART_NUMBER       ""
#endif

// ============================================================================
// Commissioning Configuration
// Per-device values can be overridden via build flags, e.g.:
//   -D MATTER_DEVICE_SETUP_PASSCODE=12345678
//   -D MATTER_DEVICE_SETUP_DISCRIMINATOR=1234
// ============================================================================
// Passcode: 1 to 99999998 (exclude 11111111, 22222222, ..., 12345678, 87654321)
#ifndef MATTER_DEVICE_SETUP_PASSCODE
#define MATTER_DEVICE_SETUP_PASSCODE      20202021
#endif
// Discriminator: 0 to 4095 (12-bit)
#ifndef MATTER_DEVICE_SETUP_DISCRIMINATOR
#define MATTER_DEVICE_SETUP_DISCRIMINATOR 3840
#endif
// SPAKE2+ PBKDF2 iteration count (1000 - 100000)
#ifndef MATTER_DEVICE_SPAKE2P_ITERATION_COUNT
#define MATTER_DEVICE_SPAKE2P_ITERATION_COUNT 1000
#endif

// ============================================================================
// Custom Device Instance Info Provider
// ============================================================================
class MatterDeviceInstanceInfoProvider : public chip::DeviceLayer::DeviceInstanceInfoProvider
{
public:
    CHIP_ERROR GetVendorName(char * buf, size_t bufSize) override;
    CHIP_ERROR GetVendorId(uint16_t & vendorId) override;
    CHIP_ERROR GetProductName(char * buf, size_t bufSize) override;
    CHIP_ERROR GetProductId(uint16_t & productId) override;
    CHIP_ERROR GetPartNumber(char * buf, size_t bufSize) override;
    CHIP_ERROR GetProductURL(char * buf, size_t bufSize) override;
    CHIP_ERROR GetProductLabel(char * buf, size_t bufSize) override;
    CHIP_ERROR GetSerialNumber(char * buf, size_t bufSize) override;
    CHIP_ERROR GetManufacturingDate(uint16_t & year, uint8_t & month, uint8_t & day) override;
    CHIP_ERROR GetHardwareVersion(uint16_t & hardwareVersion) override;
    CHIP_ERROR GetHardwareVersionString(char * buf, size_t bufSize) override;
    CHIP_ERROR GetRotatingDeviceIdUniqueId(chip::MutableByteSpan & uniqueIdSpan) override;
};

// ============================================================================
// Custom Commissionable Data Provider
// ============================================================================
class MatterCommissionableDataProvider : public chip::DeviceLayer::CommissionableDataProvider
{
public:
    // Call once at startup to compute SPAKE2+ verifier from passcode
    CHIP_ERROR Init();

    CHIP_ERROR GetSetupDiscriminator(uint16_t & setupDiscriminator) override;
    CHIP_ERROR SetSetupDiscriminator(uint16_t setupDiscriminator) override;
    CHIP_ERROR GetSpake2pIterationCount(uint32_t & iterationCount) override;
    CHIP_ERROR GetSpake2pSalt(chip::MutableByteSpan & saltBuf) override;
    CHIP_ERROR GetSpake2pVerifier(chip::MutableByteSpan & verifierBuf, size_t & outVerifierLen) override;
    CHIP_ERROR GetSetupPasscode(uint32_t & setupPasscode) override;
    CHIP_ERROR SetSetupPasscode(uint32_t setupPasscode) override;

private:
    uint8_t mSalt[chip::Crypto::kSpake2p_Min_PBKDF_Salt_Length];
    chip::Crypto::Spake2pVerifierSerialized mSerializedVerifier;
    size_t mVerifierLen = 0;
    bool mInitialized = false;
};

// ============================================================================
// Pairing Code Helpers
// ============================================================================
const char * getMatterManualPairingCode();
const char * getMatterQRCodeUrl();

// ============================================================================
// Initialization - call AFTER Matter.begin()
// ============================================================================
void initMatterDeviceProviders();
