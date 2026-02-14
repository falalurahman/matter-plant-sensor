#include "MatterDeviceProvider.h"
#include <Arduino.h>
#include <cstring>
#include <setup_payload/ManualSetupPayloadGenerator.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>
#include <setup_payload/SetupPayload.h>

// Static instances (must survive for the lifetime of the process)
static MatterDeviceInstanceInfoProvider sDeviceInstanceInfoProvider;
static MatterCommissionableDataProvider sCommissionableDataProvider;

// Static buffers for pairing codes
static char sManualPairingCode[22];  // max 21 chars + null
static char sQRCodeUrl[256];

// ============================================================================
// Helper
// ============================================================================
static CHIP_ERROR CopyString(const char * src, char * buf, size_t bufSize)
{
    size_t len = strlen(src);
    if (len >= bufSize) {
        return CHIP_ERROR_BUFFER_TOO_SMALL;
    }
    strncpy(buf, src, bufSize);
    buf[bufSize - 1] = '\0';
    return CHIP_NO_ERROR;
}

// ============================================================================
// DeviceInstanceInfoProvider implementation
// ============================================================================

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetVendorName(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_VENDOR_NAME, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetVendorId(uint16_t & vendorId)
{
    vendorId = MATTER_DEVICE_VENDOR_ID;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetProductName(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_PRODUCT_NAME, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetProductId(uint16_t & productId)
{
    productId = MATTER_DEVICE_PRODUCT_ID;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetPartNumber(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_PART_NUMBER, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetProductURL(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_PRODUCT_URL, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetProductLabel(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_PRODUCT_LABEL, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetSerialNumber(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_SERIAL_NUMBER, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetManufacturingDate(uint16_t & year, uint8_t & month, uint8_t & day)
{
    // __DATE__ format: "Mmm dd yyyy" (e.g. "Feb  3 2026")
    static const char buildDate[] = __DATE__;
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    uint8_t m = 0;
    for (uint8_t i = 0; i < 12; i++) {
        if (buildDate[0] == months[i * 3] &&
            buildDate[1] == months[i * 3 + 1] &&
            buildDate[2] == months[i * 3 + 2]) {
            m = i + 1;
            break;
        }
    }
    day   = (uint8_t)((buildDate[4] == ' ' ? 0 : (buildDate[4] - '0') * 10) + (buildDate[5] - '0'));
    month = m;
    year  = (uint16_t)((buildDate[7] - '0') * 1000 + (buildDate[8] - '0') * 100 +
                        (buildDate[9] - '0') * 10 + (buildDate[10] - '0'));
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetHardwareVersion(uint16_t & hardwareVersion)
{
    hardwareVersion = MATTER_DEVICE_HW_VERSION;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetHardwareVersionString(char * buf, size_t bufSize)
{
    return CopyString(MATTER_DEVICE_HW_VERSION_STRING, buf, bufSize);
}

CHIP_ERROR MatterDeviceInstanceInfoProvider::GetRotatingDeviceIdUniqueId(chip::MutableByteSpan & uniqueIdSpan)
{
    // 16-byte unique ID - in production this should be truly unique per device
    static const uint8_t kUniqueId[16] = {
        0xFA, 0x1A, 0x10, 0x50, 0xAA, 0x77, 0x08, 0xE0,
        0xFA, 0x4E, 0xD3, 0xD1, 0xC3, 0x00, 0x00, 0x01
    };
    if (uniqueIdSpan.size() < sizeof(kUniqueId)) {
        return CHIP_ERROR_BUFFER_TOO_SMALL;
    }
    memcpy(uniqueIdSpan.data(), kUniqueId, sizeof(kUniqueId));
    uniqueIdSpan.reduce_size(sizeof(kUniqueId));
    return CHIP_NO_ERROR;
}

// ============================================================================
// CommissionableDataProvider implementation
// ============================================================================

CHIP_ERROR MatterCommissionableDataProvider::Init()
{
    // Generate a deterministic salt from the serial number
    memset(mSalt, 0, sizeof(mSalt));
    const char * serial = MATTER_DEVICE_SERIAL_NUMBER;
    size_t serialLen = strlen(serial);
    for (size_t i = 0; i < sizeof(mSalt); i++) {
        mSalt[i] = (i < serialLen) ? serial[i] : (uint8_t)(i + 0x53);
    }

    // Compute SPAKE2+ verifier at runtime from passcode
    chip::Crypto::Spake2pVerifier verifier;
    CHIP_ERROR err = verifier.Generate(
        MATTER_DEVICE_SPAKE2P_ITERATION_COUNT,
        chip::ByteSpan(mSalt, sizeof(mSalt)),
        MATTER_DEVICE_SETUP_PASSCODE
    );
    if (err != CHIP_NO_ERROR) {
        Serial.printf("ERROR: Failed to generate SPAKE2+ verifier: %" PRIu32 "\n", err.AsInteger());
        return err;
    }

    // Serialize the verifier for later retrieval
    chip::MutableByteSpan verifierSpan(mSerializedVerifier);
    err = verifier.Serialize(verifierSpan);
    if (err != CHIP_NO_ERROR) {
        Serial.printf("ERROR: Failed to serialize verifier: %" PRIu32 "\n", err.AsInteger());
        return err;
    }
    mVerifierLen = verifierSpan.size();
    mInitialized = true;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterCommissionableDataProvider::GetSetupDiscriminator(uint16_t & setupDiscriminator)
{
    setupDiscriminator = MATTER_DEVICE_SETUP_DISCRIMINATOR;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterCommissionableDataProvider::SetSetupDiscriminator(uint16_t setupDiscriminator)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR MatterCommissionableDataProvider::GetSpake2pIterationCount(uint32_t & iterationCount)
{
    iterationCount = MATTER_DEVICE_SPAKE2P_ITERATION_COUNT;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterCommissionableDataProvider::GetSpake2pSalt(chip::MutableByteSpan & saltBuf)
{
    if (saltBuf.size() < sizeof(mSalt)) {
        return CHIP_ERROR_BUFFER_TOO_SMALL;
    }
    memcpy(saltBuf.data(), mSalt, sizeof(mSalt));
    saltBuf.reduce_size(sizeof(mSalt));
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterCommissionableDataProvider::GetSpake2pVerifier(chip::MutableByteSpan & verifierBuf, size_t & outVerifierLen)
{
    if (!mInitialized) {
        return CHIP_ERROR_INCORRECT_STATE;
    }
    if (verifierBuf.size() < mVerifierLen) {
        return CHIP_ERROR_BUFFER_TOO_SMALL;
    }
    memcpy(verifierBuf.data(), mSerializedVerifier, mVerifierLen);
    verifierBuf.reduce_size(mVerifierLen);
    outVerifierLen = mVerifierLen;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterCommissionableDataProvider::GetSetupPasscode(uint32_t & setupPasscode)
{
    setupPasscode = MATTER_DEVICE_SETUP_PASSCODE;
    return CHIP_NO_ERROR;
}

CHIP_ERROR MatterCommissionableDataProvider::SetSetupPasscode(uint32_t setupPasscode)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

// ============================================================================
// Pairing Code Helpers
// ============================================================================

static void generatePairingCodes()
{
    chip::PayloadContents payload;
    payload.version = 0;
    payload.vendorID = MATTER_DEVICE_VENDOR_ID;
    payload.productID = MATTER_DEVICE_PRODUCT_ID;
    payload.commissioningFlow = chip::CommissioningFlow::kStandard;
    payload.rendezvousInformation.SetValue(
        chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE,
                                         chip::RendezvousInformationFlag::kOnNetwork));
    payload.discriminator.SetLongValue(MATTER_DEVICE_SETUP_DISCRIMINATOR);
    payload.setUpPINCode = MATTER_DEVICE_SETUP_PASSCODE;

    // Generate manual pairing code
    chip::ManualSetupPayloadGenerator manualGen(payload);
    chip::MutableCharSpan manualCodeSpan(sManualPairingCode, sizeof(sManualPairingCode));
    CHIP_ERROR err = manualGen.payloadDecimalStringRepresentation(manualCodeSpan);
    if (err != CHIP_NO_ERROR) {
        strncpy(sManualPairingCode, "ERROR", sizeof(sManualPairingCode));
    }

    // Generate QR code payload
    chip::QRCodeBasicSetupPayloadGenerator qrGen(payload);
    char qrPayload[128];
    chip::MutableCharSpan qrSpan(qrPayload, sizeof(qrPayload));
    err = qrGen.payloadBase38Representation(qrSpan);
    if (err != CHIP_NO_ERROR) {
        strncpy(sQRCodeUrl, "ERROR", sizeof(sQRCodeUrl));
    } else {
        snprintf(sQRCodeUrl, sizeof(sQRCodeUrl),
                 "https://project-chip.github.io/connectedhomeip/qrcode.html?data=%s",
                 qrPayload);
    }
}

const char * getMatterManualPairingCode()
{
    return sManualPairingCode;
}

const char * getMatterQRCodeUrl()
{
    return sQRCodeUrl;
}

// ============================================================================
// Initialization
// ============================================================================

void initMatterDeviceProviders()
{
    // Override the DeviceInstanceInfoProvider (vendor name, product name, serial, etc.)
    chip::DeviceLayer::SetDeviceInstanceInfoProvider(&sDeviceInstanceInfoProvider);

    // Initialize and install the CommissionableDataProvider (passcode, discriminator, SPAKE2+)
    CHIP_ERROR err = sCommissionableDataProvider.Init();
    if (err == CHIP_NO_ERROR) {
        chip::DeviceLayer::SetCommissionableDataProvider(&sCommissionableDataProvider);
    } else {
        Serial.println("WARNING: CommissionableDataProvider init failed, using defaults.");
    }

    // Keep the ExampleDAC provider for attestation (VID 0xFFF1 works with Apple Home)

    // Generate pairing codes
    generatePairingCodes();

    Serial.println("Custom providers installed.");
    Serial.printf("  Vendor:  %s (0x%04X)\n", MATTER_DEVICE_VENDOR_NAME, MATTER_DEVICE_VENDOR_ID);
    Serial.printf("  Product: %s (0x%04X)\n", MATTER_DEVICE_PRODUCT_NAME, MATTER_DEVICE_PRODUCT_ID);
    Serial.printf("  Serial:  %s\n", MATTER_DEVICE_SERIAL_NUMBER);
    Serial.printf("  Passcode:      %lu\n", (unsigned long)MATTER_DEVICE_SETUP_PASSCODE);
    Serial.printf("  Discriminator: %u\n", MATTER_DEVICE_SETUP_DISCRIMINATOR);
}
