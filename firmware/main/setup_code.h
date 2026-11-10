// Matter setup code of this unit: the passcode and discriminator from its units/<unit>.defaults.
#pragma once

// Before esp_matter::start(): makes the Matter stack use this unit's passcode and discriminator.
void setup_code_install();

// On the Matter task, once the server is ready: prints the QR payload, a link that draws the QR code, and
// the manual pairing code to the USB console. Printed at every boot, whatever the log level.
void setup_code_print();
