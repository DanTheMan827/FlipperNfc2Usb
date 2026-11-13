# NFC2USB: USB NFC reader emulator for Flipper Zero

When the app starts, it asks which USB device the Flipper should become. Press **Back** on a status screen to return to the menu; that also restores the Flipper's normal USB.

| Mode | USB identity | Host side |
|---|---|---|
| MFRC522 UART (CDC), the default | Generic USB CDC-ACM serial port, `1209:0001` | Serial port; NXP MFRC522 register protocol in UART mode. No driver needed on Windows 10+, macOS or Linux |
| MFRC522 UART (CP2102) | Silicon Labs CP2102, `10C4:EA60` | The same, behind an emulated CP2102 for software that looks for that adapter |
| ACS ACR122U | `072F:2200`, "ACS" / "ACR122U PICC Interface" | USB CCID reader (PC/SC), PN532 inside |
| ACS ACR1552U | `072F:2303`, "ACS" / "ACR1552 1S CL Reader", PICC + SAM interfaces | USB CCID reader (PC/SC) |

RF goes through the Flipper's own NFC antenna, so real cards answer.

Settings are saved to `apps_data/mfrc522_uart/settings.bin` on the SD card whenever they change: the MFRC522 adapter (CDC or CP2102), chip version, Strict UART, wire timing and the reader buzzer.

## Build and run

```
pip install ufbt
cd mfrc522_uart
ufbt            # builds dist/mfrc522_uart.fap
ufbt launch     # builds, installs and starts it over USB
```

The app lives under **Apps → USB → NFC2USB**. It targets firmware 1.5.1 (API 88.x). Every exported firmware function it calls was checked against `targets/f7/api_symbols.csv` of that release. While a mode is running, qFlipper and the CLI are disconnected.

## Card types

✓ = implemented, ✗ = the emulated device supports it but the Flipper can't reach it, — = the emulated device doesn't support it.

| Card type | MFRC522 | ACR122U | ACR1552U | Notes |
|---|---|---|---|---|
| MIFARE Classic 1K/4K/Mini, Plus SL1/SL2 (Crypto1) | ✓ | ✓ | ✓ | |
| MIFARE Ultralight / C / EV1, NTAG | ✓ | ✓ | ✓ | |
| ISO14443-4A (DESFire, smart cards) | ✓ | ✓ | ✓ | |
| ISO14443-4B | — | ✓ | ✓ | |
| FeliCa 212 kbit/s | — | ✓ | ✓ | |
| FeliCa 424 kbit/s | — | ✗ | ✗ | The firmware's FeliCa mode is fixed at 212 kbit/s |
| ISO15693 (ICODE SLI/SLIX, ST LRI, my-d) | — | — | ✓ | |
| SRI / SRIX (ST25TB) | — | — | ✓ | Off by default, like on the real reader |
| Picopass / iClass over ISO15693 | — | — | ✓* | Off by default. *Identification and block reads only; see limits |
| Picopass over ISO14443B, Innovatron, CTS | — | — | ✗ | No matching RF framing in the Flipper NFC library |
| Topaz / Jewel (NFC Type 1) | — | ✗ | ✗ | Needs each command byte sent as a separate RF frame, which the library can't do |
| ISO14443 at 212/424/848 kbit/s | ✗ | — | ✗ | The library configures ISO14443 at 106 kbit/s only |

## MFRC522 UART mode

The simulated chip sits behind one of two USB serial adapters, chosen under **USB adapter** in the settings (**Config**, the OK button, on the status screen). Changing it re-plugs the Flipper as the other adapter.

### CDC side (default)

A standard USB CDC-ACM device (CDC PSTN 1.2), so every current OS uses its built-in serial driver. The ID is the pid.codes test VID/PID `1209:0001`, which no vendor driver claims.

* SET_LINE_CODING / GET_LINE_CODING: any rate, 5–8 data bits, none/odd/even/mark/space parity, 1/1.5/2 stop bits; anything else stalls.
* SET_CONTROL_LINE_STATE: DTR and RTS. DTR is shown as "Port open", since hosts raise it when they open the port; data flows either way.
* SEND_BREAK: on, off and timed.
* SERIAL_STATE notifications: carrier (DCD/DSR) when the port opens, and framing errors.
* Interface protocol 0 (no AT commands), so Linux ModemManager leaves the port alone.
* Writes to the host are at most 63 bytes, so every packet is short and hosts reading with large buffers get the data at once.

### CP2102 side (AN571 vendor protocol)

The device and interface descriptors match a CP2102, and GET_PARTNUM returns `02`. Implemented requests:

* IFC_ENABLE, SET/GET_BAUDDIV, SET/GET_BAUDRATE (any rate is accepted exactly), SET/GET_LINE_CTL
* SET_BREAK, IMM_CHAR, SET_MHS, GET_MDMSTS, SET_XON/XOFF
* SET/GET_EVENTMASK, SET_CHAR, SET/GET_CHARS, GET_PROPS
* GET_COMM_STATUS (live queue depths and latched errors), RESET, PURGE, SET/GET_FLOW
* EMBED_EVENTS, GET_EVENTSTATE

When the host deconfigures USB, the module loses power and the chip is power-cycled.

### MFRC522 side (checked against the NXP datasheet Rev 3.9)

* **UART protocol** (§8.1.3): a read is `1 r aaaaaa` and gets the data byte back. A write is address then data, and the chip echoes the address byte.
* **UART rate:** 9600 at power-on (`EBh`). SerialSpeedReg follows the datasheet formula, and SoftReset returns it to 9600. Rates above 1228.8 kBd are treated as unusable.
* **Registers:** all 64 with their datasheet reset values, including the reserved ones (`23h`=88h, `25h`=87h, `3Ch`=FFh, `3Eh`=03h).
* **Commands:** all of them, with the §10 behaviour:
  * Mem waits for 25 bytes.
  * MFAuthent waits for its 12 bytes, blocks the FIFO (WrErr), keeps TxIRq/RxIRq blocked, and ends with IdleIRq. It performs real Crypto1 authentication, including nested authentication.
  * CalcCRC is continuous.
  * Transceive keeps running after each reply.
  * IdleIRq follows the §9.3.1.5 rules.
  * The self-test vectors for v1.0 and v2.0 are the datasheet's (§16.1.1).
* **Timer:** TPrescalEven takes effect only on v2.0 silicon. TAuto stops after the 5th received bit. TAutoRestart is supported.
* **Soft power-down:** exits 1024 clocks (about 38 µs) after PowerDown is cleared.
* **Status line** `Chip <baud> (<SerialSpeedReg>) <state>`: the rate the chip's register selects (`EBh` = 9631 Bd at power-on). State is `ok` when the host port matches it (±3 %, 8N1), `=host` when it doesn't but Strict UART is off so the chip is assumed to follow the host, `MISMATCH` when it doesn't and Strict UART is on (bytes are garbled, as on real wires), and `OVERSPD` above 1228.8 kBd.
* **Status screen:** adapter and host port (`CDC open 921600 8N1`), the chip's own rate, RF state with the running command, and the last RF exchange. The button bar at the bottom opens **Config** (OK) and the **USB log** (Right): the last control requests the host sent and whether the Flipper accepted them. **Status** (Left or Back) returns from the log.
* **Settings** (**Config** on the status screen):
  * USB adapter: CDC or CP2102
  * chip version
  * Strict UART: rates and framing must match, as on real wires
  * wire timing: real or instant
  * power-cycle chip: a supply cut, so the internal buffer is lost

## ACR122U mode (checked against ACS API v2.04 and the NXP PN532 User Manual UM0701-02)

* **ATRs** follow the manual's tables. MIFARE 1K, FeliCa (`…03 F0 11 … 8A`) and ISO14443-B (the full ATQB as historical bytes, T0 = `8C`) reproduce the manual's examples byte for byte.
* **Pseudo-APDUs:**
  * `FF CA 00` returns the UID, PUPI or IDm; `FF CA 01` returns the full ATS.
  * `FF 82` takes key numbers 00–01.
  * `FF 86` and `FF 88` authenticate.
  * `FF B0` reads up to 16 bytes; `FF D6` writes 16 bytes (Classic) or 4 (Ultralight).
  * `FF D7` and `FF B1` are the value-block commands.
  * `FF 00 40` drives the LED and buzzer, with blinking; the reply waits for the sequence to finish.
  * `FF 00 41`, `48` (`ACR122U207`), `50`, `51` and `52` are supported.
* **Polling:** the reader polls for ISO14443-A, ISO14443-B and FeliCa as set in the PICC operating parameter (default `FF`). The interval is 250 or 500 ms, and bit 6 controls auto-RATS.
* **Native commands:** FeliCa native APDUs are passed straight to the card. DESFire native answers shorter than 2 bytes get `90 00` appended.
* **PN532 emulation through `FF 00 00 00`:**

  | Command | Behaviour |
  |---|---|
  | GetFirmwareVersion | Returns `32 01 06 07` |
  | GetGeneralStatus | Last error, external field `00`, target bit rate and type, SAM byte `80` |
  | ReadRegister / WriteRegister | CIU registers |
  | SetParameters, SAMConfiguration, PowerDown | Supported |
  | RFConfiguration | Field on/off, fRetryTimeout, retries |
  | InListPassiveTarget | 106 kbit/s A (with UID filter), 212 kbit/s FeliCa (POL_RES), 106 kbit/s B (ATQB + ATTRIB_RES). FeliCa 424 and Jewel report no target |
  | InAutoPoll | Supported |
  | InDataExchange | MIFARE commands, T=CL for A and B, FeliCa frames |
  | InCommunicateThru | Supported |
  | InDeselect / InRelease / InSelect / InPSL | Supported |

  Error codes follow UM0701 §7.1.

## ACR1552U mode (checked against ACS Reference Manual v1.09 and the TSP v1.08)

* **USB:** a PICC CCID interface and a SAM CCID interface; the SAM slot is empty. Interface names are "ACR1552 1S CL Reader PICC" and "… SAM". The Flipper's USB core only serves string indices 1–3, so the app chains its own descriptor handler in front for these. The serial number comes from the Flipper's UID.
* **Polling** follows Set Polling/ATR Option (default `8Bh`, including the RF-off intervals) and Set PICC Polling Type (default `07 05`: A, B, FeliCa, ISO15693; plus Picopass-B, which isn't reachable).
* **ATRs** follow the manual's "Supported PICC ATR" table. MIFARE 1K, DESFire, ISO14443-B (the manual's EZ-Link example), FeliCa, ICODE SLIX, SRI and Picopass reproduce the published ATRs byte for byte.
* **Extra MIFARE identification** (option bit 3): Ultralight EV1 (`003D`), Ultralight C (`003A`), Plus SL2 (`0038`/`0039`). The SmartMX Part-4 ATR follows option bit 7.
* **Pseudo-APDUs:**

  | APDU | Function |
  |---|---|
  | `FF CA` | P1 = 00 / 01 / 02 / 80, per card type as in §5.3.3.1 |
  | `FF 82` / `FF 86` | Crypto1 keys |
  | `FF B0` / `FF D6` | Mode and 12-bit address; skips sector trailers; Classic, Ultralight, ISO15693, SRI and Picopass (read only) |
  | `FF D7` / `FF B1` | Value block (write, increment/decrement into a target block, copy) |
  | `FF 00 00 00` | Pass-through: ISO14443-3 native commands and FeliCa |
  | `FF FB` | ISO15693 pass-through (flags `00` means addressed `22h` with the UID inserted) |
  | `FF C2` | PC/SC 2.0 Part 3 transparent session, below |

* **`FF C2` transparent session:**
  * Manage Session: start/end (polling paused and resumed), RF on/off, timer.
  * Transparent Exchange: flags for CRC and protocol prologue, transmit bit framing, timer, FWTI, retries, transceive. The response carries `92`, `96` and `97` objects.
  * Switch Protocol: A, B, ISO15693 or FeliCa at layer 2, 3 or 4, answering `5F 51`/`8F`.
  * The manual's §5.3.5.6 example sequence produces the documented responses.
* **Escape commands:**

  | Code | Function |
  |---|---|
  | `25` | RF control and PCD/PICC status |
  | `23` | Polling/ATR option |
  | `01 20` | Polling type |
  | `24` | Auto PPS |
  | `35` | PICC type |
  | `50` | RF power |
  | `E5` | Selective suspend |
  | `18` | Firmware version (`ACR1552 R FW 1.07.00`) |
  | `33` | Serial number |
  | `F0` | S/N in USB descriptor |
  | `28` | Buzzer, single and repeatable |
  | `29` | LED: blue and green, shown on the Flipper's RGB LED |
  | `21` | UI behaviour (default `6F`: beep on presence and power-up, blue status LED) |

## Limits

* **RF reach** (see the card table): 106 kbit/s ISO14443 only; FeliCa at 212 kbit/s; no Topaz, Innovatron, CTS or Picopass-over-B. Auto-PPS always reports 106 kbit/s as the current speed.
* **One card at a time.** Collisions are reported, but without a bit position.
* **No host-supplied parity bits.** That covers PN532 ParityDisable and the transparent-session parity flags, so some libnfc attack tools won't work.
* **No 3DES authentication.** `FF 86` for Ultralight C or DESFire (key types `80h`/`0Ah`) answers `6A 81`.
* **Picopass is best-effort.** I couldn't access the Flipper Picopass app's source, so the iClass framing (ACTALL/IDENTIFY/SELECT/READ) is from my knowledge of the protocol. The Picopass CRC is the firmware's own variant and was checked against a known frame. Cards report the 2KS card name, and writes (which need a MAC) are refused.
* **Short APDUs only.** The ACR1552U supports extended APDUs, but the emulation doesn't.
* **Reader modes the ACR1552U has but the app doesn't emulate:** HID keyboard, card emulation and discovery mode.
* **Not in the documents, so best judgement:**
  * the CCID class-descriptor timing fields
  * the ACR1552U serial-number format and its reply to unknown escape codes (`6A 81`)
  * the v0.0 and FM17522 self-test vectors
  * whether MFRC522 RxCRCEn strips the CRC bytes (here it does)
* **Unverified field name.** The `usbd_device.descriptor_callback` field name comes from libusb_stm32, a submodule missing from the firmware tarball.
* **Presence checks.** For MIFARE Classic and Ultralight, the 1 s idle presence check re-selects the card, which ends a Crypto1 session.
