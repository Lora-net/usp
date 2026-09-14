# WiFi-Based Region Detection Example for LR1110/LR1120

This project demonstrates a region detection system based on nearby Wi-Fi access points, using Semtech's LR1110/LR1120 radios and the LoRa Basics Modem (LBM) library. The goal is to determine the most probable LoRaWAN region (e.g., EU868, US915, CN470) by analyzing Wi-Fi metadata such as country codes and SSIDs.


## Project Structure

### 1. `wifi_region_finder` Module

**Files**:
- `wifi_region_finder.h`
- `wifi_region_finder.c`

**Purpose**:
Provides region scoring logic based on Wi-Fi access point metadata (MAC address, SSID, country code). This module helps estimate the most likely LoRaWAN region.

> **Note:** This module does **not** perform Wi-Fi scans. Scanning is handled by the LoRa Basics Modem Wi-Fi geolocation service. This module only processes the results.

**Main API Functions**:

- `void wrf_init(void);`
  Initializes the internal state of the module.

- `void wrf_process_ap(const uint8_t* mac, const char* ssid, const char* cc, wrf_region_score_t* score);`
  Takes a single access point's MAC, SSID, and country code strings, and updates the region scores. Each MAC is considered only once per scan sequence.

- `void wrf_print_region_scores(const wrf_region_score_t* score);`
  Outputs the current score breakdown per region to a debug trace/log — useful for diagnostics.

- `int wrf_get_highest_score_region(const wrf_region_score_t* score, int threshold, smtc_modem_region_t* region, uint8_t* confidence);`
  Evaluates scores and selects the most probable region, if the score exceeds a given threshold. Also returns a confidence metric (0–100), calculated from internal scoring heuristics.

### How Region Detection Works (Inside `wifi_region_finder`)

The `wifi_region_finder` module implements a scoring-based approach to determine the most probable LoRaWAN region based on metadata from nearby Wi-Fi access points.

#### MAC Address De-duplication

Each access point (AP) is identified by its MAC address. The module ensures:
- Each MAC is only considered **once per scan session**
- Separate checks are applied for:
  - **SSID-based scoring**
  - **Country Code-based scoring**

This prevents any single AP from influencing the result multiple times.

#### Scoring Logic

Scores are accumulated for each region using two types of hints:

- **Country Code Matching**
If a known 2-letter Wi-Fi country code (e.g., `"FR"`, `"US"`, `"CN"`) is found and associated with a region, the region's score is incremented.

- **SSID Pattern Matching**
If the SSID contains known keywords typical for a region (e.g., `"xfinity"` for US), the region's score is incremented.

Both types of matches are independent and additive.

> 💡 Matching is done using `memcmp()` on non-null-terminated strings.

#### Confidence Score Calculation

Once all APs are processed, the `wrf_get_highest_score_region()` function determines the best candidate region:

1. **Top Score Ratio**
   - `top_score_ratio = (top_score * 100) / total_score`
   → How dominant is the top-scoring region?

2. **Top Score Margin**
   - `top_score_margin = (top_score - second_best) * 100 / top_score`
   → How much better is it compared to the second-best?

3. **Final Confidence**
   - `confidence = (top_score_ratio + top_score_margin) / 2`

If the top score is **below a user-defined threshold**, or the result is ambiguous, detection fails and `-1` is returned.

**Customization Required**:
This module is **an example implementation** and should be adapted to:
- Include or exclude specific LoRaWAN regions (e.g., AS923, AU915, etc.).
- Adjust the mapping of **country codes** to regions.
- Define **SSID patterns** commonly found in each region.

You are encouraged to extend the scoring logic with your region-specific ISP SSIDs or additional regulatory domains.

### 2. main loop

**Purpose**:
Serves as an application-level example demonstrating how to:
1. Use the LBM API to perform a Wi-Fi scans.
2. Parse scan results and extract AP metadata.
3. Feed the results into `wifi_region_finder` to determine the most likely region.
4. Display the result and confidence level via UART/logging.

> **Note:** This application will indefinitely perform WiFi scans one after the other, and each time push the results
to the wrf module to update region scoring.

## Getting Started

1. Clone or import the code into your LBM project.
2. Customize the `wifi_region_finder.c` logic as needed for your target regions.
3. Build and flash the firmware to your LR11xx-enabled board.
4. Open a terminal to view the Wi-Fi scan and inferred region.

## Build

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/geolocation/wifi_region_detection -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1110 -G Ninja; cmake --build build --target wifi_region_detection
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/wifi_region_detection verify reset exit"
```
