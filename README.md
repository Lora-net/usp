# Unified Software Platform (USP)

> **⚠️ ALPHA RELEASE v0.5.1** - Not all samples have been fully validated. Version 1.0.0 coming soon.

## Overview

USP provides an abstraction layer for scheduling and managing radio access across multiple modulations (LoRa, FSK, LR-FHSS, FLRC) and protocols (LoRaWAN). The library enables applications to request radio access, configure transmissions/receptions, and schedule operations with priority management.

## Key Features

- **Priority-based scheduling** - Manage radio access with configurable priorities
- **Multi-modulation support** - LoRa, FSK, LR-FHSS, and FLRC modulations
- **LoRa capabilities** - Full support for transmission, reception, and ranging
- **Precise timing** - Schedule radio operations with accurate timing control
- **Asynchronous operations** - Callback support for non-blocking execution
- **Seamless integration** - Built on Semtech's radio planner

## Architecture

### Components

| Component | Description |
|-----------|-------------|
| **RAC Library** | Radio Access Component (RAC) API for Semtech transceiver management |
| **LoRa Basics Modem** | Integrated LoRaWAN stack (v4.9.0) |
| **Examples** | Sample applications demonstrating RAC API usage |

### Supported Hardware

- **Nucleo-STM32L476RG** development board
- **LoRa Plus Expansion Board** with Wio-LR2021

## Getting Started

📚 **[View Full API Documentation →](smtc_rac_lib/README.md)**

## Roadmap

**Version 1.0.0** (Coming Soon)
- ✅ Complete validation of all samples
- ✅ Bug fixes and stability improvements
- ✅ Production-ready release
