# MSPM0 Car Project Structure

## Directory Layout

This project now follows a CubeMX-inspired layout while preserving MSPM0-specific tooling.

- `Core/`
  - `Src/`: application entry and top-level runtime flow
  - `Inc/`: project-wide public headers and build configuration headers
- `Board/`
  - `SysConfig/`: SysConfig-generated board configuration sources
  - `Startup/`: startup assembly and linker command/scatter files
  - `Target/`: target configuration files such as `.ccxml`
- `drivers/`
  - `BSP/`: peripheral and board support drivers
  - `Platform/`: platform-level common headers and system services
- `Modules/`
  - `Control/`: control, tracking, kinematics, and sensor-processing logic
  - `Mission/`: menu, mode tree, and competition task orchestration
- `Project/`
  - `Keil/`: Keil project files
  - `CCS/`: CCS import specification files

## Current Refactor Boundary

This refactor focuses on:

- reorganizing source files into clearer responsibility-based folders
- updating Keil and CCS project metadata to match the new layout
- preserving current runtime behavior as much as possible

The following are intentionally deferred to a later cleanup pass:

- removing `AllHeader.h` and replacing it with explicit module dependencies
- consolidating scattered global state into dedicated modules
- splitting mission logic from direct hardware actuation calls

## Development Notes

- Prefer adding new board-generated files under `Board/SysConfig/`.
- Prefer adding hardware-facing code under `drivers/BSP/`.
- Prefer adding reusable control logic under `Modules/Control/`.
- Prefer adding competition-flow or UI orchestration code under `Modules/Mission/`.
