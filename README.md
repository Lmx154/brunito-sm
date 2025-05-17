# Brunito Project

This repository contains code for the Brunito project, which consists of three main components:
- Navigation Controller (NAVC)
- Flight Controller (FC)
- Ground Station (GS)

## Build and Upload Instructions

The project uses PlatformIO for building and uploading firmware. You can selectively build and upload each component independently.

### Available Commands

#### Upload a specific component

```bash
# Upload only the Navigation Controller
pio run -e navc -t upload

# Upload only the Flight Controller
pio run -e fc -t upload

# Upload only the Ground Station
pio run -e gs -t upload
```

#### Upload the entire project

To build and upload all components:

```bash
pio run -e blackpill_f401cc -t upload
```

#### Build without uploading

To build without uploading (for example, to check for compilation errors):

```bash
# Build only the Navigation Controller
pio run -e navc

# Build only the Flight Controller
pio run -e fc

# Build only the Ground Station
pio run -e gs

# Build the entire project
pio run -e blackpill_f401cc
```

## Project Structure

Each component is contained in its own source file:
- `NAVC.cpp` - Navigation Controller
- `FC.cpp` - Flight Controller
- `GS.cpp` - Ground Station

The PlatformIO configuration includes separate build environments for each component, allowing you to work on each part independently without having to upload the entire firmware.