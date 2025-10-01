# Simplified Camera Stream Viewer

This is a simplified version of a GTK-based camera streaming application. The complex features have been removed to reduce clutter and dependencies.

## Features

- **Simple UI**: Clean GTK3 interface with camera list and details pane
- **Camera Management**: Add, remove, and configure camera streams
- **Basic Settings**: Configure IP, port, encoding, resolution, and framerate
- **Simulated Connection**: Connect/disconnect buttons with status tracking (no actual streaming)

## Removed Complexity

The following features were removed to simplify the codebase:

- Remote SSH camera detection and control
- GStreamer pipeline management
- Video display windows
- Configuration import/export
- Pipeline builder UI
- Threading and background operations
- Credential management
- Complex error handling

## Building

### Install Dependencies

```bash
make install-deps
```

### Compile

```bash
make
```

### Run

```bash
./camera_viewer
```

### Clean

```bash
make clean
```

## Usage

1. **Add Camera**: Click "Add Camera" to create a new camera entry
2. **Configure**: Select a camera from the list to edit its settings
3. **Connect**: Use Connect/Disconnect buttons (simulated - no actual streaming)
4. **Apply Settings**: Update camera parameters

## File Structure

- `main.c` - Application entry point and initialization
- `app_data.h` - Data structures for application state
- `ui_setup.c/h` - UI creation and layout
- `ui_callbacks.c/h` - Event handlers and UI logic
- `config_manager.h` - Minimal config stub
- `Makefile` - Build configuration

## Dependencies

- GTK+ 3.0
- GStreamer 1.0 (minimal usage)
- pkg-config

This simplified version provides a clean foundation that you can extend with the specific features you need without the complexity of the original implementation. 