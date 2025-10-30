# ugv-rover

## Build

Use the build helper script for out-of-source builds into `build/`.

```bash
./build.sh            # Release
./build.sh debug      # Debug
./build.sh clean      # Clean
```

### Running

Executable is placed in `build/` as `rover_control`.

```bash
./build/rover_control
```

Optionally:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### Dependencies

- Boost components: `system`, `thread`
- `pthread` (via `Threads`)
- JsonCpp (optional; detected via `pkg-config` or CMake if available)

On Debian/Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libboost-system-dev libboost-thread-dev libjsoncpp-dev pkg-config
```
