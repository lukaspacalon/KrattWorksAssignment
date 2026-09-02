# KrattWorksAssignment
# Drone MAVLink/UDP — Architecture & Documentation

A drone simulator and Ground Control Station (GCS) project communicating via the MAVLink protocol over UDP. 
This project applies a modern and robust C++ system architecture based on **Hexagonal Architecture**, declarative programming, and deterministic pure functions.


## Architecture & Design Choices

Among the 6 potential architectures (*Layered*, *Hexagonal*, *Reactor/Proactor*, *Actor Model*, *Pub/Sub*, *Pipes & Filters*), this project adopts a **Hexagonal Architecture (Ports & Adapters)** as its core framework, combined with a **Reactor** pattern as the transport adapter.

* Maximum Testability & Race-Free Concurrency: Complete isolation of I/O dependencies.
* Architectural Clarity: Every boundary has a distinct responsibility that can be justified in a single sentence.

## Design Invariants

Invariant 
**MAVLink confined to protocol + codec module**
Generated C structs must never leak into flight logic or GUI (prevents full rewrites if changing dialect)

**UDP behind an `ITransport` interface**
Enables deterministic integration tests via in-memory loopbacks and packet drop injection

**Deterministic flight logic (`update(dt, now)`)**
`now` and `dt` are injected explicitly (no direct `Clock::now()` calls), eliminating flaky tests.

**Domain loop in the main thread**
Simulation runs in the startup thread (Drone side), GUI runs in the startup thread (GCS side).

## Declarative & Functional Approach (Air Safety)

1. **Safety by Construction (Declarative):**
   Instead of scattering imperative `if/switch` statements that risk missing critical conditions (e.g., disarming mid-flight), state transition rules are centralized in a **constexpr truth table**. Any undeclared transition is physically impossible to execute.
2. **Determinism & Bug Reproducibility:**
   Using pure functions, any bug occurring at a specific millisecond can be reproduced at 100% precision down to the nanosecond—without requiring network threads or running active processes.
3. **Elimination of Race Conditions:**
   Immutable *Value Objects* and snapshots are used for communication between the UDP/MAVLink network thread and the main simulation thread.

## 🛠️ C++ Mastery & Code Structure

To ensure a real-time system with low latency:
* **Memory Management:** Zero hidden dynamic allocations inside high-frequency execution loops (100 Hz).
* **RAII & Lifetimes:** Strict ownership tracking (`std::unique_ptr`, `std::shared_ptr`, references).
* **Directory Layout:**
  * `src/domain/` : Pure domain logic (no sockets, no MAVLink).
  * `src/adapters/` : MAVLink translation, UDP sockets, and network handling.
  * `src/app/` / `src/bin/` : Program entry points and dependency injection.

## 🚀 Getting Started

### 1. Extract the Archive

tar -xzf kratt-complete-adapters.tar.gz
cd kratt-hex

### 2. Quick Verification (Hello World)

./tools/hello.sh
Expected output: build OK followed by SUCCESS: the GCS saw the drone's heartbeat.

### 3. Running Tests

# Run the full test suite
./tools/diag.sh

# Run geofence tests only
./tools/diag.sh geofence

Expected output: 100% tests passed out of 116

### 4. Running in Production (Drone & GCS) 

Open two separate terminal windows:

Terminal 1 — Drone:
./build/bin/Drone --bind-port 14551 --gcs 127.0.0.1:14550

Terminal 2 — GCS:
./build/bin/GCS --bind-port 14550