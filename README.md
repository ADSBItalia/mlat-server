<p align="center">
  <img src="assets/logo.png" alt="ADSBItalia Logo" width="380">
</p>

<h1 align="center">🛰️ ADSBItalia Native Rust MLAT Server</h1>

<p align="center">
  <b>An ultra-high-performance, production-grade Multilateration (MLAT) server written from scratch in native Rust.</b><br>
  Designed for modern flight tracking networks, community ADS-B aggregators, and high-density SDR receiver meshes.
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Language-Rust_1.80+-orange.svg?style=flat-square&logo=rust" alt="Rust 1.80+"/>
  <img src="https://img.shields.io/badge/Async_Runtime-Tokio-blue.svg?style=flat-square&logo=tokio" alt="Tokio"/>
  <img src="https://img.shields.io/badge/Memory_Footprint-35MB-green.svg?style=flat-square" alt="35MB Memory"/>
  <img src="https://img.shields.io/badge/Tested_Feeders-500+_Live-brightgreen.svg?style=flat-square" alt="500+ Feeders"/>
  <img src="https://img.shields.io/badge/License-GPL--3.0-blue.svg?style=flat-square" alt="GPL-3.0 License"/>
  <img src="https://img.shields.io/badge/Production_Status-Active-success.svg?style=flat-square" alt="Production Active"/>
</p>

---

## 📖 Overview

For nearly a decade, open-source flight-tracking aggregators (such as ADSBexchange, Airplanes.live, and regional SDR communities) have relied on the legacy Python implementation of `mlat-server` (originally created by *mutability* and maintained by *wiedehopf*).

While pioneering for its time, the legacy Python daemon presents severe operational bottlenecks at scale:
* **Massive Memory Bloat:** Consumes **2 to 3+ GB of RAM** with persistent memory leaks under hundreds of feeders.
* **CPython GIL Contention:** Python's Global Interpreter Lock bottlenecks multi-core solving under heavy message bursts.
* **Incomplete Kinematics:** Does not calculate **Vertical Rate ($V_z$ ft/min)**, leaving aircraft profiles incomplete in `readsb` and `tar1090`.
* **Rigid DOF Requirements:** Drops non-barometric squitters (e.g. Mode-S DF11, surface monitors, military Mode-A/C) if fewer than 4 stations are synchronized in the same clique.

**ADSBItalia Native Rust MLAT Server** was developed to eliminate these bottlenecks completely. It replaces the legacy architecture with a lock-free, zero-allocation, multi-threaded engine built on top of **Tokio**, **DashMap**, and a custom **Levenberg-Marquardt WGS84 Geodetic TDoA Solver**.

---

## ⚡ Performance Comparison (Live Production Benchmark)

The following metrics were collected on the live **ADSBItalia** network with **492 concurrent feeder stations** across Europe and the Mediterranean:

| Metric | Legacy Python `mlat-server` | **ADSBItalia Rust MLAT** | Difference |
| :--- | :--- | :--- | :--- |
| **RAM Utilization (RSS)** | **~2,150 MB** (and climbing) | **~35 – 80 MB** (flat) | **-96% Memory Reduction** ⚡ |
| **Memory Management** | Python GC with known cyclic leaks | **Zero GC / RAII + `mimalloc`** | Zero leaks, flat profile |
| **Concurrency Architecture** | Process multiprocessing + GIL | **Asynchronous Tokio + Lock-free Concurrency** | True multi-core scalability |
| **Kinematic Parameters** | Position + Groundspeed + Track | **Position + Speed + Track + Vertical Rate ($V_z$)** | 100% full flight envelope |
| **Minimum Required Receivers** | Strictly $\ge 4$ without baro alt | **$\ge 3$ stations with WGS84 Ellipsoid constraint** | Solves more targets |
| **Stationary Target Filter** | Emits jitter noise as motion | **Integrated Stationary Anchor Filter** | Zero fake speed on towers |
| **Deployment Complexity** | Python virtualenv, gcc, cython, numpy | **Single self-contained binary** (~15 MB) | Zero runtime dependencies |

---

## 🧠 Core Architecture & Mathematical Principles

```
  [ 500+ Remote Receivers ]  (mlat-client / ultrafeeder / readsb)
              │
              ▼ (Port 41113/TCP - Raw or PROXY Protocol v1)
   ┌────────────────────────────────────────────────────────┐
   │             Tokio Non-Blocking Network Core            │
   │  - Streaming zlib decompression per client             │
   │  - Lock-free message ingest & timestamp normalization  │
   └──────────────────────────┬─────────────────────────────┘
                              │
                              ▼
   ┌────────────────────────────────────────────────────────┐
   │             Clock Synchronization Graph                │
   │  - Multi-hop pairwise drift & offset calibration       │
   │  - 2,600+ real-time synchronized station pairs         │
   └──────────────────────────┬─────────────────────────────┘
                              │
                              ▼
   ┌────────────────────────────────────────────────────────┐
   │            Exact Hyperbolic TDoA Solver                │
   │  - Levenberg-Marquardt on WGS84 Earth Ellipsoid        │
   │  - RAIM residual pruning & geometric sanity checks     │
   └──────────────────────────┬─────────────────────────────┘
                              │
                              ▼
   ┌────────────────────────────────────────────────────────┐
   │       Adaptive α-β Trajectory & Kinematic Smoother      │
   │  - GDOP-weighted position smoothing                    │
   │  - Ground speed (knots) & Track heading (degrees)      │
   │  - Vertical Rate (ft/min) from barometric delta        │
   │  - Stationary Anchor (suppresses jitter on towers)     │
   └──────────────────────────┬─────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
 [ BaseStation SBS (Port 32007) ]    [ JSON Telemetry (/var/lib/mlat-server) ]
   └──> readsb / tar1090 live map      └──> clients.json & sync.json for web APIs
```

### 1. High-Precision Time Difference of Arrival (TDoA)
When a Mode-S / Mode-A/C transponder emits a message, multiple ground receivers detect it at nanosecond-scale timestamps:
$$\Delta t_{ij} = t_i - t_j = \frac{\|\mathbf{x} - \mathbf{s}_i\| - \|\mathbf{x} - \mathbf{s}_j\|}{c}$$
Where $\mathbf{x} = (X, Y, Z)$ is the aircraft's Cartesian ECEF position, $\mathbf{s}_i$ is the $i$-th receiver's coordinates, and $c$ is the speed of light in air ($299,702,547 \text{ m/s}$).

The solver uses a damped non-linear least squares algorithm (**Levenberg-Marquardt**) constrained to the **WGS84 Earth Ellipsoid**:
$$\frac{X^2 + Y^2}{a^2} + \frac{Z^2}{b^2} = 1$$
This enables robust positioning even with **only 3 synchronized receivers** when altitude is constant or on the ground.

### 2. Clock Synchronization Graph
Receivers using standard low-cost RTL-SDR dongles experience oscillator frequency drift. The server automatically constructs an inter-receiver clock sync graph:
* Pairs of stations that simultaneously receive identical reference ADS-B frames compute relative clock offsets and frequency drift.
* Station pairs are continuously calibrated, supporting multi-hop synchronization across hundreds of stations without requiring hardware GPS-disciplined oscillators (GPSDO).

### 3. Adaptive Kinematic Filtering & Vertical Rate
* **Ground Speed & Track Heading:** Calculated from smoothed East-North-Up (ENU) velocity vectors and filtered with angular wrap-around smoothing.
* **Vertical Rate ($V_z$ in ft/min):** Computed from genuine barometric altitude change between successive Mode-S frames ($\Delta \text{alt} / \Delta t$), clamped and rounded to ICAO standard 64 ft/min increments.
* **Stationary Target Anchor:** Detects stationary transmitters (such as Surface Surveillance Monitors, ground towers, and parked aircraft) whose net displacement over 6 seconds is under 40 knots, cleanly suppressing residual TDoA jitter and preventing false speed or climb rates.

---

## 🚀 Quick Start (Automated Installer)

The easiest way to install and configure the server is using the interactive English wizard:

```bash
git clone https://github.com/ADSBItalia/mlat-server.git
cd mlat-server
sudo bash install.sh
```

### The interactive wizard will:
1. Detect and verify your **Rust/Cargo** toolchain (installs Rustup automatically if missing).
2. Prompt you for custom ports and settings:
   * **Feeder TCP Port** (Default: `41113`)
   * **BaseStation SBS Port** (Default: `32007`)
   * **Working Directory** (Default: `/var/lib/mlat-server`)
   * **Service Name** (Default: `adsb-mlat-server`)
3. Compile the optimized release binary (`cargo build --release`).
4. Install the binary to `/usr/local/bin/`.
5. Create and enable the systemd service.
6. Provide exact firewall instructions for your system.

---

## 🔒 Firewall Configuration (Manual Step)

To receive incoming traffic from remote feeders, allow inbound connections on your chosen Feeder Ingestion Port (e.g. `41113/tcp`):

* **UFW (Ubuntu / Debian):**
  ```bash
  sudo ufw allow 41113/tcp comment "MLAT Feeder Ingress"
  ```

* **firewalld (RHEL / CentOS / Fedora):**
  ```bash
  sudo firewall-cmd --permanent --add-port=41113/tcp
  sudo firewall-cmd --reload
  ```

* **iptables:**
  ```bash
  sudo iptables -A INPUT -p tcp --dport 41113 -j ACCEPT
  ```

* **Cloud Providers (AWS, Hetzner, Oracle Cloud, GCP, DigitalOcean, OVH):**
  Ensure TCP port `41113` is permitted in your cloud provider's **Security Group** or **Firewall Rules** dashboard.

---

## 📡 Integrating with `readsb` and `tar1090`

To stream multilaterated aircraft directly into your live `readsb` map, edit `/etc/default/readsb` and add the SBS connector:

```text
--net-connector=127.0.0.1,32007,sbs_in_mlat
```

Restart `readsb`:
```bash
sudo systemctl restart readsb
```

Aircraft multilaterated by the Rust server will immediately display on **tar1090** with the green `MLAT` badge, complete with coordinates, speed, track, and vertical rate!

---

## 🛠️ Manual Build & Command-Line Usage

You can also compile and run the binary manually:

```bash
# Build optimized release binary
cargo build --release

# Run standalone
./target/release/adsbitalia-mlat-server \
  --client-listen 0.0.0.0:41113 \
  --basestation-listen 127.0.0.1:32007 \
  --work-dir /var/lib/mlat-server
```

### Available Command-Line Arguments:
* `--client-listen [HOST:]PORT`: TCP address and port for incoming feeder client connections (default: `0.0.0.0:41113`). Supports both direct connections and **HAProxy PROXY Protocol v1**.
* `--basestation-listen [HOST:]PORT`: TCP address and port for outbound BaseStation SBS feed (default: `127.0.0.1:32007`).
* `--work-dir PATH`: Directory where `clients.json` and `sync.json` telemetry files are written (default: `/var/lib/mlat-server`).

---

## 📊 Service Management

```bash
# View live real-time logs
sudo journalctl -u adsb-mlat-server -f

# Restart service
sudo systemctl restart adsb-mlat-server

# Check service status & RAM usage
sudo systemctl status adsb-mlat-server
```

---

## 🗑️ Uninstallation

To cleanly remove the service and binary:
```bash
sudo bash uninstall.sh
```

---

## 📜 License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**. See the [LICENSE](LICENSE) file for details.

---

## 🤝 Acknowledgments & Credits

* **[ADSBItalia](https://adsbitalia.it)** — The Italian community flight tracking network.
* **wiedehopf & mutability** — For their pioneering work on the original Python `mlat-server` and `mlat-client` protocols.
* **readsb & tar1090** — For outstanding open-source ADS-B demodulation and web visualization.
