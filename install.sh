#!/usr/bin/env bash
# ==============================================================================
#        RUST MLAT SERVER - INTERACTIVE INSTALLER & SETUP WIZARD
# ==============================================================================
# A high-performance, low-memory Multilateration (MLAT) server written in Rust.
# Compatible with standard mlat-client, ultrafeeder, readsb, and tar1090.
# ==============================================================================

set -e

# Terminal formatting colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

clear
echo -e "${CYAN}${BOLD}"
echo "  ╔═══════════════════════════════════════════════════════════════════╗"
echo "  ║             🛰️  RUST MLAT SERVER INSTALLER & SETUP                ║"
echo "  ║        High-Performance Multi-threaded MLAT Server in Rust        ║"
echo "  ╚═══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# 1. Ensure Root Privileges
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}[!] This installer requires root privileges to set up systemd services.${NC}"
    echo -e "    Relaunching automatically with sudo...\n"
    exec sudo bash "$0" "$@"
fi

# 2. Check & Install Prerequisites (Rust & Cargo Toolchain)
echo -e "${BLUE}[1/5] Checking build dependencies...${NC}"
if ! command -v cargo &> /dev/null; then
    echo -e "${YELLOW}[*] Rust and Cargo toolchain not found. Installing Rustup...${NC}"
    if command -v apt-get &> /dev/null; then
        apt-get update -qq && apt-get install -y build-essential pkg-config libssl-dev curl
    elif command -v dnf &> /dev/null; then
        dnf groupinstall -y "Development Tools" && dnf install -y pkgconfig openssl-devel curl
    elif command -v yum &> /dev/null; then
        yum groupinstall -y "Development Tools" && yum install -y pkgconfig openssl-devel curl
    fi
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    export PATH="$HOME/.cargo/bin:/root/.cargo/bin:$PATH"
    source "$HOME/.cargo/env" 2>/dev/null || true
else
    echo -e "${GREEN}[✓] Rust toolchain is installed: $(cargo --version)${NC}"
fi

# 3. Interactive Configuration Wizard
echo -e "\n${BLUE}[2/5] Server Configuration Parameters:${NC}"
echo -e "${CYAN}-----------------------------------------------------------------${NC}"

# Prompt: Feeder Ingestion Port
read -p "$(echo -e "${BOLD}MLAT Feeder Ingestion TCP Port${NC} [Default: ${GREEN}41113${NC}]: ")" INPUT_CLIENT_PORT
CLIENT_PORT=${INPUT_CLIENT_PORT:-41113}

# Prompt: Feeder Listen IP
read -p "$(echo -e "${BOLD}MLAT Feeder Listen IP Address${NC} [Default: ${GREEN}0.0.0.0${NC}]: ")" INPUT_CLIENT_IP
CLIENT_IP=${INPUT_CLIENT_IP:-0.0.0.0}

# Prompt: BaseStation SBS Output Port
read -p "$(echo -e "${BOLD}BaseStation SBS Output Port (for readsb/tar1090)${NC} [Default: ${GREEN}32007${NC}]: ")" INPUT_SBS_PORT
SBS_PORT=${INPUT_SBS_PORT:-32007}

# Prompt: BaseStation SBS Bind IP
read -p "$(echo -e "${BOLD}BaseStation SBS Bind IP Address${NC} [Default: ${GREEN}127.0.0.1${NC}]: ")" INPUT_SBS_IP
SBS_IP=${INPUT_SBS_IP:-127.0.0.1}

# Prompt: Working Directory for JSON outputs
read -p "$(echo -e "${BOLD}Working Directory for telemetry (clients.json & sync.json)${NC} [Default: ${GREEN}/var/lib/mlat-server${NC}]: ")" INPUT_WORKDIR
WORKDIR=${INPUT_WORKDIR:-/var/lib/mlat-server}

# Prompt: Systemd Service Name
read -p "$(echo -e "${BOLD}Systemd Service Name${NC} [Default: ${GREEN}adsb-mlat-server${NC}]: ")" INPUT_SERVICE_NAME
SERVICE_NAME=${INPUT_SERVICE_NAME:-adsb-mlat-server}

echo -e "${CYAN}-----------------------------------------------------------------${NC}"
echo -e "${BOLD}Configuration Summary:${NC}"
echo -e "  • Feeder Ingestion Address : ${GREEN}${CLIENT_IP}:${CLIENT_PORT}/tcp${NC}"
echo -e "  • BaseStation SBS Egress   : ${GREEN}${SBS_IP}:${SBS_PORT}/tcp${NC}"
echo -e "  • Working Directory        : ${GREEN}${WORKDIR}${NC}"
echo -e "  • Systemd Service          : ${GREEN}${SERVICE_NAME}.service${NC}"
echo -e "${CYAN}-----------------------------------------------------------------${NC}"

read -p "Proceed with compilation and installation? [Y/n]: " CONFIRM
if [[ "$CONFIRM" =~ ^[Nn]$ ]]; then
    echo -e "${RED}[X] Installation cancelled by user.${NC}"
    exit 1
fi

# 4. Compile Optimized Release Binary
echo -e "\n${BLUE}[3/5] Compiling optimized Rust release binary...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PATH="$HOME/.cargo/bin:/root/.cargo/bin:$PATH"
cargo build --release

BIN_PATH="/usr/local/bin/${SERVICE_NAME}"
cp -f target/release/adsbitalia-mlat-server "$BIN_PATH"
chmod 755 "$BIN_PATH"
echo -e "${GREEN}[✓] Binary installed at: ${BIN_PATH}${NC}"

# 5. Create Data Directory
echo -e "\n${BLUE}[4/5] Preparing working directory...${NC}"
mkdir -p "$WORKDIR"
chmod 755 "$WORKDIR"
echo -e "${GREEN}[✓] Data directory ready: ${WORKDIR}${NC}"

# 6. Configure and Start Systemd Service
echo -e "\n${BLUE}[5/5] Creating and starting Systemd service...${NC}"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

cat << EOF > "$SERVICE_FILE"
[Unit]
Description=High-Performance Rust MLAT Server
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=${WORKDIR}
Environment="RUST_LOG=info"
Environment="MALLOC_ARENA_MAX=2"
ExecStart=${BIN_PATH} \\
  --client-listen ${CLIENT_IP}:${CLIENT_PORT} \\
  --basestation-listen ${SBS_IP}:${SBS_PORT} \\
  --work-dir ${WORKDIR}
Restart=always
RestartSec=5
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}.service"

echo -e "\n${GREEN}${BOLD}═══════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}         🎉 INSTALLATION COMPLETED SUCCESSFULLY!                  ${NC}"
echo -e "${GREEN}${BOLD}═══════════════════════════════════════════════════════════════════${NC}"
echo -e "Service Status : $(systemctl is-active ${SERVICE_NAME}.service)"
echo -e "Feeder Listen  : ${GREEN}${CLIENT_IP}:${CLIENT_PORT}/tcp${NC}"
echo -e "SBS Listen     : ${GREEN}${SBS_IP}:${SBS_PORT}/tcp${NC}"
echo -e "Data Directory : ${GREEN}${WORKDIR}${NC}"

# 7. Manual Firewall Guidance
echo -e "\n${YELLOW}${BOLD}⚠️  FIREWALL CONFIGURATION (MANUAL ACTION REQUIRED):${NC}"
echo -e "To allow incoming connections from your remote feeders, please ensure"
echo -e "port ${GREEN}${CLIENT_PORT}/tcp${NC} is open in your firewall:\n"

if command -v ufw &> /dev/null; then
    echo -e "  • ${BOLD}UFW (Ubuntu/Debian):${NC}"
    echo -e "    ${CYAN}sudo ufw allow ${CLIENT_PORT}/tcp comment \"MLAT Feeder Ingress\"${NC}\n"
fi

if command -v firewall-cmd &> /dev/null; then
    echo -e "  • ${BOLD}firewalld (RHEL/CentOS/Fedora):${NC}"
    echo -e "    ${CYAN}sudo firewall-cmd --permanent --add-port=${CLIENT_PORT}/tcp${NC}"
    echo -e "    ${CYAN}sudo firewall-cmd --reload${NC}\n"
fi

echo -e "  • ${BOLD}iptables:${NC}"
echo -e "    ${CYAN}sudo iptables -A INPUT -p tcp --dport ${CLIENT_PORT} -j ACCEPT${NC}\n"

echo -e "  • ${BOLD}Cloud Hosting (AWS, Hetzner, Oracle Cloud, GCP, DigitalOcean):${NC}"
echo -e "    Remember to allow inbound ${GREEN}TCP ${CLIENT_PORT}${NC} in your cloud provider's"
echo -e "    Security Group or Network Firewall dashboard.\n"

# 8. Service Management & Readsb Integration Notes
echo -e "${BOLD}Management commands:${NC}"
echo -e "  • View live server logs : ${CYAN}journalctl -u ${SERVICE_NAME} -f${NC}"
echo -e "  • Restart service       : ${CYAN}sudo systemctl restart ${SERVICE_NAME}${NC}"
echo -e "  • Stop service          : ${CYAN}sudo systemctl stop ${SERVICE_NAME}${NC}"

echo -e "\n${BOLD}Readsb / Tar1090 Integration:${NC}"
echo -e "To display MLAT aircraft on your map, add the following connector to ${CYAN}/etc/default/readsb${NC}:"
echo -e "  ${YELLOW}--net-connector=${SBS_IP},${SBS_PORT},sbs_in_mlat${NC}"
echo -e "Then restart readsb: ${CYAN}sudo systemctl restart readsb${NC}\n"
