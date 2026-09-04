#!/usr/bin/env bash
# ==============================================================================
#        RUST MLAT SERVER - UNINSTALL SCRIPT
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}[!] This script requires root privileges to remove systemd services.${NC}"
    exec sudo bash "$0" "$@"
fi

read -p "Enter the systemd service name to uninstall [Default: adsb-mlat-server]: " INPUT_SERVICE
SERVICE_NAME=${INPUT_SERVICE:-adsb-mlat-server}

echo -e "${RED}[*] Stopping and disabling ${SERVICE_NAME}.service...${NC}"
systemctl stop "${SERVICE_NAME}.service" 2>/dev/null || true
systemctl disable "${SERVICE_NAME}.service" 2>/dev/null || true

rm -f "/etc/systemd/system/${SERVICE_NAME}.service"
rm -f "/usr/local/bin/${SERVICE_NAME}"
systemctl daemon-reload

echo -e "${GREEN}[✓] Uninstallation completed successfully.${NC}"
