#!/usr/bin/env bash
# Enable/disable pf NAT for Bramble CYW43 utun subnet (192.168.4.0/24) on macOS.
#
# Usage:
#   sudo ./scripts/macos-cyw43-pf-nat.sh enable
#   sudo ./scripts/macos-cyw43-pf-nat.sh disable
#   ./scripts/macos-cyw43-pf-nat.sh status
#
# Uses a dedicated pf anchor so we do not rewrite the user's global ruleset.
set -euo pipefail

ANCHOR="bramble_cyw43"
SUBNET="192.168.4.0/24"
TMP_DIR="${TMPDIR:-/tmp}"
ANCHOR_FILE="$TMP_DIR/bramble-cyw43-pf.anchor"
CONF_FILE="$TMP_DIR/bramble-cyw43-pf.conf"

detect_outgoing() {
  route -n get default 2>/dev/null | awk '/interface:/{print $2; exit}'
}

cmd="${1:-status}"
case "$cmd" in
  enable)
    if [[ "$(id -u)" -ne 0 ]]; then
      echo "Need root: sudo $0 enable" >&2
      exit 1
    fi
    out_if="$(detect_outgoing)"
    if [[ -z "$out_if" ]]; then
      echo "No default route interface found" >&2
      exit 1
    fi
    cat >"$ANCHOR_FILE" <<EOF
# Bramble CYW43 guest subnet → $out_if
nat on $out_if from $SUBNET to any -> ($out_if)
pass from $SUBNET to any keep state
pass from any to $SUBNET keep state
EOF
    cat >"$CONF_FILE" <<EOF
# Minimal loader for Bramble anchor (does not replace system pf.conf)
anchor "$ANCHOR"
load anchor "$ANCHOR" from "$ANCHOR_FILE"
EOF
    # Enable pf if needed, then load only our anchor into the running ruleset.
    pfctl -e 2>/dev/null || true
    if ! pfctl -a "$ANCHOR" -f "$ANCHOR_FILE" 2>/dev/null; then
      # Some systems require referencing the anchor from main; try flush+load via conf.
      pfctl -f "$CONF_FILE" 2>/dev/null || {
        echo "pfctl failed to load anchor '$ANCHOR'." >&2
        echo "You can also add this to /etc/pf.conf and 'sudo pfctl -f /etc/pf.conf':" >&2
        echo "  anchor \"$ANCHOR\"" >&2
        echo "  load anchor \"$ANCHOR\" from \"$ANCHOR_FILE\"" >&2
        exit 1
      }
    fi
    sysctl -w net.inet.ip.forwarding=1 >/dev/null
    echo "[bramble-pf] NAT enabled: $SUBNET → $out_if (anchor $ANCHOR)"
    ;;
  disable)
    if [[ "$(id -u)" -ne 0 ]]; then
      echo "Need root: sudo $0 disable" >&2
      exit 1
    fi
    pfctl -a "$ANCHOR" -F all 2>/dev/null || true
    echo "[bramble-pf] anchor $ANCHOR flushed"
    ;;
  status)
    echo "default iface: $(detect_outgoing || echo none)"
    echo "--- pf anchors ---"
    pfctl -s Anchors 2>/dev/null | grep -F "$ANCHOR" || echo "(no $ANCHOR)"
    echo "--- $ANCHOR NAT ---"
    pfctl -a "$ANCHOR" -s nat 2>/dev/null || true
    ;;
  *)
    echo "Usage: $0 enable|disable|status" >&2
    exit 2
    ;;
esac
