#!/usr/bin/env zsh
# start_acc_mac.sh -- ACC launcher (macOS host)
#
# Avvia agent + web dashboard nella VM, poi apre il browser sul Mac.
#
# Usage:
#   ./start_acc_mac.sh           # hardware mode (real Pico)
#   ./start_acc_mac.sh --mock    # mock mode (no hardware needed)

VM_HOST="accvm"
VM_SCRIPT="~/start_acc_vm.sh"

echo "=== ACC Launcher (macOS → VM) ==="
ssh -x "$VM_HOST" "bash $VM_SCRIPT $*" &
SSH_PID=$!

# Apri il browser solo quando Flask risponde (dopo che la VM ha rilevato il device)
echo "Waiting for dashboard..."
until curl -s http://192.168.64.10:5000 > /dev/null 2>&1; do
    sleep 0.5
done
open "http://192.168.64.10:5000"

wait $SSH_PID
