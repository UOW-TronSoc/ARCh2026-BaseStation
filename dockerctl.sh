#!/usr/bin/env bash
################################################################################
# dockerctl.sh - Docker Compose Management CLI (Linux)
#
# Usage:
#   ./dockerctl.sh [quick-command]
#
# Examples:
#   ./dockerctl.sh              # Interactive menu
#   ./dockerctl.sh down         # Quick stop
#   ./dockerctl.sh clean        # Quick cleanup
################################################################################

set -euo pipefail

# Config
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="docker-compose.yml"

# Colors
R='\033[0;31m' G='\033[0;32m' Y='\033[1;33m' B='\033[0;34m' N='\033[0m'

# Helpers
err() { echo -e "${R}✗${N} $*" >&2; }
ok() { echo -e "${G}✓${N} $*"; }
info() { echo -e "${B}ℹ${N} $*"; }
warn() { echo -e "${Y}⚠${N} $*"; }

confirm() {
    read -rp "$1 [y/N]: " res
    [[ "$res" =~ ^[yY]$ ]]
}

compose() {
    docker compose -f "$SCRIPT_DIR/$COMPOSE_FILE" "$@"
}

get_service_status() {
    local svc="$1"
    compose ps "$svc" --format "{{.State}}" 2>/dev/null || echo "unknown"
}

# Verify compose file exists
check_compose_file() {
    if [[ ! -f "$SCRIPT_DIR/$COMPOSE_FILE" ]]; then
        err "Compose file not found: $COMPOSE_FILE"
        exit 1
    fi
}

# Quick commands
quick_down() {
    info "Stopping containers..."
    compose down
    ok "Stopped"
}

quick_clean() {
    warn "This will remove ALL containers, volumes, and prune the system"
    confirm "Continue?" || { info "Cancelled"; exit 0; }
    compose down -v
    docker system prune -f
    ok "Cleanup complete"
}

# Container management
start_build() {
    clear
    info "Building and starting..."
    compose up --build -d && ok "Started"
    read -rp "Press ENTER..."
}

start() {
    clear
    info "Starting..."
    compose up -d && ok "Started"
    read -rp "Press ENTER..."
}

stop() {
    clear
    info "Stopping..."
    compose down && ok "Stopped"
    read -rp "Press ENTER..."
}

restart_one() {
    clear
    echo "Services:"
    local svcs && svcs=$(compose ps -a --services 2>/dev/null)
    [[ -z "$svcs" ]] && warn "No services found" && read -rp "Press ENTER..." && return
    
    # Show services with status
    local count=1
    while IFS= read -r svc; do
        local status && status=$(get_service_status "$svc")
        printf "%2d. %-30s [%s]\n" "$count" "$svc" "$status"
        ((count++))
    done <<< "$svcs"
    
    echo
    read -rp "Service name: " svc
    [[ -z "$svc" ]] && return
    
    info "Building and restarting $svc..."
    compose up -d --build "$svc" && ok "Rebuilt and restarted $svc"
    read -rp "Press ENTER..."
}

restart_all() {
    clear
    info "Building and restarting all services..."
    compose up -d --build && ok "Rebuilt and restarted all"
    read -rp "Press ENTER..."
}

# Monitoring
view_ps() {
    clear
    compose ps
    read -rp "Press ENTER..."
}

view_logs() {
    clear
    echo "Services:"
    local svcs && svcs=$(compose ps -a --services 2>/dev/null)
    [[ -z "$svcs" ]] && warn "No services found" && read -rp "Press ENTER..." && return
    
    # Show services with status
    local count=1
    while IFS= read -r svc; do
        local status && status=$(get_service_status "$svc")
        printf "%2d. %-30s [%s]\n" "$count" "$svc" "$status"
        ((count++))
    done <<< "$svcs"
    
    echo
    read -rp "Service name: " svc
    [[ -z "$svc" ]] && return
    
    read -rp "Follow logs? [y/N]: " follow
    echo
    if [[ "$follow" =~ ^[yY]$ ]]; then
        info "Following logs (Ctrl+C to exit)"
        compose logs -f "$svc" || true
    else
        compose logs --tail=100 "$svc"
        read -rp "Press ENTER..."
    fi
}

exec_shell() {
    clear
    echo "Running services:"
    local svcs && svcs=$(compose ps --services --filter "status=running" 2>/dev/null)
    [[ -z "$svcs" ]] && warn "No running services. Start them first" && read -rp "Press ENTER..." && return
    
    echo "$svcs" | nl -w2 -s'. '
    echo
    read -rp "Service name: " svc
    [[ -z "$svc" ]] && return
    
    # Check if running
    local status && status=$(get_service_status "$svc")
    if [[ "$status" != "running" ]]; then
        err "Service '$svc' is not running"
        read -rp "Press ENTER..."
        return
    fi
    
    info "Connecting to $svc (type 'exit' to leave)"
    echo
    compose exec "$svc" /bin/bash 2>/dev/null || compose exec "$svc" /bin/sh
}

# Cleanup operations
prune_containers() {
    clear
    warn "Remove all stopped containers"
    confirm "Continue?" || { info "Cancelled"; read -rp "Press ENTER..."; return; }
    docker container prune -f
    ok "Containers pruned"
    read -rp "Press ENTER..."
}

prune_images() {
    clear
    warn "Remove unused images (not referenced by any container)"
    confirm "Continue?" || { info "Cancelled"; read -rp "Press ENTER..."; return; }
    docker image prune -f
    ok "Images pruned"
    read -rp "Press ENTER..."
}

prune_volumes() {
    clear
    warn "Remove unused volumes - THIS MAY CAUSE DATA LOSS"
    confirm "Are you sure?" || { info "Cancelled"; read -rp "Press ENTER..."; return; }
    docker volume prune -f
    ok "Volumes pruned"
    read -rp "Press ENTER..."
}

full_reset() {
    clear
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ⚠️  DANGER: FULL DOCKER SYSTEM RESET"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "This will remove:"
    echo "  • All stopped containers"
    echo "  • All networks not used by containers"
    echo "  • All images without containers"
    echo "  • All build cache"
    echo "  • All volumes not used by containers"
    echo ""
    echo "⚠️  THIS WILL CAUSE DATA LOSS ⚠️"
    echo ""
    
    confirm "Type YES to proceed" || { info "Cancelled"; read -rp "Press ENTER..."; return; }
    
    read -rp "Type 'DELETE' to confirm: " conf
    [[ "$conf" != "DELETE" ]] && { info "Cancelled"; read -rp "Press ENTER..."; return; }
    
    warn "Performing full system reset..."
    docker system prune -a --volumes -f
    ok "System reset complete"
    read -rp "Press ENTER..."
}

# Menu
show_menu() {
    clear
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Docker Compose Management (Linux)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Compose file: $COMPOSE_FILE"
    echo ""
    echo "Container Management:"
    echo "  1) Build & start       5) Restart all"
    echo "  2) Start               6) View containers"
    echo "  3) Stop"
    echo "  4) Restart service"
    echo ""
    echo "Monitoring:"
    echo "  7) View logs           8) Shell (exec)"
    echo ""
    echo "Cleanup:"
    echo "  9) Prune containers   12) Prune volumes"
    echo " 10) Prune images       13) Full reset (DANGER)"
    echo " 11) Quick prune all"
    echo ""
    echo "  0) Exit"
    echo ""
}

prune_quick() {
    clear
    warn "Quick prune: containers, images, volumes"
    confirm "Continue?" || { info "Cancelled"; read -rp "Press ENTER..."; return; }
    docker container prune -f
    docker image prune -f
    docker volume prune -f
    ok "Pruned"
    read -rp "Press ENTER..."
}

main_loop() {
    while true; do
        show_menu
        read -rp "Select [0-13]: " choice
        case "$choice" in
            1) start_build ;;
            2) start ;;
            3) stop ;;
            4) restart_one ;;
            5) restart_all ;;
            6) view_ps ;;
            7) view_logs ;;
            8) exec_shell ;;
            9) prune_containers ;;
            10) prune_images ;;
            11) prune_quick ;;
            12) prune_volumes ;;
            13) full_reset ;;
            0) clear; info "Goodbye!"; exit 0 ;;
            *) err "Invalid: $choice"; sleep 1 ;;
        esac
    done
}

# Main
main() {
    # Check Docker
    command -v docker &>/dev/null || { err "Docker not installed"; exit 1; }
    docker info &>/dev/null || { err "Docker not running"; exit 1; }
    docker compose version &>/dev/null || { err "Docker Compose not installed"; exit 1; }
    
    # Check OS (Linux only)
    local os && os="$(uname -s)"
    if [[ "$os" != "Linux" ]]; then
        err "This script is configured for Linux only. Detected: $os"
        exit 1
    fi
    
    # Load env if exists
    [[ -f "$SCRIPT_DIR/.env" ]] && set -a && source "$SCRIPT_DIR/.env" && set +a
    
    # Verify compose file exists
    check_compose_file
    
    # Quick commands
    local cmd="${1:-}"
    case "$cmd" in
        down) quick_down; exit 0 ;;
        clean) quick_clean; exit 0 ;;
        help|--help|-h)
            echo "Usage: ./dockerctl.sh [command]"
            echo ""
            echo "Commands:"
            echo "  (no args)    Interactive menu"
            echo "  down         Stop all containers"
            echo "  clean        Stop and remove all containers/volumes"
            echo "  help         Show this help"
            echo ""
            echo "Examples:"
            echo "  ./dockerctl.sh           # Interactive menu"
            echo "  ./dockerctl.sh down      # Quick stop"
            echo "  ./dockerctl.sh clean     # Full cleanup"
            exit 0
            ;;
        "")
            # No command, show menu
            ;;
        *)
            err "Unknown command: $cmd (use: down, clean, help)"
            exit 1
            ;;
    esac
    
    main_loop
}

main "$@"
