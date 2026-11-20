#!/bin/bash
#
# T3 Foundation Gemstone Project [t3gemstone.org]
#
# This file is copied from the jetify.com/devbox
# SPDX-License-Identifier: Apache-2.0
#
# Installs Docker and Jetify Devbox on the development machine. Do not run it
# on the robot (T3 Gemstone O1); to install the robot: bash scripts/install-native.sh

set -euo pipefail

readonly TITLE="🚀 T3 Gemstone UGV ROS"
readonly DOCS_URL="https://docs.t3gemstone.org/en/introduction"
readonly COMMUNITY_URL="https://community.t3gemstone.org"

BOLD="$(tput bold 2>/dev/null || echo '')"
GREY="$(tput setaf 8 2>/dev/null || echo '')"
UNDERLINE="$(tput smul 2>/dev/null || echo '')"
RED="$(tput setaf 1 2>/dev/null || echo '')"
GREEN="$(tput setaf 2 2>/dev/null || echo '')"
YELLOW="$(tput setaf 3 2>/dev/null || echo '')"
BLUE="$(tput setaf 4 2>/dev/null || echo '')"
MAGENTA="$(tput setaf 5 2>/dev/null || echo '')"
NO_COLOR="$(tput sgr0 2>/dev/null || echo '')"
readonly BOLD GREY UNDERLINE RED GREEN YELLOW BLUE MAGENTA NO_COLOR
readonly CLEAR_LAST_MSG="\033[1F\033[0K"

DESCRIPTION=$(
    cat <<EOF
  This script installs Docker and Jetify Devbox 📦, the foundation of the UGV ROS development environment.
  Building, testing and trial runs without hardware happen in a Distrobox environment with ROS 2 Jazzy.
  Powered by Jetify Devbox, Distrobox, ${BOLD}Love${NO_COLOR} and ${BOLD}Passion${NO_COLOR}.
EOF
)
readonly DESCRIPTION

SILENT="${SILENT:-0}"

parse_flags() {
    while [ "$#" -gt 0 ]; do
        case "$1" in
        -s | --silent)
            SILENT=1
            shift 1
            ;;
        *)
            error "Unknown option: $1"
            exit 1
            ;;
        esac
    done
}

has() {
    command -v "$1" 1>/dev/null 2>&1
}

delay() {
    sleep "${1:-0.3}"
}

title() {
    printf "\n%s\n" "${BOLD}${MAGENTA}$*${NO_COLOR}"
}

header() {
    printf "%s\n" "${BOLD}$*${NO_COLOR}"
}

plain() {
    printf "%s\n" "$*"
}

info() {
    printf "%s\n" "${BOLD}${GREY}→${NO_COLOR} $*"
}

warn() {
    printf "%s\n" "${YELLOW}! $*${NO_COLOR}"
}

error() {
    printf "%s\n" "${RED}✘ $*${NO_COLOR}" >&2
}

success() {
    printf "%s\n" "${GREEN}✓${NO_COLOR} $*"
}

start_task() {
    printf "%s\n" "${BOLD}${GREY}→${NO_COLOR} $*..."
}

end_task() {
    printf "${CLEAR_LAST_MSG}%s\n" "${GREEN}✓${NO_COLOR} $*... [DONE]"
}

fail_task() {
    printf "${CLEAR_LAST_MSG}%s\n" "${RED}✘ $*... [FAILED]${NO_COLOR}" >&2
}

check_requirements() {
    start_task requirements
    for i in "curl" "git" "sudo"; do
        if ! has "$i"; then
            echo "$i is required to start the installation! For help: $DOCS_URL"
            fail_task requirements
            return 1
        fi
    done
    success requirements
    end_task requirements
}

install_docker() {
    start_task docker
    if has docker; then
        success docker
    elif curl -fsSL https://get.docker.com | sh; then
        success docker
    else
        fail_task docker
    fi
    end_task docker
}

set_docker_perms() {
    start_task docker-permissions

    if ! has docker; then
        error "Docker not found, could not set permissions."
        fail_task docker-permissions
    elif ! id -nG "$USER" | grep -qw "docker"; then
        info "sudo groupadd docker && sudo usermod -aG docker $USER"
        sudo groupadd docker || true
        sudo usermod -aG docker "$USER" || true
        warn "Log out and back in for the docker group membership to take effect."
        success docker-permissions
    fi

    end_task docker-permissions
}

install_devbox() {
    start_task devbox
    if has devbox; then
        success devbox
    elif curl -fsSL https://get.jetify.com/devbox | bash; then
        success devbox
    else
        fail_task devbox
    fi
    end_task devbox
}

intro_msg() {
    title "${TITLE}"
    printf "\n"
    plain "${DESCRIPTION}"
    printf "\n"
}

next_steps_msg() {
    printf "\n"
    header "What's next?"
    plain "  1. ${BOLD}Enter the development environment: devbox shell, then task fetch and task box${NO_COLOR}"
    plain "     ${GREY}Docs: ${UNDERLINE}${BLUE}${DOCS_URL}${NO_COLOR}"
    plain "  2. ${BOLD}Get help and give feedback${NO_COLOR}"
    plain "     ${GREY}Our community: ${UNDERLINE}${BLUE}${COMMUNITY_URL}${NO_COLOR}"
    printf "\n"
}

main() {
    parse_flags "$@"
    if [ "$SILENT" -eq 0 ]; then
        intro_msg
        delay 3
    fi
    check_requirements
    install_docker
    set_docker_perms
    install_devbox
    if [ "$SILENT" -eq 0 ]; then
        next_steps_msg
    fi
}

main "$@"
