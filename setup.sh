#!/usr/bin/env bash
set -e

echo "==> Installing system dependencies..."

# Ubuntu / Debian setup
if command -v apt-get &> /dev/null; then
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        cmake \
        pkg-config \
        libeigen3-dev \
        libopencv-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libyaml-cpp-dev
# macOS (Homebrew) setup
elif command -v brew &> /dev/null; then
    brew update
    brew install cmake pkg-config eigen opencv glfw yaml-cpp
fi

echo "==> Done!"
