# Start from the official Ubuntu 24.04 LTS image.
FROM ubuntu:24.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential gdb cmake software-properties-common clang clang-format \
    clang-tidy make ninja-build git curl python3-full python3-pip python3-dev unzip patchelf doxygen \
    libgmp-dev libmpfr-dev libmpc-dev gcc-14 g++-14 libegl-dev libegl1-mesa-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxi-dev \
    libudev-dev \
    libfreetype-dev \
    libflac-dev \
    libvorbis-dev \
    libgl1-mesa-dev libx11-xcb-dev libfontenc-dev libice-dev libsm-dev libxaw7-dev libxcomposite-dev libxdamage-dev libxinerama-dev libxkbfile-dev libxmu-dev libxmuu-dev libxpm-dev libxres-dev libxss-dev libxt-dev libxtst-dev libxv-dev libxxf86vm-dev libxcb-glx0-dev libxcb-render0-dev libxcb-render-util0-dev libxcb-xkb-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-dri3-dev uuid-dev libxcb-cursor-dev libxcb-dri2-0-dev libxcb-dri3-dev libxcb-present-dev libxcb-composite0-dev libxcb-ewmh-dev libxcb-res0-dev

# Install system dependencies
RUN apt-get install -y pipx && \
    pipx ensurepath

RUN rm -rf /var/lib/apt/lists/*

# Add the conan profile
ADD conan-profile /root/default-conan-profile

# Set environment for root user
ENV PATH="/root/.local/bin:${PATH}"

# Install and configure Conan
RUN pipx install "conan>=2.0"
RUN conan profile detect --force && \
    mv /root/default-conan-profile /root/.conan2/profiles/default && \
    conan remote update conancenter --url="https://center2.conan.io"

# Keep the container running
CMD ["sleep", "infinity"]`