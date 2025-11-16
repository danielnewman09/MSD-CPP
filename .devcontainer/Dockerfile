# Start from the official Ubuntu 24.04 LTS image.
FROM ubuntu:24.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# ============================ THE FIX: PART 1 ============================
# Define arguments for user/group IDs. Default to 1000, a common standard.
ARG USER_UID=1000
ARG USER_GID=1000
# =======================================================================

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential gdb cmake software-properties-common clang clang-format \
    clang-tidy make ninja-build git curl python3-full python3-pip unzip \
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

# ============================ THE FIX: PART 2 ============================
# Create a non-root user with the UID/GID passed in as build arguments.
# RUN groupadd --gid ${USER_GID} vscode && \
# =======================================================================

# Add the conan profile, ensuring correct ownership from the start.
# Note the use of build arguments in the --chown flag.
ADD --chown=${USER_UID}:${USER_GID} conan-profile /home/ubuntu/default-conan-profile

# Switch to the non-root user
USER ubuntu

# Set environment for the user
ENV PATH="/home/ubuntu/.local/bin:${PATH}"

# Install and configure Conan as the non-root user
RUN pipx install "conan>=2.0"
RUN conan profile detect --force && \
    mv /home/ubuntu/default-conan-profile /home/ubuntu/.conan2/profiles/default && \
    conan remote update conancenter --url="https://center2.conan.io"

# Switch back to root user.
USER root

# Keep the container running
CMD ["sleep", "infinity"]`