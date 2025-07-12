FROM ubuntu:22.04

# Install necessary packages
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && \
    apt-get install -y wget xz-utils bzip2 make unzip cppcheck openocd

# --- Multi-arch Toolchain Download ---
ARG TARGETARCH
RUN mkdir -p /home/ubuntu/dev/tools/arm-none-eabi-gcc && \
    cd /home/ubuntu/dev/tools/arm-none-eabi-gcc && \
    if [ "$TARGETARCH" = "amd64" ]; then \
      TOOLCHAIN_URL="https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz"; \
      TOOLCHAIN_DIR="arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi"; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
      TOOLCHAIN_URL="https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi.tar.xz"; \
      TOOLCHAIN_DIR="arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi"; \
    else \
      echo "ERROR: Unknown arch $TARGETARCH"; exit 1; \
    fi && \
    wget -O toolchain.tar.xz $TOOLCHAIN_URL && \
    tar -xf toolchain.tar.xz && rm toolchain.tar.xz

# Copy your local include files
# This assumes you have `docker_include/` in your build context,
# and that the directory name matches $TOOLCHAIN_DIR in the extraction above.
ARG TARGETARCH
# Instead, do two COPYs, one for each arch:
COPY docker_include /home/ubuntu/dev/tools/arm-none-eabi-gcc/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/include
COPY docker_include /home/ubuntu/dev/tools/arm-none-eabi-gcc/arm-gnu-toolchain-14.3.rel1-aarch64-arm-none-eabi/include

# Create a non-root user named "ubuntu"
RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1001 ubuntu
USER ubuntu
WORKDIR /home/ubuntu
