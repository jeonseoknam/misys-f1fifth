# syntax=docker/dockerfile:1
#
# Rebuild of container b520034a42ad (image misys:3D_xvn) for NVIDIA AGX Orin.
#
# Source environment (detected from the running container):
#   * Userspace CUDA 12.2 + TensorRT 8.6  -> JetPack 6.0 base (L4T R36.3)
#   * Ubuntu 22.04, Python 3.10, aarch64
#   * ROS 2 Humble (/opt/ros/humble)
#   * PyTorch 2.10 / torchvision 0.25 -> installed from a Jetson wheel index
#   * Host L4T is R36.4.4 (JetPack 6.2); a JP6.0 container runs fine on it.
#
# The base image already ships CUDA 12.2, cuDNN and TensorRT for JetPack 6.0,
# so we do NOT reinstall those via apt/pip.
FROM nvcr.io/nvidia/l4t-jetpack:r36.3.0

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# 1) Base tooling, locale, ROS 2 + universe repositories
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl wget gnupg2 lsb-release ca-certificates \
        software-properties-common sudo git build-essential cmake \
    && locale-gen en_US en_US.UTF-8 && update-locale LANG=en_US.UTF-8 \
    && add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2) Install every apt package captured from the original container.
#    One-by-one so a single host-only/x86-only package (it won't resolve on
#    this base) does not abort the build. Skips -> /tmp/apt_skipped.log
# ---------------------------------------------------------------------------
COPY packages.apt /tmp/packages.apt
RUN apt-get update \
    && while IFS= read -r pkg; do \
         pkg="$(echo "$pkg" | tr -d '\r' | xargs)"; \
         [ -z "$pkg" ] && continue; \
         apt-get install -y --no-install-recommends "$pkg" \
           || echo "$pkg" >> /tmp/apt_skipped.log; \
       done < /tmp/packages.apt \
    && rm -rf /var/lib/apt/lists/* \
    && echo "===== apt packages skipped =====" && (cat /tmp/apt_skipped.log 2>/dev/null || echo none)

# ---------------------------------------------------------------------------
# 3) Python deps.
#    3a) Jetson/base-provided wheels are EXCLUDED from PyPI installs
#        (torch, torchvision, tensorrt, uff, graphsurgeon, onnx-graphsurgeon)
#        -- tensorrt/uff/graphsurgeon already come from the JetPack base image.
#    3b) Everything else is installed tolerantly. Skips -> /tmp/pip_skipped.log
# ---------------------------------------------------------------------------
COPY requirements.pip /tmp/requirements.pip
RUN python3 -m pip install --upgrade pip \
    && grep -ivE '^(torch|torchvision|torchaudio|tensorrt|tensorrt-|uff|graphsurgeon|onnx-graphsurgeon|nvidia-)' \
         /tmp/requirements.pip \
       | grep -vE ' @ |^-e |file://|/home/|/opt/|^#|^[[:space:]]*$' > /tmp/req.clean || true; \
    while IFS= read -r p; do \
      p="$(echo "$p" | tr -d '\r' | xargs)"; \
      [ -z "$p" ] && continue; \
      python3 -m pip install "$p" || echo "$p" >> /tmp/pip_skipped.log; \
    done < /tmp/req.clean; \
    echo "===== pip packages skipped ====="; (cat /tmp/pip_skipped.log 2>/dev/null || echo none)

# ---------------------------------------------------------------------------
# 3c) PyTorch / torchvision for Jetson (CUDA 12.2 / JetPack 6).
#     PyPI has no aarch64+CUDA wheels; use the Jetson wheel index.
#     Adjust the version/URL if the index no longer hosts exactly 2.10.0.
# ---------------------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
        --index-url https://pypi.jetson-ai-lab.dev/jp6/cu122 \
        torch==2.10.0 torchvision==0.25.0 \
    || echo "WARN: jetson torch install failed - install manually, see README" >> /tmp/pip_skipped.log

# ---------------------------------------------------------------------------
# 4) Recreate the non-root user the original ran as (misys, uid 1000)
# ---------------------------------------------------------------------------
RUN useradd -m -s /bin/bash -u 1000 misys 2>/dev/null || true \
    && usermod -aG sudo,video,audio,dialout,plugdev,render,i2c misys 2>/dev/null || true \
    && echo "misys ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/misys \
    && chmod 0440 /etc/sudoers.d/misys

# ---------------------------------------------------------------------------
# 5) Runtime environment (matches the original image config)
# ---------------------------------------------------------------------------
ENV CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64: \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    HOME=/home/misys

RUN echo 'source /opt/ros/humble/setup.bash 2>/dev/null' >> /home/misys/.bashrc \
    && echo '[ -f $HOME/forza_ws/install/setup.bash ] && source $HOME/forza_ws/install/setup.bash' >> /home/misys/.bashrc \
    && echo '[ -f $HOME/xvn_ws/install/setup.bash ]   && source $HOME/xvn_ws/install/setup.bash'   >> /home/misys/.bashrc \
    && chown misys:misys /home/misys/.bashrc

# ---------------------------------------------------------------------------
# 6) (OPTIONAL) g2o built from source (the original had ~/g2o).
# ---------------------------------------------------------------------------
# RUN git clone https://github.com/RainerKuemmerle/g2o.git /opt/g2o \
#  && cmake -S /opt/g2o -B /opt/g2o/build -DCMAKE_BUILD_TYPE=Release \
#  && cmake --build /opt/g2o/build -j"$(nproc)" && cmake --install /opt/g2o/build

# ---------------------------------------------------------------------------
# 7) (OPTIONAL) Your application code from GitHub (see ../code_export/README).
#     After pushing, uncomment and set the repo URL, then rebuild.
# ---------------------------------------------------------------------------
ARG REPO_URL=https://github.com/jeonseoknam/misys-f1fifth.git
ARG REPO_BRANCH=main
RUN git clone --branch "$REPO_BRANCH" "$REPO_URL" /home/misys/forza_ws \
 && bash -lc 'source /opt/ros/humble/setup.bash && cd /home/misys/forza_ws && colcon build --symlink-install' \
 && chown -R misys:misys /home/misys/forza_ws

USER misys
WORKDIR /home/misys
CMD ["bash"]
