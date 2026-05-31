# Rebuildable Docker environment — AGX Orin (JetPack 6 / ROS 2 Humble)

Reproduces the environment of container `b520034a42ad` (image `misys:3D_xvn`) as a
**buildable Dockerfile** so it can be recreated on any other NVIDIA AGX Orin.

## What the original container was (detected, not guessed)
| | |
|---|---|
| Base | JetPack 6.0 userspace → `nvcr.io/nvidia/l4t-jetpack:r36.3.0` (**CUDA 12.2**, cuDNN, **TensorRT 8.6**) |
| OS | Ubuntu 22.04, Python 3.10, aarch64 |
| Host L4T | R36.4.4 (JetPack 6.2) — a JP6.0 container runs fine on it |
| ROS | ROS 2 **Humble** (`/opt/ros/humble`) |
| DL | PyTorch 2.10 + torchvision 0.25 (Jetson wheels), TensorRT 8.6 (from base) |
| apt | 226 manually-installed packages → `packages.apt` |
| pip | 155 packages → `requirements.pip` |
| User | `misys` (uid 1000), workdir `/home/misys` |
| Runtime | `--runtime nvidia --privileged --net host` + X11/USB/serial/audio/DRI mounts |

### Not reproduced (intentionally)
- `~/.cache`, `~/.ros`, `~/.vscode-server` — caches/IDE state
- `shared_dir` (61 GB) — a **host bind-mount**, not part of the image (re-mounted at run time)
- `XVN_dataset` (1.3 GB) and rosbags — data, keep these outside the image

## Files
- `Dockerfile` — base + apt + pip + Jetson torch + user + ROS sourcing
- `packages.apt` / `requirements.pip` — captured manifests (installed tolerantly)
- `jetson_pip_excluded.txt` — pip names that must NOT come from PyPI (torch/tensorrt/…)
- `build.sh` / `run.sh`

## Build & run on another AGX Orin
```bash
# Prereqs: JetPack 6.x flashed, nvidia container runtime present,
#          and `docker pull nvcr.io/nvidia/l4t-jetpack:r36.3.0` works.

./build.sh misys:3d_xvn_rebuilt                 # build
docker run --rm misys:3d_xvn_rebuilt \          # review anything that didn't install
  sh -c 'cat /tmp/apt_skipped.log /tmp/pip_skipped.log 2>/dev/null'
./run.sh  misys:3d_xvn_rebuilt                  # run (GPU + GUI + devices)
```

## Caveats
- **PyTorch**: PyPI has no aarch64+CUDA wheels. The Dockerfile pulls torch from
  the Jetson index (`pypi.jetson-ai-lab.dev/jp6/cu122`). If that index no longer
  hosts exactly `2.10.0`, pick the closest JP6/cu122 build and update section 3c.
- **Version match**: target Orin should be JetPack 6.x. If its L4T differs a lot,
  change the `FROM` tag (`r36.3.0` → matching tag).
- **Tolerant installs**: apt/pip install one-by-one; review the skip logs.
- **g2o** (section 6) and **your code** (section 7) are optional/commented — see
  `../code_export/` for moving code to GitHub.
- This is a **clean rebuild**, not a byte-for-byte snapshot. For an exact copy:
  `docker commit b520034a42ad misys:snapshot && docker save misys:snapshot | gzip > snap.tgz`
