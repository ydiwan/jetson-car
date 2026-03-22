# Jetson Orin Nano AI Setup (UFLD-V2)

This repository requires a highly specific hardware-accelerated environment to run the Ultra-Fast-Lane-Detection models alongside ROS 2 Humble. Standard `pip install` commands will pull CPU-only versions and break the perception pipeline.

This setup is strictly for **JetPack 6.1 (CUDA 12.6 / Python 3.10)**.

## Installation

**Clone the required sidecar repository**
Ensure the UFLD-V2 repository is cloned into your workspace so the ROS 2 node can access its utilities.
```bash
cd ~/youssef/rework
git clone [https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)
```

## Final Check

After installation, verify that the correct versions of PyTorch and torchvision are installed with CUDA support:
```bash
python3 -c "import torch; import torchvision; from addict import Dict; print('GPU ACTIVE:', torch.cuda.is_available()); print('Torch:', torch.__version__); print('Vision:', torchvision.__version__)"
```