#!/bin/bash
# Jetson Orin Nano (JetPack 6.1) Auto-Installer for UFLD-v2
# Run with: bash install_jetson.sh

echo "STARTING JETSON AI ENVIRONMENT SETUP"

echo "Installing System Dependencies"
sudo apt-get update
sudo apt-get install -y python3-pip libopenblas-dev libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev

echo "Mapping CUDA 12.6 Paths"
# Temporarily export
export PATH=/usr/local/cuda-12.6/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}

# Permanently add to bashrc if not already there
if ! grep -q "cuda-12.6" ~/.bashrc; then
    echo 'export PATH=/usr/local/cuda-12.6/bin:${PATH}' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
    echo "CUDA paths injected into ~/.bashrc"
fi

echo "Installing cuSPARSELt for Pytorch 2.5"
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb
sudo dpkg -i /tmp/cuda-keyring.deb
sudo apt-get update
sudo apt-get install -y libcusparselt0 libcusparselt-dev

echo "Installing PyTorch 2.5, Vision, and Python Dependencies"
pip3 cache purge

# Pull official JetPack 6.1 PyTorch
wget https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl -O /tmp/torch-2.5.0-cp310-cp310-linux_aarch64.whl
pip3 install /tmp/torch-2.5.0-cp310-cp310-linux_aarch64.whl

# Pull matching TorchVision
pip3 install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl --no-deps

# Install UFLD requirements
pip3 install --user scipy tqdm tensorboard scikit-learn pathspec imagesize ujson pandas

# Force clean install of addict
pip3 uninstall addict -y
pip3 install --user --force-reinstall addict

echo "INSTALLATION COMPLETE"
echo "Please close this terminal and open a new one to apply the CUDA path changes."