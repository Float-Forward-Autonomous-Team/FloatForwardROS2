# GPU image for fine-tuning the perception models (eWaSR and YOLO) on a PC with an NVIDIA card.
# Separate from the team image: building or running it changes nothing in the team's setup.
#
# Build once, from the repo folder (PowerShell):
#   docker build -t ff-train models/training
# Run a fine-tuning (the repo folder is mounted at /work, the scripts run from /work/models):
#   docker run --rm -it --gpus all --shm-size 4g -v "${PWD}:/work" ff-train python finetune_ewasr.py --lars /work/datasets/LaRS
#
# Needs an NVIDIA driver that supports CUDA 12.8 or newer: "CUDA Version" at the top right of nvidia-smi.
FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime

# git for the eWaSR code; libGL and glib for OpenCV
RUN apt-get update && apt-get install -y --no-install-recommends git libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# ultralytics: the version in the team container; albumentations 1.x: the API of eWaSR's own augmentation
RUN pip install --no-cache-dir "ultralytics==8.4.180" "albumentations<2" timm onnx onnxruntime

# the eWaSR code at the commit our ONNX export used, and the released weights the fine-tuning starts from
RUN git clone https://github.com/tersekmatija/eWaSR.git /opt/eWaSR \
 && git -C /opt/eWaSR checkout d1ad199522490cddcd764ef3249da1e74077c2ac
ADD https://github.com/tersekmatija/eWaSR/releases/download/0.1.0/ewasr_resnet18.pth /opt/eWaSR/ewasr_resnet18.pth

ENV EWASR_CODE=/opt/eWaSR EWASR_WEIGHTS=/opt/eWaSR/ewasr_resnet18.pth
WORKDIR /work/models
