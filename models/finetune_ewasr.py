"""Fine-tunes eWaSR on LaRS, exports the result to ONNX and compares it with the current model.

Runs in the GPU training container (models/training/Dockerfile). From the repo folder in PowerShell:
    docker build -t ff-train models/training
    docker run --rm -it --gpus all --shm-size 4g -v "${PWD}:/work" ff-train python finetune_ewasr.py --lars /work/datasets/LaRS

What it does:
 1. Splits the LaRS training frames by video: four fifths to train on, one fifth held out for testing.
    The split is saved once in datasets/LaRS_split/ and reused by every later run and by finetune_yolo.py.
 2. Resizes the training frames to the model's 512 x 384 once (datasets/LaRS_cache_512x384/).
 3. Fine-tunes the released eWaSR weights with the authors' loss, optimiser and augmentation, plus a
    brightness augmentation, and shows still-water, heavy-reflection and extra-bright frames 3 times as often.
    Settings are fixed in advance and the last epoch is kept: nothing is chosen by looking at test frames.
 4. Saves the weights (datasets/runs/<name>/) and exports <name>.onnx here, with the same input and output
    as ewasr_resnet18.onnx, so the node can load it with -p model_path:=...
 5. Scores the current and the new model on the held-out frames and on the LaRS validation frames with
    evaluate_lars.py, and lists every condition where the new model is more than 1 point worse.

Options:
    --epochs 30          passes over the training frames
    --batch 8            frames per step; use 4 if the GPU runs out of memory
    --lr 1e-6            the authors' base learning rate (decoder 10x, decoder biases 20x)
    --focus-weight 3     how much more often Monaco-like frames are shown
    --focus TAG ...      which scene conditions count as Monaco-like (default: waves=still
                         reflections=heavy extra_bright); names as in the evaluate_lars.py tables
    --name ewasr_resnet18_lars   name of the new model files; give a new name to keep an earlier run's model
    --old-model FILE     the model to compare with (default ewasr_resnet18.onnx)
    --limit N / --eval-limit N   use only N frames, for a quick test of the whole run
    --device cpu         force the CPU (slow; for testing)
    --no-eval            skip the comparison at the end
"""

import argparse
import json
import os
import random
import subprocess
import sys
import time
import types

import cv2
import numpy as np
import torch

import lars_tools as LT

EWASR_CODE = os.environ.get("EWASR_CODE", "/opt/eWaSR")
EWASR_WEIGHTS = os.environ.get("EWASR_WEIGHTS", "/opt/eWaSR/ewasr_resnet18.pth")
SIZE = (512, 384)  # width, height: the model's input, as in the node
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
SEPARATION_WEIGHT = 0.01  # the authors' weight for the water-obstacle separation loss


def import_ewasr(code_dir):
    """Imports the eWaSR model and losses from the authors' code, with the same small patches as
    export_ewasr.py: no training framework needed, no ImageNet download, resize without anti-aliasing
    (as the released weights were trained and as ONNX exports it)."""
    sys.path.insert(0, code_dir)
    try:
        import pytorch_lightning  # noqa: F401
    except ImportError:
        pl = types.ModuleType("pytorch_lightning")
        pl.Callback = object
        pl.LightningModule = object
        sys.modules["pytorch_lightning"] = pl
    try:
        import timm.models.layers.helpers  # noqa: F401
    except ImportError:
        import timm.layers
        helpers = types.ModuleType("timm.models.layers.helpers")
        helpers.to_2tuple = timm.layers.to_2tuple
        sys.modules["timm.models.layers.helpers"] = helpers
    import torchvision
    import torchvision.transforms.functional as TF
    original_resize = TF.resize

    def resize(img, size, interpolation=TF.InterpolationMode.BILINEAR, max_size=None, antialias=None):
        return original_resize(img, size, interpolation, max_size, antialias=False)

    TF.resize = resize
    import wasr.models as M
    from wasr.loss import focal_loss, water_obstacle_separation_loss
    M.resnet18 = lambda pretrained=False, **kw: torchvision.models.resnet18(weights=None)
    return M, focal_loss, water_obstacle_separation_loss


def load_model(M, weights):
    model = M.get_model("ewasr_resnet18", num_classes=3, pretrained=False)
    state = torch.load(weights, map_location="cpu")
    if isinstance(state, dict) and "model" in state:
        state = state["model"]
    model.load_state_dict(state, strict=True)  # all 373 entries must match, or training stops here
    return model


def augmentation():
    """The eWaSR authors' training augmentation, plus brighter and flatter images for sunny scenes."""
    import albumentations as A
    color = A.Compose([A.ColorJitter(p=0.7, hue=0.05), A.RandomGamma(p=1, gamma_limit=(70, 120))], p=0.5)
    noise = A.Compose([A.GaussNoise(p=0.5), A.ISONoise(p=0.5)], p=0.3)
    bright = A.RandomBrightnessContrast(brightness_limit=(0.0, 0.3), contrast_limit=(-0.3, 0.0), p=0.25)
    return A.Compose([
        A.HorizontalFlip(),
        A.ShiftScaleRotate(scale_limit=[0, 0.3], rotate_limit=15, border_mode=0, p=0.7),
        color, bright, noise,
    ])


class Frames(torch.utils.data.Dataset):
    """Training frames from the cache: image (normalised like the node) and one-hot label mask.
    Ignore pixels (255) become all-zero rows, so they add nothing to the loss, as in eWaSR's own loader."""

    def __init__(self, images, masks, augment):
        self.images, self.masks, self.augment = images, masks, augment

    def __len__(self):
        return len(self.images)

    def __getitem__(self, i):
        img = cv2.imdecode(np.frombuffer(self.images[i], np.uint8), cv2.IMREAD_COLOR)[..., ::-1]
        lbl = cv2.imdecode(np.frombuffer(self.masks[i], np.uint8), cv2.IMREAD_UNCHANGED)
        onehot = np.stack([lbl == 0, lbl == 1, lbl == 2], axis=-1).astype(np.float32)
        img = np.ascontiguousarray(img)
        if self.augment is not None:
            out = self.augment(image=img, masks=[onehot])
            img, onehot = out["image"], out["masks"][0]
        x = ((img.astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)
        return torch.from_numpy(np.ascontiguousarray(x)), torch.from_numpy(np.ascontiguousarray(onehot.transpose(2, 0, 1)))


def seed_worker(_):
    seed = torch.initial_seed() % 2 ** 32
    np.random.seed(seed)
    random.seed(seed)


def build_cache(names, frames, cache_dir):
    """Resizes each training frame and its mask to 512 x 384 once; returns them as encoded bytes."""
    img_dir, mask_dir = os.path.join(cache_dir, "images"), os.path.join(cache_dir, "masks")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(mask_dir, exist_ok=True)
    images, masks, made, start = [], [], 0, time.perf_counter()
    for i, name in enumerate(names, 1):
        ip, mp = os.path.join(img_dir, name + ".jpg"), os.path.join(mask_dir, name + ".png")
        if not (os.path.exists(ip) and os.path.exists(mp)):
            img = cv2.resize(cv2.imread(frames["image_of"][name]), SIZE, interpolation=cv2.INTER_AREA)
            lbl = LT.read_label_mask(os.path.join(frames["masks_dir"], name + ".png"))
            lbl = cv2.resize(lbl.astype(np.uint8), SIZE, interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(ip, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            cv2.imwrite(mp, lbl)
            made += 1
        with open(ip, "rb") as f:
            images.append(f.read())
        with open(mp, "rb") as f:
            masks.append(f.read())
        if made and (i % 200 == 0 or i == len(names)):
            print(f"  resized {i}/{len(names)} frames ({time.perf_counter() - start:.0f} s)", flush=True)
    return images, masks


def export_onnx(model, path):
    """Same export as export_ewasr.py: input 'image' 1x3x384x512, output 'logits' 1x3x384x512."""
    class OnlyImage(torch.nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, image):
            no_imu = torch.zeros(image.shape[0], image.shape[2], image.shape[3])
            return self.m({"image": image, "imu_mask": no_imu})["out"]

    wrapped = OnlyImage(model.cpu().eval()).eval()
    torch.onnx.export(wrapped, (torch.randn(1, 3, SIZE[1], SIZE[0]),), path, opset_version=17,
                      input_names=["image"], output_names=["logits"], dynamo=False)
    # check the exported file gives the same classes as the PyTorch model
    import onnxruntime as ort
    x = torch.randn(1, 3, SIZE[1], SIZE[0])
    with torch.no_grad():
        ref = wrapped(x).numpy()
    got = ort.InferenceSession(path, providers=["CPUExecutionProvider"]).run(None, {"image": x.numpy()})[0]
    same = float((ref.argmax(1) == got.argmax(1)).mean()) * 100
    print(f"  exported {path}: {same:.2f} % of pixels get the same class as in PyTorch")


def evaluate(args, split, frames_file, label, report):
    """Runs evaluate_lars.py on the current and the new model and returns its output."""
    cmd = [sys.executable, "evaluate_lars.py", "--lars", args.lars, "--split", split,
           "--model", args.old_model, "--model", args.name + ".onnx"]
    if frames_file:
        cmd += ["--frames-file", frames_file]
    if args.eval_limit:
        cmd += ["--limit", str(args.eval_limit)]
    print(f"\n=== {label} ===", flush=True)
    out = subprocess.run(cmd, capture_output=True, text=True)
    text = out.stdout + (out.stderr if out.returncode else "")
    print(text, flush=True)
    report.write(f"=== {label} ===\n{text}\n")
    return text


def worse_rows(text):
    """Rows of the difference table that evaluate_lars.py marked as more than 1 point worse."""
    rows, in_diff = [], False
    for line in text.splitlines():
        if line.startswith("difference:"):
            in_diff = True
        elif in_diff and "worse:" in line:
            rows.append(line.strip())
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lars", required=True, help="folder you unzipped LaRS into")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--lr", type=float, default=1e-6)
    parser.add_argument("--focus-weight", type=float, default=3.0)
    parser.add_argument("--focus", nargs="+", default=list(LT.FOCUS_TAGS))
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--name", default="ewasr_resnet18_lars")
    parser.add_argument("--old-model", default="ewasr_resnet18.onnx", help="current model, for the comparison")
    parser.add_argument("--weights", default=EWASR_WEIGHTS, help="released eWaSR weights to start from")
    parser.add_argument("--code", default=EWASR_CODE, help="folder with the eWaSR source code")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--eval-limit", type=int, default=0)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-eval", action="store_true")
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device(args.device)
    gpu = torch.cuda.get_device_name(0) if device.type == "cuda" else "none, running on the CPU"
    print(f"PyTorch {torch.__version__}, GPU: {gpu}")
    data_root = os.path.dirname(os.path.abspath(args.lars.rstrip("/\\")))
    run_dir = os.path.join(data_root, "runs", args.name)
    os.makedirs(run_dir, exist_ok=True)

    # 1. split by video, and the frames to train on
    train, holdout, split_dir = LT.make_or_load_split(args.lars)
    frames = LT.list_frames(args.lars, "train")
    available = set(frames["names"])
    train = [n for n in train if n in available]
    if args.limit:
        train = train[:args.limit]
    conditions = LT.conditions_for(frames["split_dir"], frames["names"])
    focus = [any(t in conditions.get(n, ()) for t in args.focus) for n in train]
    print(f"Training on {len(train)} frames, {sum(focus)} of them Monaco-like "
          f"({', '.join(args.focus)}), shown {args.focus_weight:g} times as often; "
          f"{len(holdout)} frames held out for testing")

    # 2. the frames at the model's size, read once into memory
    images, masks = build_cache(train, frames, os.path.join(data_root, "LaRS_cache_512x384"))

    # 3. model, optimiser and data, as in the eWaSR authors' training
    M, focal_loss, separation_loss = import_ewasr(args.code)
    model = load_model(M, args.weights).to(device).train()
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True
    encoder, dec_w, dec_b = [], [], []
    for pname, p in model.named_parameters():
        (encoder if pname.startswith("backbone") else dec_w if "weight" in pname else dec_b).append(p)
    optimiser = torch.optim.RMSprop([{"params": encoder, "lr": args.lr}, {"params": dec_w, "lr": args.lr * 10},
                                     {"params": dec_b, "lr": args.lr * 20}],
                                    momentum=0.9, alpha=0.9, weight_decay=1e-6)
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimiser, lambda e: (1 - e / args.epochs) ** 0.9)
    weights = torch.tensor([args.focus_weight if f else 1.0 for f in focus], dtype=torch.double)
    sampler = torch.utils.data.WeightedRandomSampler(weights, num_samples=len(train), replacement=True,
                                                     generator=torch.Generator().manual_seed(args.seed))
    loader = torch.utils.data.DataLoader(Frames(images, masks, augmentation()), batch_size=args.batch,
                                         sampler=sampler, num_workers=args.workers, drop_last=True,
                                         pin_memory=device.type == "cuda", worker_init_fn=seed_worker,
                                         persistent_workers=args.workers > 0)
    if len(loader) == 0:
        raise SystemExit(f"Fewer training frames ({len(train)}) than one batch ({args.batch}).")

    # 4. train
    history, start = [], time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        t0, sums, steps = time.perf_counter(), np.zeros(3), 0
        for x, y in loader:
            x, y = x.to(device, non_blocking=True), y.to(device, non_blocking=True)
            out = model({"image": x, "imu_mask": torch.zeros(x.shape[0], x.shape[2], x.shape[3], device=device)})
            fl = focal_loss(out["out"], y, target_scale="labels")
            sl = SEPARATION_WEIGHT * separation_loss(out["aux"], y).to(device)
            loss = fl + sl
            optimiser.zero_grad(set_to_none=True)
            loss.backward()
            optimiser.step()
            sums += [loss.item(), fl.item(), sl.item()]
            steps += 1
        scheduler.step()
        mean = sums / steps
        history.append({"epoch": epoch, "loss": mean[0], "focal": mean[1], "separation": mean[2],
                        "seconds": time.perf_counter() - t0})
        left = (time.perf_counter() - start) / epoch * (args.epochs - epoch) / 60
        print(f"  epoch {epoch}/{args.epochs}: loss {mean[0]:.4f} (focal {mean[1]:.4f}, separation "
              f"{mean[2]:.4f}), {time.perf_counter() - t0:.0f} s, about {left:.0f} min left", flush=True)

    # 5. save and export
    weights_path = os.path.join(run_dir, args.name + ".pth")
    torch.save(model.state_dict(), weights_path)
    with open(os.path.join(run_dir, "run.json"), "w") as f:
        json.dump({"args": vars(args), "train_frames": len(train), "focus_frames": int(sum(focus)),
                   "holdout_frames": len(holdout), "split_dir": split_dir, "history": history}, f, indent=1)
    print(f"\nSaved weights: {weights_path}")
    export_onnx(model, args.name + ".onnx")

    # 6. compare with the current model on frames neither was trained on
    if args.no_eval:
        return
    if not os.path.exists(args.old_model):
        print(f"{args.old_model} not found here: skipping the comparison.")
        return
    with open(os.path.join(run_dir, "comparison.txt"), "w") as report:
        held = evaluate(args, "train", os.path.join(split_dir, "holdout.txt"), "held-out training frames", report)
        val = evaluate(args, "val", None, "LaRS validation frames", report)
    worse = [("held-out", r) for r in worse_rows(held)] + [("validation", r) for r in worse_rows(val)]
    print("=== switch rule: keep the new model only if no condition is more than 1 point worse ===")
    if not worse:
        print("No condition is more than 1 point worse on the held-out or the validation frames.")
    for where, row in worse:
        print(f"  worse on {where} frames: {row}")
    print(f"Full comparison saved in {os.path.join(run_dir, 'comparison.txt')}")


if __name__ == "__main__":
    main()
