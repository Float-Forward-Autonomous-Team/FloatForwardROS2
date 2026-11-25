# Turns the eWaSR PyTorch weights into one ONNX file that the ROS node can run.
# Run once, inside the export venv (see the perception setup notes), from this folder:
#   python export_ewasr.py
import sys
import types

sys.path.insert(0, "eWaSR")

# eWaSR imports two things it does not need just to run the model: fake them
pl = types.ModuleType("pytorch_lightning")
pl.Callback = object
pl.LightningModule = object
sys.modules["pytorch_lightning"] = pl
import timm.layers  # noqa: E402

helpers = types.ModuleType("timm.models.layers.helpers")
helpers.to_2tuple = timm.layers.to_2tuple
sys.modules["timm.models.layers.helpers"] = helpers

import torch  # noqa: E402
import torchvision  # noqa: E402
import torchvision.transforms.functional as TF  # noqa: E402

original_resize = TF.resize


def resize(img, size, interpolation=TF.InterpolationMode.BILINEAR, max_size=None, antialias=None):
    return original_resize(img, size, interpolation, max_size, antialias=False)  # ONNX cannot do antialias


TF.resize = resize

import wasr.models as M  # noqa: E402

M.resnet18 = lambda pretrained=False, **kw: torchvision.models.resnet18(weights=None)
model = M.get_model("ewasr_resnet18", num_classes=3, pretrained=False)
model.load_state_dict(torch.load("ewasr_resnet18.pth", map_location="cpu"), strict=False)
model.eval()


class OnlyImage(torch.nn.Module):
    def __init__(self, m):
        super().__init__()
        self.m = m

    def forward(self, image):
        no_imu = torch.zeros(image.shape[0], image.shape[2], image.shape[3])
        return self.m({"image": image, "imu_mask": no_imu})["out"]


torch.onnx.export(
    OnlyImage(model).eval(),
    (torch.randn(1, 3, 384, 512),),
    "ewasr_resnet18.onnx",
    opset_version=17,
    input_names=["image"],
    output_names=["logits"],
    dynamo=False,
)
print("saved ewasr_resnet18.onnx")
