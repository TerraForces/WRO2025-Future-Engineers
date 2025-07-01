from ultralytics import YOLO
from ultralytics.utils import TQDM
import json
import shutil
from collections import defaultdict
from pathlib import Path
import numpy as np

def convert_coco(labels_dir, save_dir):
    save_dir = Path(save_dir)
    for directory in save_dir / "labels", save_dir / "images":
        directory.mkdir(parents=True, exist_ok=True)
    for json_file in sorted((Path(labels_dir)  / "annotations").resolve().glob("*.json")):
        lname = json_file.stem.replace("instances_", "")
        (save_dir / "images" / lname).mkdir(parents=True, exist_ok=True)
        for src_jpg_file in sorted((Path(labels_dir) / lname).resolve().glob("*.jpg")):
            dst_jpg_file = (save_dir / "images" / lname / src_jpg_file.stem).with_suffix(".jpg")
            shutil.copyfile(src = src_jpg_file, dst = dst_jpg_file)
        fn = Path(save_dir) / "labels" / lname
        fn.mkdir(parents=True, exist_ok=True)
        with open(json_file, encoding="utf-8") as f:
            data = json.load(f)
        images = {f"{x['id']:d}": x for x in data["images"]}
        annotations = defaultdict(list)
        for ann in data["annotations"]:
            annotations[ann["image_id"]].append(ann)
        for img_id, anns in TQDM(annotations.items(), desc=f"Annotations {json_file}"):
            img = images[f"{img_id:d}"]
            h, w = img["height"], img["width"]
            f = Path(img["file_name"]).name
            bboxes = []
            for ann in anns:
                if ann.get("iscrowd", False):
                    continue
                box = np.array(ann["bbox"], dtype=np.float64)
                box[:2] += box[2:] / 2
                box[[0, 2]] /= w
                box[[1, 3]] /= h
                if box[2] <= 0 or box[3] <= 0:
                    continue
                cls = ann["category_id"]
                box = [cls] + box.tolist()
                if box not in bboxes:
                    bboxes.append(box)
            with open((fn / f).with_suffix(".txt"), "a", encoding="utf-8") as file:
                for box in bboxes:
                    for part in box:
                        file.write(str(part) + " ")
                    file.write("\n")
    
# convert COCO to YOLO dataset
convert_coco(labels_dir = "/home/terra/datasets/WRO 2025/COCO/", save_dir = "/home/terra/datasets/WRO 2025/YOLO/")

# remove training result directory
if Path("runs").is_dir():
	shutil.rmtree("runs")

# load pretrained yolo11s model
model = YOLO("yolo11s.pt")

# train model with YOLO dataset
train_results = model.train(
    data = "/media/terra/Volume/08 WRO/WRO-Future-Engineers-2025/WRO RPI5/coco_wro.yaml",
    epochs = 600,
    workers = 8,
    batch = 0.9,
    cache = True,
    save_period = 60,
    device = 0,
    warmup_epochs = 10.0
)

# export the model to ONNX format for hailomz compiling
path = model.export(
	format = "onnx",
	opset = 11
)