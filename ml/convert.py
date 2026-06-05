import json, os, shutil
from pathlib import Path
from sklearn.model_selection import train_test_split

def convert(ann_file, img_dir, out_dir):
    with open(ann_file) as f:
        coco = json.load(f)

    for split in ['train', 'val']:
        Path(f"{out_dir}/images/{split}").mkdir(parents=True, exist_ok=True)
        Path(f"{out_dir}/labels/{split}").mkdir(parents=True, exist_ok=True)

    # Build image_id → full path map using COCO file_name field directly
    print("Indexing images...")
    img_path_map = {}
    for root, dirs, files in os.walk(img_dir):
        for f in files:
            if f.lower().endswith('.jpg') or f.lower().endswith('.jpeg'):
                # Use relative path from img_dir as key to match COCO file_name
                rel = os.path.relpath(os.path.join(root, f), img_dir)
                img_path_map[rel] = os.path.join(root, f)
                # Also index by basename with batch prefix
                batch = os.path.basename(root)
                img_path_map[f"{batch}/{f}"] = os.path.join(root, f)
                img_path_map[f] = os.path.join(root, f)  # fallback

    print(f"Found {len([v for v in img_path_map.values() if os.path.exists(v)])} images on disk")

    img_map = {i['id']: i for i in coco['images']}
    ann_map = {}
    for ann in coco['annotations']:
        ann_map.setdefault(ann['image_id'], []).append(ann)

    img_ids = list(img_map.keys())
    train_ids, val_ids = train_test_split(img_ids, test_size=0.2, random_state=42)

    for split, ids in [('train', train_ids), ('val', val_ids)]:
        copied = 0
        for img_id in ids:
            img  = img_map[img_id]
            w, h = img['width'], img['height']
            
            # Try multiple key formats to find the image
            file_name = img['file_name']
            basename  = os.path.basename(file_name)
            src = (img_path_map.get(file_name) or
                   img_path_map.get(basename) or
                   img_path_map.get(file_name.replace('data/', '')))

            if src and os.path.exists(src):
                # Use image_id as filename to guarantee uniqueness
                unique_name = f"{img_id:06d}.jpg"
                dst = f"{out_dir}/images/{split}/{unique_name}"
                shutil.copy(src, dst)
                copied += 1

                anns = ann_map.get(img_id, [])
                label_path = f"{out_dir}/labels/{split}/{img_id:06d}.txt"
                with open(label_path, 'w') as lf:
                    for ann in anns:
                        x, y, bw, bh = ann['bbox']
                        cx = (x + bw/2) / w
                        cy = (y + bh/2) / h
                        nw = bw / w
                        nh = bh / h
                        lf.write(f"0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")

        print(f"{split}: {copied}/{len(ids)} images copied")

    print("Done — dataset ready")

if __name__ == '__main__':
    convert(
        ann_file='TACO/data/annotations.json',
        img_dir='TACO/data',
        out_dir='TACO'
    )
