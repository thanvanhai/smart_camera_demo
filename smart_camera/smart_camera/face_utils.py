# smart_camera/smart_camera/face_utils.py
import os
import cv2
import numpy as np
import insightface
from insightface.app import FaceAnalysis

class FaceEmbedder:
    def __init__(self, det_size=(640, 640), providers=None):
        self.app = FaceAnalysis(name="buffalo_l")  # ArcFace tốt, cân bằng tốc độ/độ chính xác
        self.app.prepare(ctx_id=0 if self._has_gpu(providers) else -1, det_size=det_size, providers=providers)

    def _has_gpu(self, providers):
        return providers and any("CUDA" in p or "Tensorrt" in p for p in providers)

    def get_face_embeddings(self, bgr_image):
        # Trả về list [(bbox, embedding)]
        faces = self.app.get(bgr_image)
        out = []
        for f in faces:
            emb = f.normed_embedding  # vector 512-D, đã norm
            # bbox float -> int
            x1, y1, x2, y2 = [int(v) for v in f.bbox]
            out.append(((x1, y1, x2, y2), emb))
        return out

def cosine_sim(a, b):
    # a, b đã norm từ InsightFace → dot chính là cosine
    return float(np.dot(a, b))

def load_enrolled_embeddings(json_path):
    import json
    if not os.path.exists(json_path):
        return {}
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    # Chuyển list -> np.array
    for name in list(data.keys()):
        data[name] = [np.array(v, dtype=np.float32) for v in data[name]]
    return data

def save_enrolled_embeddings(json_path, db):
    import json
    serial = {k: [v.astype(float).tolist() for v in vals] for k, vals in db.items()}
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(serial, f, ensure_ascii=False, indent=2)

def verify(emb, enrolled_vecs, agg="max"):
    # So sánh 1 embedding với nhiều mẫu enroll → trả về điểm cao nhất
    if not enrolled_vecs:
        return -1.0
    sims = [cosine_sim(emb, v) for v in enrolled_vecs]
    return max(sims) if agg == "max" else float(np.mean(sims))
