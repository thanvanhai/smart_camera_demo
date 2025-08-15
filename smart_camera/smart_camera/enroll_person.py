# scripts/enroll_person.py (bạn có thể đặt trong smart_camera/ hoặc scripts/)
import glob, os, cv2, numpy as np
from smart_camera.face_utils import FaceEmbedder, load_enrolled_embeddings, save_enrolled_embeddings

EMB_PATH = "data/embeddings.json"
ENROLL_DIR = "data/enroll"   # chứa ảnh: /data/enroll/Alice/*.jpg

def main(person_name, img_glob):
    fe = FaceEmbedder(providers=["CPUExecutionProvider"])
    db = load_enrolled_embeddings(EMB_PATH)
    db.setdefault(person_name, [])
    for path in glob.glob(img_glob):
        img = cv2.imread(path)
        if img is None: 
            continue
        faces = fe.get_face_embeddings(img)
        if not faces: 
            continue
        # lấy mặt lớn nhất
        faces.sort(key=lambda x: (x[0][2]-x[0][0])*(x[0][3]-x[0][1]), reverse=True)
        _, emb = faces[0]
        db[person_name].append(emb)
        print(f"Added embedding from {os.path.basename(path)}")

    save_enrolled_embeddings(EMB_PATH, db)
    print(f"Saved to {EMB_PATH}. Total samples for {person_name}: {len(db[person_name])}")

if __name__ == "__main__":
    import sys
    # ví dụ: python scripts/enroll_person.py Alice "data/enroll/Alice/*.jpg"
    main(sys.argv[1], sys.argv[2])
