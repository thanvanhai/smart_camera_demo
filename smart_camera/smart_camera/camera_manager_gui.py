import tkinter as tk
from tkinter import simpledialog, messagebox
import sqlite3
import cv2
from PIL import Image, ImageTk

DB_PATH = "camera.db"

# -------------------- Database --------------------
def init_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS cameras (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            topic TEXT NOT NULL,
            video_path TEXT DEFAULT ''
        )
    """)
    conn.commit()
    conn.close()

def get_cameras():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT id, name, video_path FROM cameras")
    rows = cursor.fetchall()
    conn.close()
    return rows

def add_camera(name, topic, video_path):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO cameras (name, topic, video_path) VALUES (?, ?, ?)", (name, topic, video_path))
    conn.commit()
    conn.close()

def delete_camera(camera_id):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("DELETE FROM cameras WHERE id=?", (camera_id,))
    conn.commit()
    conn.close()

# -------------------- Camera Stream --------------------
class CameraStream:
    def __init__(self, canvas, label, camera):
        self.canvas = canvas
        self.label = label
        self.id, self.name, self.video_path = camera
        self.label.config(text=self.name)

        self.cap = cv2.VideoCapture(self.video_path, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.label.config(text=f"{self.name} (Cannot open)")
            return

        self.stop_event = False
        self.update_frame()

    def update_frame(self):
        if self.stop_event or not self.cap.isOpened():
            if self.cap.isOpened():
                self.cap.release()
            return
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.canvas.imgtk = imgtk
            self.canvas.config(image=imgtk)
        self.canvas.after(30, self.update_frame)

    def stop(self):
        self.stop_event = True

# -------------------- GUI --------------------
class CameraManagerGUI:
    def __init__(self):
        init_db()
        self.root = tk.Tk()
        self.root.title("Camera Manager & Viewer")

        # Controls frame (list + buttons)
        self.controls_frame = tk.Frame(self.root)
        self.controls_frame.pack(side=tk.LEFT, padx=10)
        self.listbox = tk.Listbox(self.controls_frame)
        self.listbox.pack(padx=5, pady=5)
        self.listbox.bind("<<ListboxSelect>>", self.on_camera_select)

        tk.Button(self.controls_frame, text="Add Camera", command=self.add_camera).pack(pady=5)
        tk.Button(self.controls_frame, text="Delete Camera", command=self.delete_camera).pack(pady=5)

        # Stream frame (1 canvas + 1 label duy nhất)
        self.stream_frame = tk.Frame(self.root)
        self.stream_frame.pack(side=tk.RIGHT, padx=10)
        self.label = tk.Label(self.stream_frame, text="")
        self.label.pack()
        self.canvas = tk.Label(self.stream_frame)
        self.canvas.pack()

        self.current_stream = None
        self.refresh_list()

    # -------------------- Refresh list --------------------
    def refresh_list(self):
        self.listbox.delete(0, tk.END)
        for cam in get_cameras():
            self.listbox.insert(tk.END, f"{cam[0]}: {cam[1]}")

    # -------------------- Add camera --------------------
    def add_camera(self):
        name = simpledialog.askstring("Camera Name", "Enter camera name:")
        video_path = simpledialog.askstring("Video Path", "Enter RTSP URL or video path:")
        if name:
            topic = f"/{name.strip().replace(' ', '_')}/image_raw"
            add_camera(name, topic, video_path or '')
            self.refresh_list()

    # -------------------- Delete camera --------------------
    def delete_camera(self):
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a camera first")
            return
        camera_id = int(self.listbox.get(selection[0]).split(":")[0])
        delete_camera(camera_id)
        self.refresh_list()
        # Dừng camera đang hiển thị nếu bị xóa
        if self.current_stream and self.current_stream.id == camera_id:
            self.current_stream.stop()
            self.current_stream = None
            self.label.config(text="")
            self.canvas.config(image='')

    # -------------------- Lazy load stream --------------------
    def on_camera_select(self, event):
        selection = self.listbox.curselection()
        if not selection:
            return
        camera_id = int(self.listbox.get(selection[0]).split(":")[0])
        cam = [c for c in get_cameras() if c[0] == camera_id][0]

        # Dừng stream hiện tại
        if self.current_stream:
            self.current_stream.stop()

        # Start stream mới, dùng 1 canvas + label duy nhất
        self.current_stream = CameraStream(self.canvas, self.label, cam)

    # -------------------- Run GUI --------------------
    def run(self):
        self.root.mainloop()
        if self.current_stream:
            self.current_stream.stop()

# -------------------- Main --------------------
def main():
    gui = CameraManagerGUI()
    gui.run()

if __name__ == "__main__":
    main()
