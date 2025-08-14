import tkinter as tk
from tkinter import simpledialog, messagebox
import sqlite3
import threading
import cv2
from PIL import Image, ImageTk

DB_PATH = "camera.db"

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

class CameraStream:
    def __init__(self, parent, camera):
        self.parent = parent
        self.id, self.name, self.video_path = camera
        self.label = tk.Label(parent, text=self.name)
        self.label.pack()
        self.canvas = tk.Label(parent)
        self.canvas.pack()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.update_frame)
        self.thread.start()

    def update_frame(self):
        while not self.stop_event.is_set():
            cap = cv2.VideoCapture(self.video_path, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.label.config(text=f"{self.name} (Cannot open)")
                import time
                time.sleep(5)
                continue
            while not self.stop_event.is_set():
                ret, frame = cap.read()
                if not ret:
                    break
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)
                self.canvas.imgtk = imgtk
                self.canvas.config(image=imgtk)
            cap.release()
            time.sleep(5)

    def stop(self):
        self.stop_event.set()
        self.thread.join()

class CameraManagerGUI:
    def __init__(self):
        init_db()
        self.root = tk.Tk()
        self.root.title("Camera Manager & Viewer")
        self.cameras_frame = tk.Frame(self.root)
        self.cameras_frame.pack(side=tk.RIGHT)
        self.controls_frame = tk.Frame(self.root)
        self.controls_frame.pack(side=tk.LEFT, padx=10)

        self.listbox = tk.Listbox(self.controls_frame)
        self.listbox.pack(padx=5, pady=5)
        self.refresh_list()

        tk.Button(self.controls_frame, text="Add Camera", command=self.add_camera).pack(pady=5)
        tk.Button(self.controls_frame, text="Delete Camera", command=self.delete_camera).pack(pady=5)

        self.camera_streams = []

    def refresh_list(self):
        self.listbox.delete(0, tk.END)
        for cam in get_cameras():
            self.listbox.insert(tk.END, f"{cam[0]}: {cam[1]}")

    def add_camera(self):
        name = simpledialog.askstring("Camera Name", "Enter camera name:")
        # topic = simpledialog.askstring("Topic", "Enter ROS topic:")
        video_path = simpledialog.askstring("Video Path", "Enter RTSP URL or video path:")
        # if name and topic:
        if name:
            topic = f"/{name.strip().replace(' ', '_')}/image_raw"
            add_camera(name, topic, video_path or '')
            self.refresh_list()
            self.start_camera_streams()

    def delete_camera(self):
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a camera first")
            return
        camera_id = int(self.listbox.get(selection[0]).split(":")[0])
        delete_camera(camera_id)
        self.refresh_list()
        self.start_camera_streams()

    def start_camera_streams(self):
        for cs in self.camera_streams:
            cs.stop()
        self.camera_streams.clear()
        for cam in get_cameras():
            cs = CameraStream(self.cameras_frame, cam)
            self.camera_streams.append(cs)

    def run(self):
        self.start_camera_streams()
        self.root.mainloop()
        for cs in self.camera_streams:
            cs.stop()

def main():
    gui = CameraManagerGUI()
    gui.run()

if __name__ == "__main__":
    main()
