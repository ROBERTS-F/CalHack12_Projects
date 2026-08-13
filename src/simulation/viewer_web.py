import os
os.environ["MUJOCO_GL"] = "egl"  # headless

import time
from pathlib import Path
import numpy as np
import mujoco
from flask import Flask, Response

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
MODEL_XML = (REPO_ROOT / "mujoco_models" / "so_arm100.xml").resolve()

app = Flask(__name__)

model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
data  = mujoco.MjData(model)
renderer = mujoco.Renderer(model, width=640, height=480)

# simple static pose (no motion)
if model.nq >= 6:
    data.qpos[:] = 0.0
    data.qpos[0] = np.pi/2
    data.qpos[1] = -np.pi/4
    data.qpos[2] =  np.pi/4
    data.qpos[3] = -np.pi/4
mujoco.mj_forward(model, data)

def mjpeg_stream():
    import imageio.v2 as iio
    while True:
        img = renderer.render()
        buf = iio.imwrite("<bytes>", img, format="jpeg", quality=80)
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n")
        time.sleep(0.033)  # ~30 FPS

@app.route("/")
def index():
    return '<html><body><img src="/stream" /></body></html>'

@app.route("/stream")
def stream():
    return Response(mjpeg_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=False, use_reloader=False)
