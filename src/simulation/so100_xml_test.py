# ---- minimal, headless snapshot ----
import os
os.environ["MUJOCO_GL"] = "egl"   # headless backend

import mujoco
from pathlib import Path
import numpy as np
import imageio.v2 as iio

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
MODEL_XML = (REPO_ROOT / "mujoco_models" / "so_arm100.xml").resolve()

def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data  = mujoco.MjData(model)

    # optional: a simple pose
    if model.nq >= 6:
        data.qpos[:] = 0
        data.qpos[0] = np.pi/2
        data.qpos[1] = -np.pi/4
        data.qpos[2] =  np.pi/4
        data.qpos[3] = -np.pi/4
    mujoco.mj_forward(model, data)

    # use default offscreen framebuffer size (640 wide)
    renderer = mujoco.Renderer(model, width=640, height=480)
    img = renderer.render()
    out = HERE / "snapshot.png"
    iio.imwrite(out, img)
    print(f"Saved {out}")

if __name__ == "__main__":
    main()
