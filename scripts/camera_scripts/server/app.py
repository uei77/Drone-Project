import base64, io, os, time
from fastapi import FastAPI, Header, HTTPException
from pydantic import BaseModel
from PIL import Image
app = FastAPI()
TOKEN = os.getenv("API_TOKEN", "gizli-anahtar")
SAVE_FRAMES = os.getenv("SAVE_FRAMES", "0") == "1"
os.makedirs("frames", exist_ok=True)
class Frame(BaseModel):
    device_id: str
    seq: int
    image: str 
def run_model(img: Image.Image):
    return "hold", 0.0
@app.post("/frame")
def frame(f: Frame, x_token: str = Header(None)):
    if x_token != TOKEN:
        raise HTTPException(401, "unauthorized")
    try:
        raw = base64.b64decode(f.image, validate=True)
        if SAVE_FRAMES:
            with open(f"frames/{f.device_id}_{f.seq}_{time.time():.3f}.jpg", "wb") as fp:
                fp.write(raw)
        img = Image.open(io.BytesIO(raw)).convert("RGB")
        action, conf = run_model(img)
    except Exception:
        return {"seq": f.seq, "action": "hold", "confidence": 0.0}
    return {"seq": f.seq, "action": action, "confidence": round(float(conf), 3)}
