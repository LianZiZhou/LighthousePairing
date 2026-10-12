"""Generate src/app.ico (concentric lighthouse rings on a rounded square)."""
from PIL import Image, ImageDraw
import os

out = os.path.join(os.path.dirname(__file__), "..", "src", "app.ico")
S = 256
img = Image.new("RGBA", (S, S), (0, 0, 0, 0))
d = ImageDraw.Draw(img)
d.rounded_rectangle([8, 8, S - 8, S - 8], radius=56, fill=(23, 27, 36, 255))
c = S / 2
for r, w, a in [(92, 10, 70), (66, 12, 255)]:
    d.ellipse([c - r, c - r, c + r, c + r], outline=(79, 140, 255, a), width=w)
d.ellipse([c - 26, c - 26, c + 26, c + 26], fill=(79, 140, 255, 255))
# small "beam" wedge
d.pieslice([c - 112, c - 112, c + 112, c + 112], start=-35, end=-15, fill=(79, 140, 255, 90))
img.save(out, sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
print("wrote", os.path.abspath(out))
