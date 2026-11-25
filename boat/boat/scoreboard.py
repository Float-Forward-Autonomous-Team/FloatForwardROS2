"""Draw the race state and numbers as one picture, so a dashboard can show them large."""

from io import BytesIO

from PIL import Image, ImageDraw, ImageFont

SIZE = (1600, 130)
BACKGROUND, LABEL, VALUE = (20, 22, 28), (150, 156, 168), (255, 255, 255)
GOOD, BAD, BUSY = (60, 200, 110), (255, 95, 85), (255, 200, 60)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size):
    """Return the bold font, or Pillow's own if it is not installed."""
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def fit(draw, text, size, width):
    """Return the largest font up to `size` that draws the text within `width` pixels."""
    while size > 16 and draw.textlength(text, font=font(size)) > width:
        size -= 2
    return font(size)


def cells(state, values, gates):
    """Return what to show: (label, text, colour, width share) from left to right."""
    stopped = state.startswith("STOPPED")
    colour = BAD if stopped else GOOD if state == "FINISHED" else BUSY
    missed, touched = int(values["missed"]), int(values["touched"])
    return [
        ("STATE", state, colour, 5),
        ("TIME", f"{values['elapsed']:.1f} s", VALUE, 3),
        ("SPEED", f"{values['speed_kn']:.1f} kn", VALUE, 3),
        ("NEXT GATE", f"{values['next_gate_m']:.0f} m", VALUE, 2.4),
        ("GATES", f"{int(values['passed'])} / {gates}", VALUE, 2.4),
        ("MISSED", str(missed), BAD if missed else GOOD, 1.8),
        ("TOUCHED", str(touched), BAD if touched else GOOD, 2.1),
    ]


def draw_scoreboard(state, values, gates):
    """Return the scoreboard as JPEG bytes."""
    image = Image.new("RGB", SIZE, BACKGROUND)
    draw = ImageDraw.Draw(image)
    row = cells(state, values, gates)
    unit = SIZE[0] / sum(c[3] for c in row)
    x = 0.0
    for label, text, colour, share in row:
        width = share * unit
        draw.text((x + 18, 12), label, font=font(22), fill=LABEL)
        big = fit(draw, text, 52, width - 30)
        draw.text((x + 18, 82), text, font=big, fill=colour, anchor="lm")
        x += width
        if x < SIZE[0] - 1:
            draw.line([(x, 14), (x, SIZE[1] - 14)], fill=(60, 64, 74), width=2)
    out = BytesIO()
    image.save(out, format="JPEG", quality=85)
    return out.getvalue()
