"""Shared helpers for building the demo videos with ffmpeg.

All on-screen text comes from video_text.toml. Long text is wrapped automatically to
fit its panel, and a warning is printed if it still does not fit.
"""
import os
import subprocess
import sys
import textwrap
import tomllib

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MEDIA = os.path.join(REPO, 'media')
TEXT_FILE = os.path.join(REPO, 'videos', 'video_text.toml')

BOLD = '/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf'
REGULAR = '/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf'
MONO = '/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf'
CHAR_WIDTH = {BOLD: 0.57, REGULAR: 0.53, MONO: 0.61}  # average glyph width as a fraction of font size

BG = '0x14171c'
ACCENT = '0xf5a623'
BODY = '0xd5dbe1'
DIM = '0x8b95a1'
SUBTLE = '0xb8c2cc'


def load_text():
    with open(TEXT_FILE, 'rb') as f:
        return tomllib.load(f)


class Builder:
    def __init__(self, name):
        self.work = os.path.join(MEDIA, 'build', name)
        os.makedirs(self.work, exist_ok=True)
        self._n = 0
        self.warnings = []

    # ---------- text helpers ----------
    def _tf(self, text):
        self._n += 1
        path = os.path.join(self.work, f'text_{self._n}.txt')
        with open(path, 'w') as f:
            f.write(text)
        return path

    def wrap(self, text, width_px, size, font=REGULAR, max_lines=None, where=''):
        """Wrap paragraphs (separated by blank lines) to fit width_px. Returns a list of lines."""
        max_chars = max(10, int(width_px / (CHAR_WIDTH[font] * size)))
        out = []
        for para in [p for p in text.strip().split('\n\n')]:
            if out:
                out.append('')
            out.extend(textwrap.wrap(' '.join(para.split()), max_chars) or [''])
        if max_lines and len(out) > max_lines:
            self.warnings.append(f'{where}: text needs {len(out)} lines but only {max_lines} fit; it will run off the panel')
        return out

    def check_width(self, text, width_px, size, font, where):
        if len(text) * CHAR_WIDTH[font] * size > width_px:
            self.warnings.append(f'{where}: "{text}" is probably too long for one line')

    def lines(self, lines, x, y, size, font=REGULAR, color='white'):
        out = []
        step = int(size * 1.45)
        for i, line in enumerate(lines):
            if line.strip():
                out.append(f"drawtext=fontfile={font}:textfile={self._tf(line)}:expansion=none:fontsize={size}:"
                           f"fontcolor={color}:x={x}:y=({y})+{i * step}")
        return ','.join(out) if out else 'null'

    def paragraph(self, text, x, y, size, width_px, max_lines, font=REGULAR, color=BODY, where=''):
        return self.lines(self.wrap(text, width_px, size, font, max_lines, where), x, y, size, font, color)

    def label(self, text, x, y, size=24, font=REGULAR):
        return (f"drawtext=fontfile={font}:textfile={self._tf(text)}:expansion=none:fontsize={size}:fontcolor=white:"
                f"box=1:boxcolor=black@0.6:boxborderw=8:x={x}:y={y}")

    def banner(self, text, color):
        return (f"drawtext=fontfile={BOLD}:textfile={self._tf(text)}:expansion=none:fontsize=44:fontcolor=white:"
                f"box=1:boxcolor={color}@0.92:boxborderw=26:x=(w-tw)/2:y=66")

    # ---------- rendering ----------
    def run(self, inputs, graph, name):
        out = os.path.join(self.work, name + '.mp4')
        gp = out + '.graph'
        with open(gp, 'w') as f:
            f.write(graph)
        args = sum((['-i', i] for i in inputs), [])
        subprocess.run(['ffmpeg', '-v', 'error', '-y'] + args + ['-/filter_complex', gp, '-map', '[out]', '-r', '30', '-an',
                        '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', out], check=True)
        print('built', name)
        return out

    @staticmethod
    def fades(d, f=0.4):
        return f"fade=t=in:st=0:d={f},fade=t=out:st={d - f:.3f}:d={f}"

    def card(self, name, title, subtitle, d):
        self.check_width(title, 1800, 76, BOLD, f'{name} title')
        self.check_width(subtitle, 1800, 34, REGULAR, f'{name} subtitle')
        g = (f"color=c={BG}:s=1920x1080:d={d}:r=30,"
             f"{self.lines([title], '(w-tw)/2', 'h/2-90', 76, BOLD)},"
             f"{self.lines([subtitle], '(w-tw)/2', 'h/2+20', 34, REGULAR, SUBTLE)},"
             f"drawbox=x=(iw-160)/2:y=ih/2-2:w=160:h=4:color={ACCENT}:t=fill,{self.fades(d)}[out]")
        return self.run([], g, name)

    def concat(self, names, final, crf=18):
        lst = os.path.join(self.work, 'concat.txt')
        with open(lst, 'w') as f:
            f.writelines(f"file '{os.path.join(self.work, n + '.mp4')}'\n" for n in names)
        os.makedirs(os.path.dirname(final), exist_ok=True)
        # Re-encode the join so timestamps are continuous (stream-copy joins can confuse seeking).
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', lst, '-vf', 'fps=30',
                        '-c:v', 'libx264', '-preset', 'slow', '-crf', str(crf), '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', final], check=True)
        size = os.path.getsize(final) / 1e6
        print(f'FINAL {final} ({size:.1f} MB)')

    def report(self):
        if self.warnings:
            print('\nText warnings:', file=sys.stderr)
            for w in self.warnings:
                print('  - ' + w, file=sys.stderr)
        else:
            print('All text fits.')


def duration(path):
    return float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', path],
                                capture_output=True, text=True, check=True).stdout)
