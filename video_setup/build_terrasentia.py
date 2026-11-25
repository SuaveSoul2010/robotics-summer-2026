"""Build media/videos/terrasentia_demo.mp4 from the photos and clips in media/terrasentia/.

Usage:  python3 videos/build_terrasentia.py
All on-screen text comes from videos/video_text.toml.
"""
import os

from common import (ACCENT, BG, BODY, BOLD, DIM, MEDIA, MONO, REGULAR, Builder, duration, load_text)

T = load_text()
X = T['terrasentia']
BADGE = T['style']['speed_badge']
b = Builder('terrasentia')
TS = os.path.join(MEDIA, 'terrasentia')
PHOTOS = os.path.join(TS, 'photos')
fades = b.fades

# ---------- 1. Meet the robot: slow-zoom photos with a caption panel ----------
MR = X['meet_the_robot']
b.check_width(MR['heading'], 580, 44, BOLD, 'meet_the_robot.heading')
photos = [  # (file, crop "w:h:x:y" in displayed pixels or None, box width, box height, caption key)
    ('front.jpg', None, 810, 1080, 'caption_front'),
    ('front.jpg', '1700:1480:525:635', 1240, 1080, 'caption_camera'),
    ('top.jpg', None, 810, 1080, 'caption_top'),
    ('view1.jpg', None, 1080, 1080, 'caption_lab_1'),
    ('view2.jpg', None, 1080, 1080, 'caption_lab_2'),
]
photos = [p for p in photos if p[4] in MR]  # a photo is left out when its caption is removed from the text file
D_PHOTO = 3.4
for i, (f, crop, bw, bh, key) in enumerate(photos):
    n = int(D_PHOTO * 30)
    c = f"crop={crop}," if crop else ''
    cap = b.paragraph(MR[key], 1300, 450, 27, 580, 15, where=f'terrasentia.meet_the_robot.{key}')
    g = (f"color=c={BG}:s=1920x1080:r=30:d={D_PHOTO}[bg];"
         f"[0]{c}scale={bw * 2}:{bh * 2},zoompan=z='1+0.0009*on':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)'"
         f":d={n}:s={bw}x{bh}:fps=30,setsar=1[ph];"
         f"[bg][ph]overlay=x={(1240 - bw) // 2}:y={(1080 - bh) // 2}:shortest=1[a];"
         f"[a]drawbox=x=1300:y=420:w=90:h=4:color={ACCENT}:t=fill,{b.lines([MR['heading']], 1300, 350, 44, BOLD)},{cap},"
         + ("fade=t=in:st=0:d=0.4," if i == 0 else '')
         + (f"fade=t=out:st={D_PHOTO - 0.4:.2f}:d=0.4," if i == len(photos) - 1 else '')
         + "null[out]")
    b.run([os.path.join(PHOTOS, f)], g, f'p{i}')

# ---------- 2. depth_roomba ----------
OA = X['obstacle_avoidance']
b.check_width(OA['heading'], 1180, 44, BOLD, 'obstacle_avoidance.heading')
capR = b.paragraph(OA['caption'], 700, 210, 26, 1180, 8, where='terrasentia.obstacle_avoidance.caption')
roomba = os.path.join(TS, 'source', 'phone', 'roomba.mp4')
speed = 1.5
d = duration(roomba) / speed
g = (f"color=c={BG}:s=1920x1080:r=30:d={d}[bg];"
     f"[0]setpts=PTS/{speed},scale=608:1080,{b.label(OA['label_video'], 14, 14, 22)},"
     f"{b.label(BADGE.format(speed=f'{speed:g}'), 'w-tw-14', 'h-th-22', 26, BOLD)}[v];"
     f"[1]crop=1700:1275:525:735,scale=560:420,{b.label(OA['label_camera'], 12, 12, 20)}[cam];"
     f"[bg][v]overlay=0:0:shortest=1[a];[a][cam]overlay=700:560[b];"
     f"[b]{b.lines([OA['heading']], 700, 110, 44, BOLD)},drawbox=x=700:y=180:w=90:h=4:color={ACCENT}:t=fill,"
     f"{capR},{fades(d)}[out]")
b.run([roomba, os.path.join(PHOTOS, 'front.jpg')], g, 'roomba')

# ---------- 3. the map still ----------
MP = X['mapping']
b.check_width(MP['map_heading'], 580, 44, BOLD, 'mapping.map_heading')
capMap = b.paragraph(MP['map_caption'], 1300, 360, 27, 580, 18, where='terrasentia.mapping.map_caption')
DM = 7.5
n = int(DM * 30)
g = (f"color=c={BG}:s=1920x1080:r=30:d={DM}[bg];"
     f"[0]crop=1650:1437:570:180,scale=2480:2160,zoompan=z='1+0.0007*on':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)'"
     f":d={n}:s=1240x1080:fps=30,setsar=1,{b.label(MP['map_label'], 16, 16)}[m];"
     f"[bg][m]overlay=0:0:shortest=1[a];"
     f"[a]drawbox=x=1300:y=330:w=90:h=4:color={ACCENT}:t=fill,{b.lines([MP['map_heading']], 1300, 260, 44, BOLD)},"
     f"{capMap},{fades(DM)}[out]")
b.run([os.path.join(PHOTOS, 'RVIZ_Occupancy_map.png')], g, 'map')

# ---------- 4. side-by-side runs with a pipeline diagram ----------
PL = X['pipeline']


def pipeline(x0=1250, y0=612):
    steps = PL['steps']
    gap = min(82, int(400 / max(1, len(steps))))
    parts = [b.lines([PL['title']], x0, y0, 22, BOLD, DIM)]
    for i, step in enumerate(steps):
        y = y0 + 44 + i * gap
        prog = 0 < i < len(steps) - 1  # the middle steps are programs; first and last are data
        b.check_width(step, 590, 21, MONO if prog else REGULAR, f'pipeline step {i + 1}')
        parts.append(f"drawbox=x={x0}:y={y}:w=620:h=52:color=0x232a33:t=fill")
        parts.append(f"drawbox=x={x0}:y={y}:w=5:h=52:color={ACCENT if prog else DIM}:t=fill")
        parts.append(b.lines([step], x0 + 22, y + 13, 21, MONO if prog else REGULAR, 'white' if prog else BODY))
        if i < len(steps) - 1:
            cx = x0 + 300
            parts.append(f"drawbox=x={cx}:y={y + 54}:w=4:h={gap - 64}:color={DIM}:t=fill")
            for k, wdt in enumerate((14, 8, 2)):
                parts.append(f"drawbox=x={cx + 2 - wdt // 2}:y={y + gap - 10 + k * 3}:w={wdt}:h=3:color={DIM}:t=fill")
    return ','.join(parts)


PIPE = pipeline()
SB = os.path.join(TS, 'side_by_side')


def sbs_clip(name, src, key, speed=1.0):
    R = X[key]
    b.check_width(R['heading'], 1150, 36, BOLD, f'{key}.heading')
    cap = b.paragraph(R['caption'], 60, 712, 25, 1150, 9, where=f'terrasentia.{key}.caption')
    d = duration(src) / speed
    badge = f",{b.label(BADGE.format(speed=f'{speed:g}'), 'w-tw-20', 'h-th-20', 28, BOLD)}" if speed != 1 else ''
    g = (f"color=c={BG}:s=1920x1080:r=30:d={d}[bg];"
         f"[0]setpts=PTS/{speed},scale=1920:590{badge}[v];[bg][v]overlay=0:0:shortest=1[a];"
         f"[a]{b.lines([R['heading']], 60, 626, 36, BOLD)},drawbox=x=60:y=684:w=80:h=4:color={ACCENT}:t=fill,"
         f"{cap},{PIPE},{fades(d)}[out]")
    b.run([src], g, name)


sbs_clip('s1', f'{SB}/sbs_20260820_173043.mp4', 'run1')
sbs_clip('s2', f'{SB}/sbs_20260820_170814.mp4', 'run2')
sbs_clip('s3', f'{SB}/sbs_20260820_173656.mp4', 'run3')
sbs_clip('s4', f'{SB}/sbs_20260820_173947.mp4', 'run4', speed=1.5)

# ---------- cards and final join ----------
b.card('c0', X['title_card']['title'], X['title_card']['subtitle'], 4)
b.card('c1', OA['card_title'], OA['card_subtitle'], 3)
b.card('c2', MP['card_title'], MP['card_subtitle'], 3.5)
b.card('c9', X['end_card']['title'], X['end_card']['subtitle'], 4)

b.concat(['c0'] + [f'p{i}' for i in range(len(photos))] + ['c1', 'roomba', 'c2', 'map', 's1', 's2', 's3', 's4', 'c9'],
         os.path.join(MEDIA, 'videos', 'terrasentia_demo.mp4'), crf=23)
b.report()
