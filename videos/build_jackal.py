"""Build media/videos/jackal_gazebo_demo.mp4 from the synced Gazebo recordings in media/jackal_sim/.

Usage:  python3 videos/build_jackal.py
All on-screen text comes from videos/video_text.toml.
"""
import os

from common import (ACCENT, BG, BODY, BOLD, MEDIA, MONO, REGULAR, Builder, load_text)

T = load_text()
J = T['jackal']
BADGE = T['style']['speed_badge']
b = Builder('jackal')
RAW = os.path.join(MEDIA, 'jackal_sim')


def segment(name, inputs, layout, t0, t1, speed, fade_in=False, fade_out=False, banner=None, banner_color='0x1e8e3e'):
    d = (t1 - t0) / speed
    tr = f"trim={t0}:{t1},setpts=(PTS-STARTPTS)/{speed}"
    badge = b.label(BADGE.format(speed=f'{speed:g}'), 'w-tw-24', 22, 30, BOLD) if speed != 1 else None
    g = layout(tr, badge, d)
    post = []
    if banner:
        post.append(b.banner(banner, banner_color))
    if fade_in:
        post.append('fade=t=in:st=0:d=0.4')
    if fade_out:
        post.append(f'fade=t=out:st={d - 0.4:.3f}:d=0.4')
    if post:
        g = g.replace('[out]', '[pre]') + ';[pre]' + ','.join(post) + '[out]'
    b.run(inputs, g, name)


# ---------------- obstacle avoidance ----------------
OA = J['obstacle_avoidance']
b.check_width(OA['heading'], 900, 40, BOLD, 'obstacle_avoidance.heading')
b.check_width(OA['subheading'], 900, 22, MONO, 'obstacle_avoidance.subheading')
CAP1 = b.paragraph(OA['caption'], 1000, 846, 23, 900, 6, where='jackal.obstacle_avoidance.caption')


def layout_avoid(tr, badge, d):
    ch = f"[0]{tr},scale=1220:720" + (f",{badge}" if badge else '') + f",{b.label(OA['label_chase'], 20, 20)}[ch]"
    return (f"color=c={BG}:s=1920x1080:r=30:d={d}[bg];{ch};"
            f"[1]{tr},scale=690:690,{b.label(OA['label_overhead'], 16, 16)}[ov];"
            f"[2]{tr},scale=480:360,{b.label(OA['label_front'], 12, 12, 22)}[fc];"
            f"[3]{tr},format=yuv420p,scale=480:360,{b.label(OA['label_lane'], 12, 12, 22)}[ld];"
            f"[bg][ch]overlay=0:0:shortest=1[a];[a][ov]overlay=1225:15[b];[b][fc]overlay=0:720[c];[c][ld]overlay=480:720[d];"
            f"[d]{b.lines([OA['heading']], 1000, 742, 40, BOLD)},"
            f"{b.lines([OA['subheading']], 1000, 800, 22, MONO, ACCENT)},{CAP1}[out]")


T1 = os.path.join(RAW, 'raw_take1_obstacle_avoidance')
in1 = [f'{T1}/gazebo_chase.mp4', f'{T1}/overhead_camera.mp4', f'{T1}/front_camera.mp4', f'{T1}/lane_detection.mp4']
segment('t1a', in1, layout_avoid, 0, 16, 1, fade_in=True)
segment('t1b', in1, layout_avoid, 16, 60, 8, fade_out=True)

# ---------------- goal navigation ----------------
NV = J['navigation']


def make_layout(goal_key, placeholder=None):
    G = NV[goal_key]
    b.check_width(G['heading'], 545, 29, BOLD, f'{goal_key}.heading')
    cap = b.paragraph(G['caption'], 1355, 716, 21, 545, 11, where=f'jackal.navigation.{goal_key}.caption')
    astar_label = G.get('label_astar', NV['label_astar'])
    extra = ''
    if placeholder:  # cover the baked-in note on the empty A* panel and draw the editable one
        lines = b.wrap(placeholder, 560, 26, max_lines=4, where=f'{goal_key}.astar_placeholder')
        extra = (",drawbox=x=0:y=420:w=640:h=200:color=0x2a2f36:t=fill,"
                 + ','.join(b.lines([ln], '(w-tw)/2', 470 + i * 38, 26, REGULAR, '0xb8c2cc')
                            for i, ln in enumerate(lines) if ln))

    def layout(tr, badge, d):
        ch = f"[3]{tr},scale=746:440,{b.label(NV['label_chase'], 12, 12, 22)}" + (f",{badge}" if badge else '') + "[ch]"
        return (f"color=c={BG}:s=1920x1080:r=30:d={d}[bg];"
                f"[0]{tr},scale=640:640,{b.label(NV['label_overhead'], 14, 14)}[ov];"
                f"[1]{tr},scale=640:640,{b.label(NV['label_map'], 14, 14)}[mp];"
                f"[2]{tr},scale=640:640{extra},{b.label(astar_label, 14, 14)}[as];{ch};"
                f"[4]{tr},scale=587:440,{b.label(NV['label_front'], 12, 12, 22)}[fc];"
                f"[bg][ov]overlay=0:0:shortest=1[a];[a][mp]overlay=640:0[b];[b][as]overlay=1280:0[c];"
                f"[c][ch]overlay=0:640[d];[d][fc]overlay=746:640[e];"
                f"[e]drawbox=x=640:y=0:w=2:h=640:color={BG}:t=fill,drawbox=x=1280:y=0:w=2:h=640:color={BG}:t=fill,"
                f"{b.lines([G['heading']], 1355, 662, 29, BOLD)},{cap}[out]")
    return layout


def ins(folder):
    d = os.path.join(RAW, folder)
    return [f'{d}/overhead_with_path.mp4', f'{d}/occupancy_mapper_window.mp4', f'{d}/astar_plan_with_path.mp4',
            f'{d}/gazebo_chase.mp4', f'{d}/front_camera.mp4']


LA = make_layout('goal1')
segment('gA1', ins('raw_goal1_up_the_corridor'), LA, 0, 18, 1, fade_in=True)
segment('gA2', ins('raw_goal1_up_the_corridor'), LA, 18, 34.4, 6)
segment('gA3', ins('raw_goal1_up_the_corridor'), LA, 34.4, 37.9, 1, fade_out=True, banner=NV['goal1']['banner'])

LB = make_layout('goal2')
segment('gB1', ins('raw_goal2_around_the_bend'), LB, 0, 14, 1, fade_in=True)
segment('gB2', ins('raw_goal2_around_the_bend'), LB, 14, 74.6, 5)
segment('gB3', ins('raw_goal2_around_the_bend'), LB, 74.6, 78.6, 1, fade_out=True, banner=NV['goal2']['banner'])

LC = make_layout('goal3', placeholder=NV['goal3']['astar_placeholder'])
segment('gC1', ins('raw_goal3_past_the_wall'), LC, 0, 14, 2, fade_in=True)
segment('gC2', ins('raw_goal3_past_the_wall'), LC, 14, 70, 10)
segment('gC3', ins('raw_goal3_past_the_wall'), LC, 70, 74, 1, fade_out=True,
        banner=NV['goal3']['banner'], banner_color='0xb3261e')

# ---------------- cards and final join ----------------
b.card('c0_title', J['title_card']['title'], J['title_card']['subtitle'], 4)
b.card('c1_sec', OA['card_title'], OA['card_subtitle'], 3)
b.card('c2_sec', NV['card_title'], NV['card_subtitle'], 3.5)
b.card('c9_end', J['end_card']['title'], J['end_card']['subtitle'], 4)

b.concat(['c0_title', 'c1_sec', 't1a', 't1b', 'c2_sec', 'gA1', 'gA2', 'gA3', 'gB1', 'gB2', 'gB3',
          'gC1', 'gC2', 'gC3', 'c9_end'], os.path.join(MEDIA, 'videos', 'jackal_gazebo_demo.mp4'), crf=18)
b.report()
