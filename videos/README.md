# videos

This folder holds the scripts that put together the two demo videos, along with all of the text that appears in them. The videos themselves are built from footage in `media/`, which is kept out of git because of its size.

| Video | Script | Built from |
| --- | --- | --- |
| `media/videos/jackal_gazebo_demo.mp4` | `build_jackal.py` | Synced Gazebo recordings in `media/jackal_sim/` |
| `media/videos/terrasentia_demo.mp4` | `build_terrasentia.py` | Photos, phone videos and side-by-side clips in `media/terrasentia/` |

## Changing the text

Every title, caption, label and banner lives in [`video_text.toml`](video_text.toml). After editing it, rebuild the videos with:

```bash
python3 videos/build_jackal.py
python3 videos/build_terrasentia.py
```

Captions can be written as ordinary paragraphs. They are wrapped automatically to fit their panel, and a blank line inside a caption starts a new paragraph. If a piece of text is too long for its space, the build prints a warning. Removing one of the photo captions from the TerraSentia section leaves that photo out of the video.

TOML treats a backslash as the start of an escape sequence, so text like `A*` should be written as it is, without a backslash in front of the asterisk.

## How the footage was made

For the Jackal video, each run was recorded straight from the simulation. The robot's front camera and an overhead camera added to the Gazebo world were saved from their ROS topics, while the Gazebo window and the matplotlib plots were screen recorded. Every recording was timestamped with the clock, so the views could be lined up afterwards. The pink driven path and the green goal marker were drawn from the robot's logged ground-truth position.

For the TerraSentia video, the phone recordings and the RViz screen recordings were lined up using the clock that RViz shows on screen, then placed side by side.

## Requirements

- `ffmpeg`, with `libx264` and the `drawtext` filter
- Python 3.11 or newer, which includes the `tomllib` module used to read the text file
- The Noto Sans fonts in `/usr/share/fonts/truetype/noto/`

Intermediate files go to `media/build/`, which is safe to delete at any time.
