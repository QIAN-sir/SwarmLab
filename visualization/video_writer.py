# -*- coding: utf-8 -*-
"""VideoWriterWithRate —— 按帧率录制 figure 为视频（对应 MATLAB graphics/VideoWriterWithRate.m）。

MATLAB 用 ``VideoWriter(path,'Motion JPEG AVI')`` + ``getframe`` + ``writeVideo``，按
output_rate 节流抓帧。Python 用 matplotlib.animation.FFMpegWriter（mp4/h264），自动探测
ffmpeg 可执行（常见路径 / PATH）。update(time, fig) 节流抓帧；close() 完成写盘。

用法::
    vw = VideoWriterWithRate("out.mp4", output_rate=0.1, fps=10)
    for t in ...:
        viewer.update(t, drone, map)      # 先刷新 figure
        vw.update(t, viewer.fig)          # 再抓帧
    vw.close()
"""
import os
import shutil

import matplotlib
from matplotlib import animation

_FFMPEG_CANDIDATES = [
    r"C:\APP\ffmpeg\ffmpeg_full\bin\ffmpeg.exe",   # 本项目已知 ffmpeg 位置
    "ffmpeg",                                       # PATH
]


def _find_ffmpeg():
    for p in _FFMPEG_CANDIDATES:
        if os.path.isabs(p):
            if os.path.exists(p):
                return p
        elif shutil.which(p):
            return shutil.which(p)
    return None


class VideoWriterWithRate:
    """按 output_rate 节流把 matplotlib figure 录成 mp4。"""

    def __init__(self, video_path, output_rate, fps=10, dpi=100):
        ff = _find_ffmpeg()
        if ff is not None and os.path.isabs(ff):
            matplotlib.rcParams["animation.ffmpeg_path"] = ff
        self.video_path = video_path
        self.output_rate = output_rate
        self.fps = fps
        self.dpi = dpi
        self.time_of_last_frame = 0.0
        self._writer = None
        self._fig = None
        self._nframes = 0

    # ------------------------------------------------------------------
    def update(self, time, fig):
        """满足 output_rate 间隔时抓取 fig 当前画面写入视频（对应 VideoWriterWithRate.update）。"""
        if fig is None:
            return
        if (time - self.time_of_last_frame) < self.output_rate - 1e-9:
            return
        self.time_of_last_frame = time
        if self._writer is None:
            self._fig = fig
            self._writer = animation.FFMpegWriter(fps=self.fps, bitrate=2000)
            self._writer.setup(fig, self.video_path, dpi=self.dpi)
        fig.canvas.draw()
        self._writer.grab_frame()
        self._nframes += 1

    def close(self):
        """完成写盘（必须调用，否则视频损坏，同 MATLAB close）。"""
        if self._writer is not None:
            self._writer.finish()
            self._writer = None
            self._fig = None

    @property
    def nframes(self):
        return self._nframes
