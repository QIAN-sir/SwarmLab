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
import subprocess

import numpy as np
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


class FrameVideoWriter:
    """把 numpy 帧序列 (H,W,3|4 uint8) 通过 ffmpeg rawvideo 管道写成 mp4。

    用于录制 GUI 交互画面（含用户视角旋转/缩放/平移）：每帧抓当前 3D 视图 widget 的
    像素（QWidget.grab()），窗口尺寸变化时用 PIL 缩放到首帧尺寸保证流连续。
    与 VideoWriterWithRate（matplotlib fig）不同，本类接受原始像素帧。
    """

    def __init__(self, path, fps=10):
        self.path = path
        self.fps = fps
        self._proc = None
        self._size = None          # (W,H)，首帧确定
        self.nframes = 0

    def add_frame(self, frame):
        """追加一帧 (H,W,3|4) uint8/可转 uint8。"""
        frame = np.asarray(frame)
        if frame.ndim != 3:
            return
        if frame.shape[2] == 4:
            frame = frame[..., :3]
        elif frame.shape[2] != 3:
            return
        frame = frame.astype(np.uint8, copy=False)
        H, W = frame.shape[0], frame.shape[1]
        if self._size is None:
            self._size = (W, H)
        elif (W, H) != self._size:
            frame = self._resize(frame, self._size)
            if frame is None:
                return
        if self._proc is None:
            ff = _find_ffmpeg()
            if ff is None:
                return
            W0, H0 = self._size
            self._proc = subprocess.Popen(
                [ff, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{W0}x{H0}", "-r", str(self.fps), "-i", "-",
                 "-c:v", "libx264", "-pix_fmt", "yuv420p", self.path],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
        try:
            self._proc.stdin.write(frame.tobytes())
            self.nframes += 1
        except (BrokenPipeError, OSError):
            pass

    @staticmethod
    def _resize(frame, size):
        try:
            from PIL import Image
            im = Image.fromarray(frame).resize(size, Image.BILINEAR)
            return np.asarray(im)
        except Exception:                                              # noqa: BLE001
            return None

    def close(self):
        """完成写盘（必须调用）。无帧时不生成文件。"""
        if self._proc is not None:
            try:
                self._proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            self._proc.wait()
            self._proc = None
