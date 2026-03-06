#!/usr/bin/env python3

"""Utils for Edge RTC."""

from dataclasses import dataclass


@dataclass
class EdgeRTCConfig:
    video_device: str | None
    framerate: int
    resolution: str
    bitrate: int
    host: str = "0.0.0.0"
    port: int = 8080
