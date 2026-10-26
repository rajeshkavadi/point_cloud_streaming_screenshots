"""Configuration: JSON config file < environment variables < CLI overrides."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, fields
from typing import List, Optional


ENV_PREFIX = "PCSNAP_"


@dataclass
class Config:
    # ROS
    topic: str = "/ouster/points"        # PointCloud2 topic to snapshot
    ros_domain_id: Optional[str] = None  # set before rclpy init if provided

    # HTTP
    host: str = "0.0.0.0"
    port: int = 8085
    api_token: str = ""                  # empty = no auth (private tailnet use)
    cors_origins: List[str] = field(default_factory=lambda: ["*"])

    # Capture behaviour
    stale_after_sec: float = 5.0
    stale_policy: str = "warn"           # "warn" or "reject"

    # Rendering
    renderer: str = "auto"               # auto | open3d | matplotlib
    image_width: int = 1280
    image_height: int = 720
    background_color: str = "0.05,0.05,0.08"
    color_mode: str = "height"           # height | intensity
    up_axis: str = "z"                   # z (ROS/robotics) or y (aerospace)
    point_size: float = 3.0
    fov_deg: float = 60.0
    perspective_elev_deg: float = 45.0   # oblique view elevation (~45 deg)
    perspective_azim_deg: float = -60.0
    clip_range_m: float = 0.0            # radial clip from sensor origin; 0 = off

    # Resource bounds
    max_render_points: int = 300_000
    max_concurrent_renders: int = 1
    max_snapshots: int = 50              # bounded snapshot registry
    snapshot_retention_sec: float = 600.0

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, config_file: Optional[str] = None,
             cli_overrides: Optional[dict] = None) -> "Config":
        cfg = cls()
        path = config_file or os.environ.get(ENV_PREFIX + "CONFIG")
        if path:
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            _apply(cfg, data)
        env_data = {}
        for f in fields(cls):
            raw = os.environ.get(ENV_PREFIX + f.name.upper())
            if raw is not None:
                env_data[f.name] = raw
        if env_data:
            _apply(cfg, env_data)
        if cli_overrides:
            _apply(cfg, {k: v for k, v in cli_overrides.items() if v is not None})
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if self.up_axis not in ("z", "y"):
            raise ValueError(f"up_axis must be 'z' or 'y', got {self.up_axis!r}")
        if self.color_mode not in ("height", "intensity"):
            raise ValueError(f"color_mode must be 'height' or 'intensity', got {self.color_mode!r}")
        if self.renderer not in ("auto", "open3d", "matplotlib"):
            raise ValueError(f"renderer must be auto|open3d|matplotlib, got {self.renderer!r}")
        if self.stale_policy not in ("warn", "reject"):
            raise ValueError(f"stale_policy must be warn|reject, got {self.stale_policy!r}")
        if self.max_render_points < 1000:
            raise ValueError("max_render_points must be >= 1000")
        if self.clip_range_m < 0:
            raise ValueError("clip_range_m must be >= 0")
        parse_color(self.background_color)  # raises on bad value

    def background_rgb(self) -> tuple:
        return parse_color(self.background_color)


def parse_color(text: str) -> tuple:
    """Parse 'r,g,b' floats in 0..1."""
    try:
        parts = [float(p.strip()) for p in text.split(",")]
    except ValueError:
        raise ValueError(f"invalid color {text!r}, expected 'r,g,b' floats in 0..1")
    if len(parts) != 3 or any(p < 0.0 or p > 1.0 for p in parts):
        raise ValueError(f"invalid color {text!r}, expected 3 floats in 0..1")
    return tuple(parts)


def _apply(cfg: Config, data: dict) -> None:
    valid = {f.name: f.type for f in fields(Config)}
    for key, value in data.items():
        if key not in valid:
            raise ValueError(f"unknown config key: {key}")
        # env values arrive as strings; coerce to the dataclass field type
        current = getattr(cfg, key)
        if isinstance(current, bool):
            setattr(cfg, key, str(value).lower() in ("1", "true", "yes", "on"))
        elif isinstance(current, int) and not isinstance(current, bool):
            setattr(cfg, key, int(value))
        elif isinstance(current, float):
            setattr(cfg, key, float(value))
        elif isinstance(current, list):
            setattr(cfg, key, list(value))
        else:
            setattr(cfg, key, value)
