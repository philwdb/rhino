from dataclasses import dataclass, field


@dataclass
class SimConfig:
    viewer: str = "none"  # "none" | "passive"
    start_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class RobotConfig:
    ip: str = "192.168.123.161"
    timeout: float = 10.0


@dataclass
class MapConfig:
    resolution: float = 0.1
    width: int = 200
    height: int = 200
    log_odds_hit: float = 0.85
    log_odds_miss: float = 0.4
    log_odds_min: float = 0.01
    log_odds_max: float = 0.99
    inflation_radius: float = 0.3


@dataclass
class NavConfig:
    replan_interval: float = 1.0
    arrival_tolerance: float = 0.25
    lookahead_distance: float = 0.5
    max_linear_vel: float = 0.4
    max_angular_vel: float = 0.8
    kp_linear: float = 1.0
    kp_angular: float = 2.0


@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8000
    mcp_port: int = 8001


@dataclass
class StorageConfig:
    db_path: str = "~/.local/state/rhino/rhino.db"


@dataclass
class RerunConfig:
    app_id: str = "rhino"
    connect: bool = False


@dataclass
class RhinoConfig:
    sim: bool = False
    sim_cfg: SimConfig = field(default_factory=SimConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    map: MapConfig = field(default_factory=MapConfig)
    nav: NavConfig = field(default_factory=NavConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    storage: StorageConfig = field(default_factory=StorageConfig)
    rerun: RerunConfig = field(default_factory=RerunConfig)
