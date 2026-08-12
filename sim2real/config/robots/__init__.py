from sim2real.config.robots.base import RobotCfg
from sim2real.config.robots.g1 import G1_CFG
from sim2real.config.robots.piplus import PIPLUS_H0W_CFG
from typing import Dict


_ROBOT_CFGS: Dict[str, RobotCfg] = {
    G1_CFG.name: G1_CFG,
    PIPLUS_H0W_CFG.name: PIPLUS_H0W_CFG,
}


def get_robot_cfg(name: str) -> RobotCfg:
    key = str(name).strip().lower()
    try:
        return _ROBOT_CFGS[key]
    except KeyError as exc:
        available = ", ".join(sorted(_ROBOT_CFGS))
        raise ValueError(f"Unknown robot '{name}'. Available robots: {available}") from exc


__all__ = ["RobotCfg", "G1_CFG", "PIPLUS_H0W_CFG", "get_robot_cfg"]
