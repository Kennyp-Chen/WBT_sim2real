from sim2real.config.robots.base import RobotCfg
from sim2real.config.robots.g1 import G1_CFG
from sim2real.config.robots.piplus import PIPLUS_H0W_CFG
from sim2real.config.robots.ht_bfmzero import (
    HI_25DOF_CFG,
    PIPLUS_LSE_23DOF_CFG,
)
from typing import Dict


_ROBOT_CFGS: Dict[str, RobotCfg] = {
    G1_CFG.name: G1_CFG,
    PIPLUS_H0W_CFG.name: PIPLUS_H0W_CFG,
    PIPLUS_LSE_23DOF_CFG.name: PIPLUS_LSE_23DOF_CFG,
    HI_25DOF_CFG.name: HI_25DOF_CFG,
}

_ROBOT_ALIASES = {
    "ht_piplus22dof": PIPLUS_H0W_CFG.name,
    "piplus22dof": PIPLUS_H0W_CFG.name,
    "ht_piplus_h0w": PIPLUS_H0W_CFG.name,
    "ht_piplus23dof": PIPLUS_LSE_23DOF_CFG.name,
    "piplus23dof": PIPLUS_LSE_23DOF_CFG.name,
    "ht_piplus_lse": PIPLUS_LSE_23DOF_CFG.name,
    "ht_hi25dof": HI_25DOF_CFG.name,
    "hi25dof": HI_25DOF_CFG.name,
    "ht_hi": HI_25DOF_CFG.name,
}


def get_robot_cfg(name: str) -> RobotCfg:
    key = str(name).strip().lower()
    key = _ROBOT_ALIASES.get(key, key)
    try:
        return _ROBOT_CFGS[key]
    except KeyError as exc:
        available = ", ".join(sorted(_ROBOT_CFGS))
        raise ValueError(f"Unknown robot '{name}'. Available robots: {available}") from exc


__all__ = [
    "RobotCfg", "G1_CFG", "PIPLUS_H0W_CFG", "PIPLUS_LSE_23DOF_CFG",
    "HI_25DOF_CFG", "get_robot_cfg",
]
