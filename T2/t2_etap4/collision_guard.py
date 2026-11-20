"""Проверка самоколлизий по модели Go1 и таблице пар из ТЗ T2."""

import csv
from math import ceil, radians
from pathlib import Path

MAX_SAMPLE_STEP = radians(0.1)
PENETRATION_LIMIT_M = 0.001
EXPECTED_CHECKED_PAIRS = 655


class CollisionGuard:
    def __init__(self, scene_path, pair_table, joint_names):
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if home_id < 0:
            raise ValueError("В модели Go1 отсутствует ключевой кадр home.")
        mujoco.mj_resetDataKeyframe(self.model, self.data, home_id)
        self.qpos_indices = [
            int(self.model.jnt_qposadr[self.model.joint(f"{name}_joint").id])
            for name in joint_names
        ]
        self.checked_pairs = self.load_checked_pairs(pair_table)

    def load_checked_pairs(self, pair_table):
        pairs = {}
        with Path(pair_table).open(encoding="utf-8", newline="") as source:
            for row in csv.DictReader(source):
                if row["diagnostic_policy"] != "checked":
                    continue
                first, second = int(row["geom1_id"]), int(row["geom2_id"])
                for index, geom_id in enumerate((first, second), 1):
                    if not 0 <= geom_id < self.model.ngeom:
                        raise ValueError(f"Геометрия {geom_id} из CSV отсутствует в модели.")
                    name = self.mujoco.mj_id2name(
                        self.model, self.mujoco.mjtObj.mjOBJ_GEOM, geom_id
                    ) or f"UNNAMED_geom_{geom_id}"
                    body_id = int(self.model.geom_bodyid[geom_id])
                    if name != row[f"geom{index}"] or body_id != int(row[f"body{index}_id"]):
                        raise ValueError(f"Геометрия {geom_id} в CSV не совпадает с моделью Go1.")

                pair = tuple(sorted((first, second)))
                if pair in pairs:
                    raise ValueError(f"Повтор пары {pair} в CSV.")
                pairs[pair] = (row["geom1"], row["geom2"])
        if len(pairs) != EXPECTED_CHECKED_PAIRS:
            raise ValueError(f"Ожидалось 655 проверяемых пар, загружено {len(pairs)}.")
        return pairs

    def pose_collision(self, angles):
        # Контакты вычисляются на отдельной копии состояния, без шага физики.
        self.data.qpos[self.qpos_indices] = angles
        self.mujoco.mj_kinematics(self.model, self.data)
        self.mujoco.mj_comPos(self.model, self.data)
        self.mujoco.mj_collision(self.model, self.data)
        for contact in self.data.contact:
            if contact.dist < -PENETRATION_LIMIT_M:
                pair = tuple(sorted((contact.geom1, contact.geom2)))
                names = self.checked_pairs.get(pair)
                if names is not None:
                    return names, -float(contact.dist)
        return None

    def transition_error(self, previous, target):
        if previous is None or previous == target:
            hit = self.pose_collision(target)
            return self.describe(hit, "целевая поза") if hit else None

        differences = tuple(end - start for start, end in zip(previous, target))
        segments = max(1, ceil(max(map(abs, differences)) / MAX_SAMPLE_STEP))
        for index in range(segments + 1):
            fraction = index / segments
            angles = target if index == segments else tuple(
                start + fraction * change for start, change in zip(previous, differences)
            )
            hit = self.pose_collision(angles)
            if hit:
                return self.describe(hit, f"{fraction:.1%} перехода")
        return None

    @staticmethod
    def describe(hit, position):
        (first, second), depth_m = hit
        return f"самоколлизия ({position}), {first} / {second}, {depth_m * 1000:.3f} мм"
