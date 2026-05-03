from __future__ import annotations

import time
import uuid
from pathlib import Path

import aiosqlite

from rhino.config import StorageConfig
from rhino.platforms.base import POI


class Storage:
    def __init__(self, cfg: StorageConfig) -> None:
        self._path = Path(cfg.db_path).expanduser()
        self._db: aiosqlite.Connection | None = None

    async def init(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._db = await aiosqlite.connect(self._path)
        await self._db.execute(
            """CREATE TABLE IF NOT EXISTS pois (
                id TEXT PRIMARY KEY,
                label TEXT NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL DEFAULT 0,
                created_at REAL NOT NULL
            )"""
        )
        await self._db.commit()

    async def add_poi(self, label: str, x: float, y: float, z: float = 0.0) -> POI:
        poi = POI(
            id=str(uuid.uuid4()),
            label=label,
            x=x,
            y=y,
            z=z,
            created_at=time.time(),
        )
        assert self._db
        await self._db.execute(
            "INSERT INTO pois (id, label, x, y, z, created_at) VALUES (?,?,?,?,?,?)",
            (poi.id, poi.label, poi.x, poi.y, poi.z, poi.created_at),
        )
        await self._db.commit()
        return poi

    async def list_pois(self) -> list[POI]:
        assert self._db
        async with self._db.execute("SELECT id, label, x, y, z, created_at FROM pois") as cur:
            rows = await cur.fetchall()
        return [POI(id=r[0], label=r[1], x=r[2], y=r[3], z=r[4], created_at=r[5]) for r in rows]

    async def delete_poi(self, poi_id: str) -> bool:
        assert self._db
        cur = await self._db.execute("DELETE FROM pois WHERE id=?", (poi_id,))
        await self._db.commit()
        return cur.rowcount > 0

    async def close(self) -> None:
        if self._db:
            await self._db.close()
