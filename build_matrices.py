'''
module to build matrices of time and distance based on the Open street map requests
'''

import json
import os
import time
from typing import Any, Dict, List, Tuple

import requests
from requests.exceptions import RequestException

OSM_URL = "https://router.project-osrm.org"
OSM_MODE = "driving"
DEFAULT_BLOCK_SIZE = 25

def _load_cache(cache_path: str):
    '''
    load cache from JSON file if it exists
    '''
    if os.path.exists(cache_path):
        with open(cache_path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def _save_cache(cache_path: str, cache: Dict[str, Any]):
    '''
    save cache to JSON file
    '''
    with open(cache_path, "w", encoding="utf-8") as f:
        json.dump(cache, f, ensure_ascii=False)


def _build_cache_key(
    name: str,
    len_coords: int,
    osm_mode: str) -> str:
    '''
    construct the cache key
    '''
    return f"matrix_{name}_n{len_coords}_{osm_mode}"

def _osrm_table_request(
    coords: List[Tuple[float, float]],
    src_indices: List[int],
    dst_indices: List[int],
    osm_url: str,
    osm_mode: str,
    retries: int = 5,
    backoff: int = 2):
    '''
    requesrt to Open Street map to get dist and time
    '''
    unique_indices = []
    seen = set()

    for idx in list(src_indices) + list(dst_indices):
        if idx not in seen:
            seen.add(idx)
            unique_indices.append(idx)

    sub_coords = [coords[idx] for idx in unique_indices]
    local_index = {
        global_idx: local_idx for local_idx, global_idx in enumerate(unique_indices)
    }

    coord_str = ";".join(f"{lon},{lat}" for (lat, lon) in sub_coords)
    url = f"{osm_url}/table/v1/{osm_mode}/{coord_str}"

    params = {
        "annotations": "duration,distance",
        "sources": ";".join(str(local_index[i]) for i in src_indices),
        "destinations": ";".join(str(local_index[j]) for j in dst_indices),
    }

    for attempt in range(1, retries + 1):
        try:
            response = requests.get(url, params=params, timeout=60)
            response.raise_for_status()
            data = response.json()

            if "durations" not in data or "distances" not in data:
                raise ValueError(
                    f"OSRM response does not contain 'durations' or 'distances': {data}"
                )

            return data

        except RequestException as e:
            if attempt < retries:
                time.sleep(backoff ** (attempt - 1))
            else:
                raise RuntimeError(
                    f"OSRM request failed after {retries} attempts"
                ) from e

def build_matrices(
    coords: List[Tuple[float, float]],
    cache_path: str,
    name: str,
    block_size: int = DEFAULT_BLOCK_SIZE,
    osm_url: str = OSM_URL,
    osm_mode: str = OSM_MODE):
    '''
    Build matrices for dist and time
    '''
    n = len(coords)
    cache = _load_cache(cache_path)

    cache_key = _build_cache_key(name=name,
        len_coords=n,
        osm_mode=osm_mode)

    if cache_key in cache:
        data = cache[cache_key]
        return data["dist_km"], data["time_min"]

    dist_km = [[0.0] * n for _ in range(n)]
    time_min = [[0.0] * n for _ in range(n)]

    for src_start in range(0, n, block_size):
        src_end = min(src_start + block_size, n)
        src_idx = list(range(src_start, src_end))

        for dst_start in range(0, n, block_size):
            dst_end = min(dst_start + block_size, n)
            dst_idx = list(range(dst_start, dst_end))

            if len(src_idx) == 1 and len(dst_idx) == 1 and src_idx[0] == dst_idx[0]:
                dist_km[src_idx[0]][dst_idx[0]] = 0.0
                time_min[src_idx[0]][dst_idx[0]] = 0.0
                continue

            data_block = _osrm_table_request(
                coords=coords,
                src_indices=src_idx,
                dst_indices=dst_idx,
                osm_url=osm_url,
                osm_mode=osm_mode,
            )

            durations = data_block["durations"]
            distances = data_block["distances"]

            for local_i, global_i in enumerate(src_idx):
                for local_j, global_j in enumerate(dst_idx):
                    d = distances[local_i][local_j]
                    t = durations[local_i][local_j]

                    dist_km[global_i][global_j] = (d / 1000.0) if d is not None else None
                    time_min[global_i][global_j] = (t / 60.0) if t is not None else None

    cache[cache_key] = {
        "dist_km": dist_km,
        "time_min": time_min
    }
    _save_cache(cache_path, cache)

    return dist_km, time_min
