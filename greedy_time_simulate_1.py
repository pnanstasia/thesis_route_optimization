'''
module for base algorithm and time simulation
'''

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
from build_matrices import build_matrices

SERVICE_TIME_BY_TYPE = {
    "ATM": 13.5,
    "TC": 3.5,
    "TOBO": 3,
    "TT": 4,
}

OSM_URL = "https://router.project-osrm.org"
OSM_MODE = "driving"
BLOCK_SIZE = 25
URGENT_TW = 15
NEAREST_POINRS = 35


class InfeasibleError(Exception):
    '''
    Raising the error when task has no solutions
    '''


@dataclass(frozen=True)
class Point:
    '''
    class for points of destination with their properties
    '''
    stop_id: str
    lat: float
    lon: float
    type: str
    tw_start: Optional[str] = None
    tw_end: Optional[str] = None

    @staticmethod
    def _time_to_minutes(value: str):
        s = value.strip()
        parts = s.split(":")
        if len(parts) not in (2, 3):
            raise ValueError(f"Invalid time value '{value}'. Use format like 'HH:MM' or 'HH:MM:SS'")
        try:
            hour = int(parts[0])
            minute = int(parts[1])
            second = int(parts[2]) if len(parts) == 3 else 0
        except ValueError as e:
            raise ValueError(f"Invalid input of the time '{value}'. Use numbers") from e
        if not (0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 59):
            raise ValueError(f"Time input is out of range - {value}")
        return hour * 60 + minute

    def __post_init__(self) -> None:
        if self.tw_start is not None:
            object.__setattr__(self, "tw_start", self._time_to_minutes(self.tw_start))
        if self.tw_end is not None:
            object.__setattr__(self, "tw_end", self._time_to_minutes(self.tw_end))

    @property
    def service(self):
        '''
        function for setting service time
        '''
        if self.stop_id == "DEPOT":
            return 0
        if self.type not in SERVICE_TIME_BY_TYPE:
            raise ValueError(f"Unknown point type {self.type} for item {self.stop_id}")
        return SERVICE_TIME_BY_TYPE[self.type]


def minutes_formating(time_mins):
    '''
    Convert time to format HH:MM:SS
    '''
    total_seconds = int(round(time_mins * 60))

    h = (total_seconds // 3600) % 24
    m = (total_seconds % 3600) // 60
    s = total_seconds % 60

    return f"{h:02d}:{m:02d}:{s:02d}"

@dataclass
class Route:
    '''
    class for route identification
    '''
    crew_id: int
    stop_sequence: List[str]
    start_time: float
    end_time: float
    distance_km: float
    workers_used: int
    served_count: int

    @property
    def start_hms(self):
        '''
        turn start time into hours:minutes format
        '''
        return minutes_formating(self.start_time)

    @property
    def end_hms(self):
        '''
        turn end time into hours:minutes format
        '''
        return minutes_formating(self.end_time)

def _feasible_next(
    current_node: int,
    candidate_node: int,
    current_time: float,
    route_deadline: int,
    stop: Point,
    dist_km: List[List[Optional[float]]],
    time_min: List[List[Optional[float]]]):
    '''
    Checking the next node whether it possible to visit or not
    '''
    time_tr = time_min[current_node][candidate_node]
    dist = dist_km[current_node][candidate_node]
    if time_tr is None or dist is None:
        return False, 0, 0, 0.0, 0

    arrival = current_time + float(time_tr)

    start_service = arrival
    wait = 0

    if stop.tw_start is not None:
        start_service = max(arrival, stop.tw_start)
        wait = max(0, start_service - arrival)

    if stop.tw_end is not None and start_service > stop.tw_end:
        return False, 0, arrival, float(dist), wait

    finish = start_service + stop.service

    t_back = time_min[candidate_node][0]
    if t_back is None:
        return False, 0, arrival, float(dist), wait

    if (finish + int(round(t_back))) > route_deadline:
        return False, 0, arrival, float(dist), wait

    return True, finish, arrival, float(dist), wait

def _iter_start_candidates(start_from: str, start_to: str, step_min: int):
    '''
    transformation of time range into minutes
    '''
    t0 = Point._time_to_minutes(start_from)
    t1 = Point._time_to_minutes(start_to)
    return list(range(t0, t1 + 1, step_min))



def nearest_points(
    current_node: int,
    current_time: float,
    tw_nodes: List[int],
    ntw_nodes: List[int],
    day_stops: List[Point],
    times: List[List[Optional[float]]],
    urgent_tw_count: int = 15,
    nearest_count: int = 35):
    '''
    Function to find the nearest candidates for point
    '''

    selected: List[int] = []
    selected_set = set()

    urgent_tw = []
    for node in tw_nodes:
        travel_time = times[current_node][node]
        if travel_time is None:
            continue

        stop = day_stops[node - 1]
        arrival = current_time + float(travel_time)

        slack = float("inf")
        if stop.tw_end is not None:
            slack = stop.tw_end - arrival

        urgent_tw.append((slack, travel_time, node))

    urgent_tw.sort(key=lambda x: (x[0], x[1]))

    for _, _, node in urgent_tw[:urgent_tw_count]:
        selected.append(node)
        selected_set.add(node)

    remaining_ranked = []
    for node in tw_nodes + ntw_nodes:
        if node in selected_set:
            continue

        travel_time = times[current_node][node]
        if travel_time is None:
            continue

        remaining_ranked.append((float(travel_time), node))

    remaining_ranked.sort(key=lambda x: x[0])

    for _, node in remaining_ranked[:nearest_count]:
        selected.append(node)
        selected_set.add(node)

    return selected

def _simulate_one_route(
    route_start: int,
    tw_nodes_in: List[int],
    ntw_nodes_in: List[int],
    day_stops: List[Point],
    dist_km: List[List[Optional[float]]],
    times: List[List[Optional[float]]],
    max_route_duration_min: int):
    '''
    Build one route
    '''
    route_deadline = route_start + max_route_duration_min
    tw_nodes = tw_nodes_in.copy()
    ntw_nodes = ntw_nodes_in.copy()

    current_node = 0
    current_time = float(route_start)
    dist_total = 0.0
    served_count = 0
    visited = []
    total_wait = 0.0

    while True:
        combined_pool = tw_nodes + ntw_nodes
        if not combined_pool:
            break

        candidate_pool = nearest_points(
            current_node=current_node,
            current_time=current_time,
            tw_nodes=tw_nodes,
            ntw_nodes=ntw_nodes,
            day_stops=day_stops,
            times=times,
            urgent_tw_count=URGENT_TW,
            nearest_count=NEAREST_POINRS,
        )

        search_pools = []
        if candidate_pool:
            search_pools.append(candidate_pool)
        if candidate_pool != combined_pool:
            search_pools.append(combined_pool)

        best_idx_in_pool = None
        best_score = (float("inf"), float("inf"))
        best_finish = None
        best_dist = None
        best_wait = None

        for pool in search_pools:
            for point_i in pool:
                stop_p = day_stops[point_i - 1]

                feasible, finish, _, leg_dist, wait = _feasible_next(
                    current_node=current_node,
                    candidate_node=point_i,
                    current_time=current_time,
                    route_deadline=route_deadline,
                    stop=stop_p,
                    dist_km=dist_km,
                    time_min=times,
                )

                if not feasible:
                    continue

                travel_time = times[current_node][point_i]
                if travel_time is None:
                    continue

                time_score = float(travel_time) + (float(wait) * 0.8)

                if stop_p.tw_end is not None:
                    urgency_bonus = (stop_p.tw_end - finish) / 10.0
                    time_score += urgency_bonus

                score = (time_score, float(leg_dist))

                if score < best_score:
                    best_score = score
                    best_idx_in_pool = point_i
                    best_finish = finish
                    best_dist = leg_dist
                    best_wait = wait

            if best_idx_in_pool is not None:
                break

        if best_idx_in_pool is None:
            break

        if best_idx_in_pool in tw_nodes:
            tw_nodes.remove(best_idx_in_pool)
        else:
            ntw_nodes.remove(best_idx_in_pool)

        visited.append(best_idx_in_pool)

        dist_total += float(best_dist or 0.0)
        current_time = float(best_finish)
        current_node = best_idx_in_pool
        served_count += 1
        total_wait += float(best_wait or 0.0)

    t_back = times[current_node][0]
    d_back = dist_km[current_node][0]
    if t_back is None or d_back is None:
        return 0, float("inf"), route_start, [], 0

    end_time = current_time + float(round(t_back))
    dist_total += float(d_back)

    if end_time > route_deadline:
        return 0, float("inf"), end_time, [], total_wait

    return served_count, dist_total, end_time, visited, total_wait

def build_routes(
    day_stops: List[Point],
    depo: Point,
    dist_km: List[List[Optional[float]]],
    times: List[List[Optional[float]]],
    max_crews: int,
    max_workers: int,
    workers_per_crew: int,
    max_route_duration_min: int,
    start_from: str,
    start_to: str,
    step_min: int,
    target_duration: int):
    '''
    Build the routes
    '''
    n_nodes = 1 + len(day_stops)
    if len(dist_km) != n_nodes or len(times) != n_nodes:
        raise ValueError("size of matrix is wrong")

    tw_nodes = []
    ntw_nodes = []

    for i, s in enumerate(day_stops, start=1):
        if (s.tw_start is not None) or (s.tw_end is not None):
            tw_nodes.append(i)
        else:
            ntw_nodes.append(i)

    tw_nodes.sort(
        key=lambda node: (
        day_stops[node - 1].tw_start if day_stops[node - 1].tw_start is not None else 0,
        day_stops[node - 1].tw_end if day_stops[node - 1].tw_end is not None else float('inf'))
    )

    routes = []
    used_workers = 0
    crew_counter = 1

    def open_new_crew():
        return crew_counter <= max_crews and (used_workers + workers_per_crew) <= max_workers

    start_range= _iter_start_candidates(start_from, start_to, step_min)

    while tw_nodes or ntw_nodes:
        if not open_new_crew():
            the_rest = [day_stops[i - 1].stop_id for i in (tw_nodes + ntw_nodes)]
            raise InfeasibleError(f"Infeasible solution, there are points {the_rest}")

        best = None
        for s in start_range:
            served, km, end_t, visited, total_wait = _simulate_one_route(
                route_start=s,
                tw_nodes_in=tw_nodes,
                ntw_nodes_in=ntw_nodes,
                day_stops=day_stops,
                dist_km=dist_km,
                times=times,
                max_route_duration_min=max_route_duration_min)
            if served == 0:
                continue

            actual_duration = end_t - s
            balance_penalty = abs(actual_duration - target_duration)

            score = (balance_penalty, -served, km, total_wait)

            if best is None or score < best["score"]:
                best = {
                    "start": s,
                    "served": served,
                    "km": km,
                    "end": end_t,
                    "visited": visited,
                    "wait": total_wait,
                    "score": score,
                }

        if best is None:
            the_rest = [day_stops[i - 1].stop_id for i in (tw_nodes + ntw_nodes)]
            raise InfeasibleError(f"Infeasible solution, there are points {the_rest}")

        used_workers += workers_per_crew
        route_start = best["start"]
        end_time = best["end"]
        dist_total = best["km"]
        visited = best["visited"]
        served_count = best["served"]

        for node in visited:
            if node in tw_nodes:
                tw_nodes.remove(node)
            elif node in ntw_nodes:
                ntw_nodes.remove(node)

        seq_nodes = [0] + visited + [0]
        stop_sequence = [depo.stop_id if n == 0 else day_stops[n - 1].stop_id for n in seq_nodes]

        routes.append(Route(
            crew_id=crew_counter,
            stop_sequence=stop_sequence,
            start_time=route_start,
            end_time=end_time,
            distance_km=dist_total,
            workers_used=workers_per_crew,
            served_count=served_count
        ))
        crew_counter += 1

        if served_count == 0:
            the_rest = [day_stops[i - 1].stop_id for i in (tw_nodes + ntw_nodes)]
            raise InfeasibleError(f"Infeasible solution, there are points {the_rest}")

    return routes


def plan_routes_per_day(
    depo: Point,
    day_stops: List[Point],
    name_mod: str,
    max_crews: int,
    max_workers: int,
    workers_per_crew: int,
    max_route_duration_min: int,
    cache_path: str,
    start_from: str,
    start_to: str,
    step_min: int,
    target_duration: int):
    '''
    main function to get routes per day
    '''
    coords = [(depo.lat, depo.lon)] + [(s.lat, s.lon) for s in day_stops]

    dist_km, time_min = build_matrices(
        coords=coords,
        cache_path=cache_path,
        name=name_mod,
        block_size=BLOCK_SIZE,
        osm_url=OSM_URL,
        osm_mode=OSM_MODE

    )

    routes = build_routes(
        day_stops=day_stops,
        depo=depo,
        dist_km=dist_km,
        times=time_min,
        max_crews=max_crews,
        max_workers=max_workers,
        workers_per_crew=workers_per_crew,
        max_route_duration_min=max_route_duration_min,
        start_from=start_from,
        start_to=start_to,
        step_min=step_min,
        target_duration=target_duration
    )
    return routes

# if __name__ == "__main__":
#     depo = Point("DEPOT", 47.82709400, 31.16899100, type="DEPOT")

#     day_stops = []

#     max_crews = 2
#     workers_per_crew = 3
#     max_workers = 6

#     try:
#         routes = plan_routes_per_day(
#             depo=depo,
#             day_stops=day_stops,
#             max_crews=max_crews,
#             max_workers=max_workers,
#             workers_per_crew=workers_per_crew,
#             max_route_duration_min=12 * 60,
#             cache_path=CACHE_PATH,
#             start_from="08:00",
#             start_to="10:00",
#             step_min=10,
#             target_duration=8*60
#         )

#         print(f"Built {len(routes)} routes:")
#         for r in routes:
#             print(
#                 f"Crew {r.crew_id}: {r.stop_sequence} | "
#                 f"start={r.start_hms} end={r.end_hms} | "
#                 f"km={r.distance_km:.2f} | workers={r.workers_used} | served={r.served_count}"
#             )

#     except InfeasibleError as e:
#         print(e)

#     visualize_routes(routes, depo, day_stops)
