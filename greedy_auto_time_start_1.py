'''
module for base algorithm
'''
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
from build_matrices import build_matrices

SERVICE_TIME_BY_TYPE  = {
    "ATM": 13.5,
    "TC": 3.5,
    "TOBO": 3,
    "TT": 4,
}
BASE_TIME_TO_START = 540 #minutes, 9:00

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
    def service(self) -> int:
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

def auto_route_start(
    tw_nodes: List[int],
    day_stops: List[Point],
    times: List[List[Optional[float]]]):
    '''
    Choose time to start
    '''
    if not tw_nodes:
        return BASE_TIME_TO_START

    latest_allowed_start = None

    for node in tw_nodes:
        stop = day_stops[node - 1]
        travel_time = times[0][node]
        if travel_time is None:
            continue

        if stop.tw_start is not None:
            bound = stop.tw_start - travel_time
        elif stop.tw_end is not None:
            bound = stop.tw_end - travel_time - stop.service
        else:
            continue

        if latest_allowed_start is None or bound < latest_allowed_start:
            latest_allowed_start = bound

    if latest_allowed_start is None:
        return BASE_TIME_TO_START

    if BASE_TIME_TO_START <= latest_allowed_start:
        return BASE_TIME_TO_START

    return float(latest_allowed_start)

def nearest_points(
    current_node: int,
    current_time: float,
    tw_nodes: List[int],
    ntw_nodes: List[int],
    day_stops: List[Point],
    times: List[List[Optional[float]]],
    urgent_tw_count: int = 15,
    nearest_count: int = 35,
) -> List[int]:
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

def build_routes(
    day_stops: List[Point],
    depo: Point,
    dist_km: List[List[Optional[float]]],
    times: List[List[Optional[float]]],
    max_crews: int,
    max_workers: int,
    workers_per_crew: int,
    max_route_duration_min: int):
    '''
    Build the routes with auto-start and duration balancing (penalty logic).
    '''
    n_nodes = 1 + len(day_stops)
    if len(dist_km) != n_nodes or len(times) != n_nodes:
        raise ValueError("Size of matrix is wrong")

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
            day_stops[node - 1].tw_end if day_stops[node - 1].tw_end is not None else float('inf')
        )
    )

    routes = []
    used_workers = 0
    crew_counter = 1

    def open_new_crew():
        return crew_counter <= max_crews and (used_workers + workers_per_crew) <= max_workers

    while tw_nodes or ntw_nodes:
        if not open_new_crew():
            the_rest = [day_stops[i - 1].stop_id for i in (tw_nodes + ntw_nodes)]
            raise InfeasibleError(f"Infeasible: points left {the_rest}")

        used_workers += workers_per_crew

        route_start = auto_route_start(
            tw_nodes=tw_nodes,
            day_stops=day_stops,
            times=times
        )
        route_deadline = route_start + max_route_duration_min

        current_node = 0
        current_time = float(route_start)
        dist_total = 0.0
        served_count = 0
        visited_in_route = []

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

            best_node = None
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

                    time_score = travel_time + (wait * 0.8)
                    if stop_p.tw_end is not None:
                        urgency_bonus = (stop_p.tw_end - finish) / 10.0
                        time_score += urgency_bonus

                    score = (time_score, leg_dist)

                    if score < best_score:
                        best_score = score
                        best_node = point_i
                        best_finish = finish
                        best_dist = leg_dist
                        best_wait = wait

                if best_node is not None:
                    break

            if best_node is None:
                break

            if best_node in tw_nodes:
                tw_nodes.remove(best_node)
            else:
                ntw_nodes.remove(best_node)

            visited_in_route.append(best_node)
            dist_total += float(best_dist or 0.0)
            current_time = float(best_finish)
            current_node = best_node
            served_count += 1

        t_back = times[current_node][0]
        d_back = dist_km[current_node][0]
        end_time = current_time + float(t_back)
        dist_total += float(d_back)

        seq_nodes = [0] + visited_in_route + [0]
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
    cache_path: str):
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
        max_route_duration_min=max_route_duration_min)

    return routes
