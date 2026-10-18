from dataclasses import dataclass
import json
from typing import Dict, List, Optional, Tuple


# =====================================================================
# DATOVÝ MODEL CESTUJÍCÍHO
# =====================================================================
@dataclass
class Passenger:
    """Třída reprezentující jednoho cestujícího v simulátoru.

    Uchovává informace o příchodu, počátečním a cílovém patře, faktoru
    pomalosti (např. imobilita/zavazadla) a naměřených simulačních časech.
    """

    id: int  # Unikátní identifikátor cestujícího
    t_arrival: float  # Čas příchodu na chodbu (v sekundách)
    from_floor: int  # Počáteční patro (odkud chce jet)
    to_floor: int  # Cílové patro (kam chce jet)
    slow: float = (
        1.0  # Koeficient pomalosti nástupu/výstupu (1.0 = standard)
    )
    board_time: float = 0.0  # Absolutní simulační čas nástupu do kabiny
    alight_time: float = 0.0  # Absolutní simulační čas výstupu z kabiny

    @property
    def direction(self) -> int:
        """Určuje požadovaný směr jízdy cestujícího.

        :return: 1 pro směr NAHORU (UP), -1 pro směr DOLŮ (DOWN), 0 pro stejné
        patro.
        """
        if self.to_floor > self.from_floor:
            return 1
        elif self.to_floor < self.from_floor:
            return -1
        return 0

    @property
    def wait_time(self) -> float:
        """Spočítá celkovou dobu čekání cestujícího na chodbe (od příchodu po
        nástup)."""
        return self.board_time - self.t_arrival if self.board_time > 0 else 0.0

    @property
    def travel_time(self) -> float:
        """Spočítá celkovou dobu jízdy v kabině (od nástupu po výstup)."""
        return (
            self.alight_time - self.board_time if self.alight_time > 0 else 0.0
        )


# =====================================================================
# 1. OPTIMALIZOVANÝ CHYTRÝ ALGORITMUS (LOOK / SCAN + ANTI-STARVATION)
# =====================================================================
class OptimizedLookElevatorController:
    """Pokročilý řídicí algoritmus výtahu založený na principu LOOK (SCAN) s

    integrovaným mechanismem Anti-Starvation proti hladovění.

    Pravidla řízení:
    1. Drží směr jízdy, dokud existují požadavky ve stávajícím směru (uvnitř
    kabiny nebo před výtahem).
    2. Na patře slučuje výstup i nástup do jednoho cyklu dveří (jediná
    zastávka).
    3. Pokud cestující čeká déle než `starvation_threshold`, výtah po
    vyprázdnění kabiny
       vynutí přímou jízdu k tomuto čekajícímu cestujícímu.
    """

    def __init__(
        self,
        floors: int = 8,
        capacity: int = 6,
        starvation_threshold: float = 200.0,
    ):
        """
        :param floors: Celkový počet pater v budově (0 až floors - 1)
        :param capacity: Maximální kapacita osob v kabině výtahu
        :param starvation_threshold: Maximální akceptovatelná doba čekání (s) před spuštěním priority
        """
        self.floors = floors
        self.capacity = capacity
        self.starvation_threshold = starvation_threshold

    def get_boardable_passengers(
        self,
        current_floor: int,
        current_direction: int,
        onboard: List[Passenger],
        waiting_queue: List[Passenger],
        has_requests_ahead: bool,
    ) -> Tuple[List[Passenger], int]:
        """Určí cestující, kteří mohou na daném patře v tomto okamžiku
        nastoupit,

        a vypočítá navazující směr jízdy výtahu.

        :param current_floor: Aktuální patro výtahu
        :param current_direction: Aktuální směr (1 = NAHORU, -1 = DOLŮ, 0 =
        IDLE)
        :param onboard: Seznam cestujících aktuálně v kabině
        :param waiting_queue: Seznam všech čekajících na chodbách budovy
        :param has_requests_ahead: Příznak, zda existují další požadavky ve
        směru jízdy
        :return: Tuple (seznam nastupujících pasažérů, nový směr jízdy)
        """
        # Kontrola volného místa v kabině
        available_cap = self.capacity - len(onboard)
        if available_cap <= 0:
            return [], current_direction

        # Filtrujeme lidi stojící přímo na tomto patře
        waiting_here = [
            p for p in waiting_queue if p.from_floor == current_floor
        ]
        if not waiting_here:
            return [], current_direction

        # A) Pokud výtah stál (IDLE), první čekající určuje nový směr
        if current_direction == 0:
            target_dir = waiting_here[0].direction
            boarders = [p for p in waiting_here if p.direction == target_dir][
                :available_cap
            ]
            return boarders, target_dir

        # B) Pokud jsou v kabině cestující, přibíráme jen ty, co jedou ve STEJNÉM směru
        if len(onboard) > 0:
            boarders = [
                p for p in waiting_here if p.direction == current_direction
            ][:available_cap]
            return boarders, current_direction

        # C) Pokud je kabina PRÁZDNÁ, zkontrolujeme nejprve lidi ve stejném směru
        same_dir_boarders = [
            p for p in waiting_here if p.direction == current_direction
        ][:available_cap]
        if same_dir_boarders:
            return same_dir_boarders, current_direction

        # D) Pokud na patře všichni jedou opačným směrem, nastoupit smí jen tehdy,
        # pokud před výtahem ve stávajícím směru už NENÍ žádný jiný požadavek.
        if not has_requests_ahead:
            opp_dir = -current_direction
            opp_boarders = [p for p in waiting_here if p.direction == opp_dir][
                :available_cap
            ]
            if opp_boarders:
                return opp_boarders, opp_dir

        return [], current_direction

    def _has_requests_ahead(
        self,
        current_floor: int,
        direction: int,
        onboard: List[Passenger],
        waiting: List[Passenger],
    ) -> bool:
        """Pomocná metoda: Ověří, zda ve směru 'direction' existuje jakýkoliv

        další požadavek na výstup nebo nástup.
        """
        if direction == 1:
            # Výstupy lidí z kabiny v vyšších patrech
            if any(p.to_floor > current_floor for p in onboard):
                return True
            # Nástupy lidí na chodbách ve vyšších patrech (pokud máme kapacitu)
            if len(onboard) < self.capacity and any(
                p.from_floor > current_floor for p in waiting
            ):
                return True
        elif direction == -1:
            # Výstupy lidí z kabiny v nižších patrech
            if any(p.to_floor < current_floor for p in onboard):
                return True
            # Nástupy lidí na chodbách v nižších patrech (pokud máme kapacitu)
            if len(onboard) < self.capacity and any(
                p.from_floor < current_floor for p in waiting
            ):
                return True
        return False

    def decide(
        self,
        current_floor: int,
        current_direction: int,
        onboard: List[Passenger],
        waiting: List[Passenger],
        current_time: float,
    ) -> Tuple[Optional[int], int]:
        """Hlavní rozhodovací funkce algoritmu volaná v každém rozhodovacím

        kroku.

        :return: Tuple (target_floor, new_direction). Pokud je výtah v klidu,
        vrací (None, 0).
        """
        has_requests_ahead = self._has_requests_ahead(
            current_floor, current_direction, onboard, waiting
        )

        # 1. Kontrola, zda někdo z kabiny vystupuje na tomto patře
        can_alight = any(p.to_floor == current_floor for p in onboard)

        # 2. Kontrola, zda může někdo platně nastoupit na tomto patře
        boarders, new_dir = self.get_boardable_passengers(
            current_floor,
            current_direction,
            onboard,
            waiting,
            has_requests_ahead,
        )

        # Pokud probíhá výstup nebo platný nástup -> cíl je AKTUÁLNÍ patro (zastavujeme)
        if can_alight or len(boarders) > 0:
            return current_floor, new_dir

        # 3. ANTI-STARVATION MECHANISMUS (Ochrana před hladověním)
        # Pokud je kabina prázdná, zkontrolujeme, zda někdo nečeká déle než limit
        if not onboard and waiting:
            starving_passengers = [
                p
                for p in waiting
                if (current_time - p.t_arrival) >= self.starvation_threshold
            ]
            if starving_passengers:
                # Vybereme nejdéle čekajícího cestujícího a vyrazíme přímo k němu
                oldest = max(
                    starving_passengers,
                    key=lambda p: current_time - p.t_arrival,
                )
                if oldest.from_floor != current_floor:
                    target_dir = 1 if oldest.from_floor > current_floor else -1
                    return oldest.from_floor, target_dir

        # 4. Standardní LOOK logika: Vyhledání nejbližšího cíle ve stávajícím směru
        active_dir = current_direction if current_direction != 0 else 1
        target = self._get_next_target(
            current_floor, active_dir, onboard, waiting
        )

        if target is not None:
            return target, active_dir

        # 5. Otočení směru jízdy: Pokud ve stávajícím směru nic není, zkusíme opačný směr
        opp_dir = -active_dir
        target = self._get_next_target(
            current_floor, opp_dir, onboard, waiting
        )

        if target is not None:
            return target, opp_dir

        # 6. Žádné požadavky v celé budově -> stav nečinnosti (IDLE)
        return None, 0

    def _get_next_target(
        self,
        current_floor: int,
        direction: int,
        onboard: List[Passenger],
        waiting: List[Passenger],
    ) -> Optional[int]:
        """Vyhledá nejbližší cílové patro před výtahem ve zadaném směru."""
        candidates = set()

        # Přidáme patra pro výstup pasažérů uvnitř kabiny
        for p in onboard:
            if (direction == 1 and p.to_floor > current_floor) or (
                direction == -1 and p.to_floor < current_floor
            ):
                candidates.add(p.to_floor)

        # Přidáme patra pro nástup čekajících (pouze pokud máme volnou kapacitu)
        if len(onboard) < self.capacity:
            for p in waiting:
                same_dir = (p.direction == direction) or (len(onboard) == 0)
                if same_dir:
                    if (direction == 1 and p.from_floor > current_floor) or (
                        direction == -1 and p.from_floor < current_floor
                    ):
                        candidates.add(p.from_floor)

        if not candidates:
            return None

        # Vracet vždy nejbližší patro ve směru jízdy (min pro UP, max pro DOWN)
        return min(candidates) if direction == 1 else max(candidates)


# =====================================================================
# 2. HLOU PÝ ALGORITMUS (FIFO / TAXÍK BEZ MEZIZASTÁVEK PRO POROVNÁNÍ)
# =====================================================================
class DumbElevatorController:
    """Srovnávací algoritmus (FIFO / Point-to-Point).

    Chová se jako jednoduché taxi:
    1. Obsluhuje požadavky striktně v pořadí příchodu (First-In, First-Out).
    2. Po cestě NENABÍRÁ žádné další lidi v mezipatřích.
    3. Teprve po doručení cestujícího do cíle jede pro dalšího v pořadí.
    """

    def __init__(
        self,
        floors: int = 8,
        capacity: int = 6,
        starvation_threshold: float = 0,
    ):
        self.floors = floors
        self.capacity = capacity

    def decide(
        self,
        current_floor: int,
        current_direction: int,
        onboard: List[Passenger],
        waiting: List[Passenger],
        current_time: float,
    ) -> Tuple[Optional[int], int]:
        # Pokud je v kabině cestující, jedeme přímo do jeho cíle
        if onboard:
            target_floor = onboard[0].to_floor
            if target_floor == current_floor:
                return current_floor, current_direction
            return target_floor, (1 if target_floor > current_floor else -1)

        # Pokud je kabina prázdná, jedeme přímo k prvnímu čekajícímu ve frontě
        if waiting:
            target_floor = waiting[0].from_floor
            if target_floor == current_floor:
                return current_floor, waiting[0].direction
            return target_floor, (1 if target_floor > current_floor else -1)

        return None, 0


# =====================================================================
# 3. DISKRÉTNÍ SIMULAČNÍ MOTOR (ELEVATOR SIMULATOR)
# =====================================================================
class ElevatorSimulator:
    """Simulační motor spravující časový průběh, pohyb výtahu a fyzikální

    zpoždění.
    """

    def __init__(self, data_json: dict, controller_class):
        lift_cfg = data_json.get("lift", {})
        self.floors = lift_cfg.get("floors", 8)
        self.capacity = lift_cfg.get("capacity", 6)

        # Fyzikální parametry výtahu
        speed = lift_cfg.get("speed", 2.0)  # Rychlost (m/s)
        height = lift_cfg.get("floorHeight", 3.0)  # Výška jednoho patra (m)
        self.travel_time_per_floor = height / speed  # Čas přejezdu 1 patra (s)
        self.door_time = lift_cfg.get("doorOpen", 1.5) + lift_cfg.get(
            "doorClose", 1.5
        )  # Čas otevření + zavření dveří (s)
        self.board_delay = data_json.get("boarding", {}).get(
            "in", 1.0
        )  # Zpoždění na 1 nastupující osobu (s)
        self.alight_delay = data_json.get("boarding", {}).get(
            "out", 1.0
        )  # Zpoždění na 1 vystupující osobu (s)

        # Instance vybraného řídicího algoritmu
        self.controller = controller_class(self.floors, self.capacity)

        # Načtení a inicializace všech pasažérů ze simulovaného scenáře
        self.all_passengers = [
            Passenger(
                id=p["id"],
                t_arrival=float(p["t"]),
                from_floor=p["from"],
                to_floor=p["to"],
                slow=float(p.get("slow", 1.0)),
            )
            for p in data_json.get("people", [])
        ]

    def run(self) -> Dict[str, float]:
        """Spustí diskrétní simulaci a vrátí naměřené metriky."""
        current_time = 0.0
        current_floor = 0
        current_direction = 0

        onboard: List[Passenger] = []
        waiting_queue: List[Passenger] = []
        unspawned = sorted(self.all_passengers, key=lambda p: p.t_arrival)
        finished: List[Passenger] = []

        stops_count = 0
        total_floors_traversed = 0

        # Hlavní simulační smyčka kognitivního času
        while unspawned or waiting_queue or onboard:
            # 1. Přunou se všichni cestující, kteří přišli do času current_time
            while unspawned and unspawned[0].t_arrival <= current_time:
                waiting_queue.append(unspawned.pop(0))

            # Pokud nikdo nečeká ani nejde a unspawned není prázdné, skočíme v čase k dalšímu příchodu
            if not waiting_queue and not onboard and unspawned:
                current_time = unspawned[0].t_arrival
                continue

            # 2. Algoritmus rozhodne o cílovém patře a novém směru
            target_floor, next_dir = self.controller.decide(
                current_floor,
                current_direction,
                onboard,
                waiting_queue,
                current_time,
            )

            if target_floor is None:
                if unspawned:
                    current_time = unspawned[0].t_arrival
                continue

            # 3. ZASTÁVKA NA PATŘE (Atomický cyklus výstupu a nástupu)
            if target_floor == current_floor:
                stops_count += 1
                current_direction = next_dir

                # A) Výstup cestujících z kabiny
                alighting = [p for p in onboard if p.to_floor == current_floor]
                for p in alighting:
                    p.alight_time = current_time
                    onboard.remove(p)
                    finished.append(p)

                # B) Nástup cestujících z chodby
                boarding = []
                if isinstance(
                    self.controller, OptimizedLookElevatorController
                ):
                    has_ahead = self.controller._has_requests_ahead(
                        current_floor,
                        current_direction,
                        onboard,
                        waiting_queue,
                    )
                    boarding, current_direction = (
                        self.controller.get_boardable_passengers(
                            current_floor,
                            current_direction,
                            onboard,
                            waiting_queue,
                            has_ahead,
                        )
                    )
                else:  # Základní FIFO logika
                    for p in list(waiting_queue):
                        if (
                            p.from_floor == current_floor
                            and len(onboard) + len(boarding) < self.capacity
                        ):
                            boarding.append(p)

                for p in boarding:
                    p.board_time = current_time
                    waiting_queue.remove(p)
                    onboard.append(p)

                # Přičtení přesného fyzikálního času manipulace na patře (dveře + osobní zpoždění)
                dwell_time = (
                    self.door_time
                    + sum(self.alight_delay * p.slow for p in alighting)
                    + sum(self.board_delay * p.slow for p in boarding)
                )
                current_time += dwell_time

            # 4. POHYB O JEDNO PATRO
            else:
                step = 1 if target_floor > current_floor else -1
                current_floor += step
                total_floors_traversed += 1
                current_direction = step
                current_time += self.travel_time_per_floor

        # Výpočet finálních metrik
        avg_wait = (
            sum(p.wait_time for p in finished) / len(finished)
            if finished
            else 0.0
        )
        avg_travel = (
            sum(p.travel_time for p in finished) / len(finished)
            if finished
            else 0.0
        )
        max_wait = max((p.wait_time for p in finished), default=0.0)

        return {
            "total_time": current_time,
            "passengers": len(finished),
            "stops": stops_count,
            "floors_traversed": total_floors_traversed,
            "avg_wait": avg_wait,
            "avg_travel": avg_travel,
            "max_wait": max_wait,
        }


# =====================================================================
# 4. SPUŠTĚNÍ A SROVNÁVACÍ VYHODNOCENÍ
# =====================================================================
if __name__ == "__main__":
    # Načtení simulačního scenáře ze souboru JSON
    with open(
        "zadani-bezny-den-ve-skole-B746-8C6D.json", "r", encoding="utf-8"
    ) as f:
        data = json.load(f)

    # Spuštění simulací pro oba algoritmy
    sim_smart = ElevatorSimulator(data, OptimizedLookElevatorController)
    res_smart = sim_smart.run()

    sim_dumb = ElevatorSimulator(data, DumbElevatorController)
    res_dumb = sim_dumb.run()

    # Formátovaný výpis srovnávací tabulky do konzole
    print("=" * 72)
    print("   SROVNÁVACÍ ANALÝZA: OPTIMALIZOVANÝ LOOK VS. HLOU PÝ (FIFO) VÝTAH")
    print("=" * 72)
    print(
        f"{'Metrika':<30} | {'Opt. LOOK (+AntiStarv)':<20} |"
        f" {'Hloupý (FIFO)':<15}"
    )
    print("-" * 72)
    print(
        f"{'Celkový čas simulace':<30} | {res_smart['total_time']:>18.2f} s |"
        f" {res_dumb['total_time']:>13.2f} s"
    )
    print(
        f"{'Přepraveno lidí':<30} | {res_smart['passengers']:>20d} |"
        f" {res_dumb['passengers']:>15d}"
    )
    print(
        f"{'Počet zastavení (Stops)':<30} | {res_smart['stops']:>20d} |"
        f" {res_dumb['stops']:>15d}"
    )
    print(
        f"{'Najeté patra celkem':<30} | {res_smart['floors_traversed']:>20d} |"
        f" {res_dumb['floors_traversed']:>15d}"
    )
    print(
        f"{'Průměrná doba čekání':<30} | {res_smart['avg_wait']:>18.2f} s |"
        f" {res_dumb['avg_wait']:>13.2f} s"
    )
    print(
        f"{'Průměrná doba jízdy':<30} | {res_smart['avg_travel']:>18.2f} s |"
        f" {res_dumb['avg_travel']:>13.2f} s"
    )
    print(
        f"{'Maximální doba čekání':<30} | {res_smart['max_wait']:>18.2f} s |"
        f" {res_dumb['max_wait']:>13.2f} s"
    )
    print("=" * 72)