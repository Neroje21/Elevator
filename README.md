# Diskrétní Simulátor Řízení Výtahů (Elevator Dispatching Simulator)

Tento projekt implementuje vysoce efektivní diskrétní simulátor pro výtahové systémy v budovách. Součástí projektu je pokročilý průmyslový řídicí algoritmus **LOOK (SCAN)** rozšířený o dynamický **Anti-Starvation mechanismus**, a také srovnávací baseline algoritmus typu **FIFO (Point-to-Point Taxi)**.

---

## 1. Přehled Projektu

Hlavním cílem simulátoru je optimalizace vertikální přepravy osob v budově s rozsáhlým denním provozem (např. školy, kancelářské budovy). Simulátor přesně modeluje fyzikální čas pohybu kabiny, manipulaci s dveřmi i individuální časové prodlevy cestujících při nástupu a výstupu.

### Klíčové přínosy optimalizovaného algoritmu:
- **Redukce spotřeby energie a opotřebení:** Snížení celkového počtu najetých pater o více než 30 % oproti základním algoritmům.
- **Atomické zastavování (Single Dwell Cycle):** Výstup i nástup více osob na stejném patře probíhá v rámci jediného cyklu otevření dveří.
- **Eliminace extrémního čekání (Anti-Starvation):** Garance horní hranice doby čekání pro cestující v mezipatřích během dopravních špiček.

---

## 2. Implementované Algoritmy

### A) Optimized LOOK (SCAN) + Anti-Starvation
Průmyslový algoritmus, který funguje podobně jako čtecí hlava pevného disku:
1. **Držení směru jízdy:** Výtah pokračuje v aktuálním směru (NAHORU nebo DOLŮ), dokud existují jakékoliv požadavky ve stávajícím směru (požadavky na výstup v kabině nebo vnější volání na chodbách).
2. **Průběžný nástup a výstup:** Pokud je v kabině volná kapacita (`capacity`), výtah po cestě zastavuje v mezipatřích a přibírá cestující jedoucí stejným směrem.
3. **Plynulé otočení směru:** K otočení směru jízdy dochází až v momentě, kdy před výtahem nezůstávají žádné další požadavky.
4. **Ochrana proti hladovění (Anti-Starvation):** Pokud libovolný cestující čeká na chodbě déle než nastavený práh (`starvation_threshold = 200.0 s`), výtah po vyprázdnění kabiny dočasně přeruší standardní sweep a vyrazí přímo pro tohoto čekajícího.

### B) Dumb FIFO (Baseline / Taxi Model)
Srovnávací naivní algoritmus:
- Obsluhuje cestující striktně podle času jejich příchodu (First-In, First-Out).
- Po naložení cestujícího jedná jako „taxi“ — jede přímo do cílového patra a po cestě ignoruje jakékoliv další čekající v mezipatřích.

---

## 3. Fyzikální Model a Parametry Simulace

Simulátor kalkuluje čas jako spojitou veličinu na základě fyzikálních konfiguračních parametrů ze vstupního JSON souboru:

| Parametr | Význam | Výchozí hodnota (Školní scénář) |
| :--- | :--- | :--- |
| `floors` | Celkový počet pater v budově | `8` (0 až 7) |
| `capacity` | Maximální počet osob v kabině | `6` osob |
| `speed` | Rychlost výtahu | `2.0` m/s |
| `floorHeight` | Výška jednoho patra | `3.0` m |
| `doorOpen` / `doorClose` | Čas otevření a zavření dveří | `1.5` s + `1.5` s (Celkem `3.0` s) |
| `boarding.in` / `out` | Základní čas nástupu/výstupu 1 osoby | `1.0` s / `1.0` s |
| `slow` | Individuální koeficient pomalosti osoby | `1.0` (standard) až `2.0` (pomalý) |

### Výpočet časových prodlev:
- **Čas přejezdu mezi 1 patrem:** $T_{\text{travel}} = \frac{\text{floorHeight}}{\text{speed}} = \frac{3.0}{2.0} = 1.5\text{ s}$
- **Trvání zastávky na patře:**
  $$T_{\text{dwell}} = (T_{\text{doorOpen}} + T_{\text{doorClose}}) + \sum (T_{\text{out}} \times \text{slow}_{\text{out}}) + \sum (T_{\text{in}} \times \text{slow}_{\text{in}})$$

---

## 4. Architektura Kódu

Skript `vytah.py` je rozdělen do čtyř logických modulů:

```text
┌─────────────────────────────────────────────────────────────────┐
│                       Passenger (Dataclass)                     │
│   Uchovává metriky: arrival_time, board_time, wait_time, atd.   │
└────────────────────────────────┬────────────────────────────────┘
                                 │
┌────────────────────────────────▼────────────────────────────────┐
│                     Elevator Controllers                        │
│ ├─ OptimizedLookElevatorController (LOOK + Anti-Starvation)     │
│ └─ DumbElevatorController (FIFO Baseline)                       │
└────────────────────────────────┬────────────────────────────────┘
                                 │
┌────────────────────────────────▼────────────────────────────────┐
│                      ElevatorSimulator                          │
│   Diskrétní simulační motor provádějící časový posun a stavy    │
└────────────────────────────────┬────────────────────────────────┘
                                 │
┌────────────────────────────────▼────────────────────────────────┐
│                          Main Execution                         │
│   Načtení JSON dat, spuštění simulací a vygenerování tabulky    │
└─────────────────────────────────────────────────────────────────┘

```
Vytvořil Alex Solnař a Tomáš Linka
Alex použil svoje předplatné na gemini (bohužel claude jsem koupil až dneska) a Tomáš poskytl nápady na zlepšení a ověroval data
