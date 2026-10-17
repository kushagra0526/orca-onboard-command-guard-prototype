"""Held-out scenarios: committed to git before the first run and never edited after.

Different from dev on purpose: other ground stations, a 51.6 deg orbit family next to sun-synchronous ones, wider
altitude and beta ranges, more targets, smaller batteries, a more error-prone AI operator, and a truth model that
differs from the guard's nominal model by up to 15 % (dev: 10 %; the guard's SoC margin was set on dev).
"""

STATIONS = {"Kiruna": (67.9, 21.1), "Inuvik": (68.4, -133.5), "Awarua": (-46.5, 168.4), "Santiago": (-33.1, -70.7),
            "Hawaii": (19.0, -155.7), "Mauritius": (-20.5, 57.5), "Troll": (-72.0, 2.5)}

HELDOUT = {"stations": STATIONS, "n_stations": (2, 4), "n_targets": (25, 45), "alt": (420, 650), "inc": (51.0, 98.5), "beta": (0, 70),
           "n_conj": [0, 1, 1, 2, 2, 3], "battery": [26.0, 32.0, 38.0], "solar": [26.0, 30.0, 34.0], "model_error": 0.15,
           "p_sun_blind": 0.8, "p_shift": 0.25, "p_forget": 0.2, "p_late": 0.4, "greed": 1.3}
SEEDS = range(70_000, 70_200)
