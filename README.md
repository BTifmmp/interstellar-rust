# 🚀 Interstellar Rust – Optymalizacja Trajektorii Ziemia–Księżyc z użyciem PSO

Projekt implementuje **zaawansowaną symulację lotu rakiety z Ziemi na Księżyc** pod wpływem pól grawitacyjnych obu ciał niebieskich (z uwzględnieniem rotacji Ziemi wokół własnej osi). Do optymalizacji parametrów startowych zastosowano **algorytm roju cząstek (Particle Swarm Optimization - PSO)**.

Celem algorytmu jest znalezienie takiej prędkości początkowej (w lokalnym układzie ENU – wschód, północ, góra) oraz ewentualnego przesunięcia startowego, które **minimalizuje odległość od wybranego punktu na Księżycu**, optymalizując jednocześnie zużycie paliwa (redukcja prędkości początkowej i końcowej).

Wyniki optymalizacji są automatycznie serializowane do pliku `*.json`. Przy kolejnych uruchomieniach program inteligentnie pomija proces obliczeniowy i przechodzi od razu do interaktywnej wizualizacji 3D, zrealizowanej w czasie rzeczywistym przy użyciu biblioteki `macroquad`.

---

## 📂 Struktura Projektu

```text
interstellar_rust/
├── Cargo.toml               # Zależności: macroquad, rand, rayon, serde, space-dust, chrono
├── config.json              # Konfiguracja startu, celu, PSO, symulacji oraz wag funkcji kosztu
├── pso_history.json         # Automatycznie generowany plik z historią optymalizacji (nazwa w zależności od nazwy w 'main')
└── src/
    ├── main.rs              # Punkt wejścia: wczytanie configu, uruchomienie PSO / odczyt historii, pętla renderowania
    ├── algo/                # Moduł optymalizacyjny
    │   ├── config.rs        # Definicje struktur: Config, GeoPoint, PsoParams, Bounds, SimulationParams
    │   ├── history.rs       # Zarządzanie historią: IterationRecord, OptimizationHistory (JSON)
    │   ├── objective.rs     # Funkcja kosztu, compute_start_state, analyze_trajectory
    │   ├── pso.rs           # Core algorytmu PSO (cząstka, rój, update, optimize)
    │   └── mod.rs
    ├── simulation/          # Moduł symulacji fizycznej
    │   ├── objects.rs       # Definicje ciał: Body, RocketState, MoonState
    │   ├── propagator.rs    # Integrator RK4, obliczanie przyspieszeń acceleration_at
    │   ├── world.rs         # TrajectoryGenerator (prekomputacja pozycji Księżyca, generowanie trajektorii)
    │   └── mod.rs
    ├── render/              # Moduł wizualizacji 3D
    │   ├── camera.rs        # DrawCamera, CameraController (WASD + sterowanie myszą)
    │   ├── drawing.rs       # Rysowanie obiektów niebieskich, trajektorii, HUD i etykiet tekstowych
    │   ├── has_position.rs  # Trait HasPosition dla RocketState, MoonState, Vec3d
    │   ├── iteration_drawer.rs # Renderowanie historii iteracji (gradienty kolorów, wyróżnienie najlepszej)
    │   ├── mouse.rs         # System przechwytywania i zwalniania kursora myszy
    │   └── mod.rs
    └── util/                # Narzędzia matematyczne i pomocnicze
        ├── math.rs          # Wektory Vec3d + operatory (Add, Sub, Mul, Div)
        ├── geometry.rs      # Transformacje: geographic_to_cartesian, enu_vector_to_cartesian
        └── mod.rs
```

---

## 🛠️ Koncepcja Działania

### 🌌 Symulacja Fizyczna

- **Grawitacja:** Model uwzględnia grawitację Ziemi i Księżyca (stałe parametry grawitacyjne $\mu$). Dokładna pozycja Księżyca w czasie pobierana jest z precyzyjnych efemeryd za pomocą biblioteki `space-dust`.
- **Propagacja Stanu:** Wykorzystano klasyczną metodę **Runge-Kutta 4** z konfigurowalnym krokiem czasowym `dt_s` (domyślnie `5.0` sekund).
- **Obsługa Kolizji:** Uderzenie w Ziemię skutkuje natychmiastowym nałożeniem ogromnej kary grawitacyjnej (`1e9`). Kontakt z powierzchnią Księżyca nie nakłada kary (traktowany jako potencjalne lądowanie).
- **Cel:** Dowolny zdefiniowany punkt na powierzchni Księżyca (`target_point`).

### 📐 Parametry Optymalizowane (6 Wymiarów)

| Parametr | Interpretacja w lokalnym układzie ENU (East-North-Up) | Jednostka |
| :------: | ----------------------------------------------------- | :-------: |
|   `vx`   | Prędkość początkowa na wschód                         |   km/s    |
|   `vy`   | Prędkość początkowa na północ                         |   km/s    |
|   `vz`   | Prędkość początkowa w górę                            |   km/s    |
|   `dx`   | Przesunięcie startowe na wschód                       |    km     |
|   `dy`   | Przesunięcie startowe na północ                       |    km     |
|   `dz`   | Wysokość startowa nad średnim poziomem morza          |    km     |

> **Informacja: ** Do prędkości zdefiniowanej w układzie ENU system automatycznie dodaje **prędkość wynikającą z ruchu obrotowego Ziemi** (dla doby gwiazdowej $\sim$ 23h 56m 4s), co wiernie symuluje start z rzeczywistej powierzchni planety.

### 📊 Funkcja Kosztu

We fragmencie uwzględniającym prędkość początkową

```math
cost = \text{best\_dist} \cdot \text{end\_speed}^{w_{\text{end}}} \cdot \text{start\_vel}^{w_{\text{start}}} + \text{collision\_penalty}
```

**Objaśnienie zmiennych:**

- `best_dist` – minimalna odległość uzyskana przez rakietę względem punktu docelowego na Księżycu.
- `start_speed` – norma wektora prędkości początkowej (koszt energetyczny startu).
- `end_speed` – prędkość relatywna do powierzchni Księżyca w punkcie największego zbliżenia.
- `collision_penalty` – wynosi `1e9` przy zderzeniu z Ziemią; w przeciwnym wypadku wynosi `0`.
- `w_dist`, `w_start`, `w_end` – wagi wagowe pobierane z konfiguracji (domyślnie `[1.0, 1.0, 2.0]`).

### 🧬 Algorytm PSO (Particle Swarm Optimization)

- Wielkość populacji definiuje parametr `num_particles`, a czas trwania `max_iterations`.
- Ruch cząstek kontrolowany jest przez współczynniki: `w` (bezwładność), `c1` (składnik poznawczy/indywidualny) oraz `c2` (składnik społeczny/grupowy).
- Każda iteracja zapisuje najlepszy lokalny wynik do struktury `history.records`.

### 💾 Zapis i Odczyt Wyników

- Po ukończeniu pełnego cyklu PSO, stan optymalizacji jest zrzucany do pliku `pso_history.json`.
- Kolejne wywołania programu wykrywają ten plik, dzięki czemu kosztowna obliczeniowo faza optymalizacji jest pomijana, a trajektorie dla wizualizacji generowane są w ułamku sekundy.

### 📺 Interfejs i Wizualizacja 3D

- **Rendering środowiska:** Ziemia i Księżyc reprezentowane są jako sfery, a za nimi rysowane są linie orbity.
- **Chmura rozwiązań:** Wyświetlane są trajektorie próbne ze **wszystkich iteracji** w postaci płynnego gradientu kolorystycznego (od szarości do jasnych barw), obrazując proces zbiegania się algorytmu.
- **Trajektoria optymalna:** Najlepsze znalezione rozwiązanie jest wyraźnie pogrubione i podświetlone na żółto wraz z dynamiczną etykietą prędkości.

---

## 🚀 Jak Uruchomić?

### Wymagania Wstępne

- Środowisko **Rust** (wersja **1.70** lub nowsza) wraz z menedżerem pakietów `cargo`.

### Pierwsze Uruchomienie (Pełna Optymalizacja)

```bash
git clone <url-tego-repozytorium>
cd interstellar_rust
cargo run --release
```

_Program załaduje plik `config.json`, uruchomi algorytm PSO (może to zająć od kilku do kilkudziesięciu minut w zależności od procesora i liczby cząstek), zapisze historię, a następnie otworzy okno wizualizacji._

### Kolejne Uruchomienia (Tryb Natychmiastowy)

```bash
cargo run --release
```

_Jeśli plik `pso_history.json` jest obecny w katalogu głównym, faza PSO zostanie pominięta, a wizualizacja uruchomi się natychmiast._

---

## 🎮 Sterowanie w Wizualizacji 3D

|                   Klawisz / Akcja                   | Działanie                                                   |
| :-------------------------------------------------: | ----------------------------------------------------------- |
| <kbd>W</kbd> <kbd>A</kbd> <kbd>S</kbd> <kbd>D</kbd> | Przemieszczanie kamery (przód, lewo, tył, prawo)            |
|                   <kbd>Mysz</kbd>                   | Rotacja kamery (rozejrzenie się po scenie)                  |
|                   <kbd>ESC</kbd>                    | Zwolnienie blokady myszy (przywrócenie kursora systemowego) |
|             <kbd>Strzałka w Górę</kbd>              | Zwiększenie tempa upływu czasu symulacji                    |
|              <kbd>Strzałka w Dół</kbd>              | Zmniejszenie tempa upływu czasu symulacji                   |
|                  <kbd>Spacja</kbd>                  | Pauza (czas = 0)                                            |

---

## ⚙️ Konfiguracja Zaawansowana (`config.json`)

Zmiany w pliku konfiguracyjnym **nie wymagają ponownej kompilacji projektu**. Program interpretuje je przy każdym uruchomieniu.

### 📌 Punkty Startowe i Docelowe

```json
"start_point": {
    "latitude_deg": 28.5,       // Dodatnie = północ, ujemne = południe
    "longitude_deg": -80.5,     // Dodatnie = wschód, ujemne = zachód
    "altitude_km": 0.0          // Wysokość nad powierzchnią planety
},
"target_point": {               // Na księżycu
    "latitude_deg": 0.0,
    "longitude_deg": 0.0,
    "altitude_km": 0.0
}
```

### 🎛️ Parametry Algorytmu PSO

```json
"pso_params": {
    "num_particles": 100,       // Wielkość populacji (wyższa = lepsza dokładność, ale wolniejszy czas obliczeń)
    "max_iterations": 40,       // Liczba epok algorytmu
    "w": 0.5,                   // Bezwładność cząstki (sugerowany zakres: 0.4 – 0.9)
    "c1": 1.5,                  // Współczynnik poznawczy (indywidualny) (sugerowany zakres: 1.5 – 2.0)
    "c2": 1.5                   // Współczynnik społeczny (grupowy) (sugerowany zakres: 1.5 – 2.0)
}
```

### 📉 Zakresy Poszukiwań (Bounds)

```json
"bounds": {                     // x - odpowiada wschód-zachód, y - północ południe, z - wysokość nad ziemią
    "vx": [-15.0, 15.0],        // Prędkość ENU - wschód (km/s)
    "vy": [-15.0, 15.0],        // Prędkość ENU - północ (km/s)
    "vz": [8.0, 16.0],          // Prędkość ENU - w górę (km/s)
    "dx": [0.0, 0.0],           // Opcjonalne przesunięcie (jeśli min == max, wymiar jest zamrożony)
    "dy": [0.0, 0.0],
    "dz": [0.0, 0.0]
}
```

### ⏱️ Parametry Integratora i Wag

```json
"simulation_params": {
    "max_duration_days": 7.0,   // Maksymalny czas trwania misji (w dniach)
    "dt_s": 5.0,                // Dokładność integracji RK4 (krok w sekundach)
    "every_nth": 400            // Decymacja danych trajektorii do renderowania (optymalizacja RAM/GPU)
},
"weights": [1.0, 1.0, 2.0]      // Tablica wag kosztu: [w_dist, w_start, w_end]
```

### 📝 Przykład Użycia

**Scenariusz:** Start z kosmodromu Cape Canaveral ($28.5^\circ$, $-80.5^\circ$, w stopniach dziesiętnych) i próba lądowania na równiku Księżyca ($0^\circ$, $0^\circ$) z maksymalnym czasem misji ustawionym na 7 dni.

1. Uruchom program z domyślnymi wartościami w pliku `config.json`.
2. Po zakończeniu pełnego procesu optymalizacji automatycznie otworzy się okno wizualizacji, w którym zobaczysz:
   - **Szare i jasne linie:** Trajektorie testowe generowane przez poszczególne cząstki w kolejnych iteracjach (obrazują proces uczenia się roju).
   - **Grubą żółtą linię:** Najlepszą globalnie znalezioną trajektorię lotu o najniższym koszcie.
   - **Żółty znacznik z etykietą:** Aktualną pozycję optymalnego statku wraz z dynamicznie wyświetlaną prędkością.

---

## 📌 Uwagi Końcowe

- **⚡ Wydajność:** Pierwsza faza optymalizacji jest wymagająca obliczeniowo. W celach szybkiego przetestowania kodu zmniejsz parametry w konfiguracji do `num_particles: 20` oraz `max_iterations: 10`. Dla uzyskania precyzyjnych i produkcyjnych wyników zaleca się ustawienie 50–100 cząstek oraz 30–50 iteracji.
- **🎯 Dokładność Integracji:** Krok czasowy `dt_s = 5.0` sekund zapewnia dobrą precyzję fizyczną dla 7-dniowej misji. Zwiększenie tego kroku przyspieszy obliczenia, jednak może prowadzić do kumulacji błędów i zniekształcenia trajektorii.
- **💾 Plik `pso_history.json`:** Wygenerowany plik z historią jest w pełni przenośny. Możesz go archiwizować, przesyłać lub uruchamiać na innych maszynach, aby natychmiast odtworzyć wygenerowaną wizualizację 3D bez ponoszenia kosztu obliczeniowego PSO.
- **🌌 Potencjalne Rozszerzenia:** Model można rozbudować o wpływ grawitacyjny Słońca lub innych ciał Układu Słonecznego. W tym celu wystarczy zaktualizować funkcję `acceleration_at` w module propagacji oraz dodać prekomputację ich pozycji w strukturze `TrajectoryGenerator`.

---

## 📄 Licencja i Autorzy

Projekt został zrealizowany w ramach przedmiotowych zajęć z języka **Rust** na **Akademii Górniczo-Hutniczej (AGH) w Krakowie**. Kod źródłowy został udostępniony na zasadach open-source.

- **Autor:** Mateusz Gawroński, Błażej Turczynowicz
- **Data:** 10-06-2026
