# Level 2 Controller Implementation Guide

## Challenge Overview

**Level 2** ist deutlich schwieriger als Level 0/1:
- ✅ **Randomisierte Gate-Positionen** (aber feste Reihenfolge)
- ✅ **Randomisierte Hindernis-Positionen**
- ✅ **Randomisierte Drohnen-Eigenschaften** (Masse, Trägheit)
- ❌ Keine randomisierten Tracks (gates in fester Reihenfolge)

## Controller Vergleich

| Feature | StateController | Level2Controller |
|---------|-----------------|------------------|
| **Waypoints** | Hard-coded | Dynamisch aus Gates |
| **Obstacle Avoidance** | Keine | Ausweichoffset |
| **Replanning** | Keine | Ja (bei Abweichung/Änderung) |
| **Adaptivität** | Keine | Hoch |
| **Für Level 2** | ❌ Nicht geeignet | ✅ Geeignet |

## Level2Controller Architektur

```
┌─────────────────────────────────┐
│   compute_control()             │
│  (aufgerufen jeden Schritt)      │
└──────────────┬──────────────────┘
               │
       ┌───────▼────────┐
       │ Should replan? │
       └───────┬────────┘
               │
        ┌──────▼──────────────┐
        │ _plan_trajectory()  │
        │  - Waypoints       │
        │  - Cubic Spline    │
        └──────┬─────────────┘
               │
        ┌──────▼──────────────┐
        │ Get desired state   │
        │ at current time     │
        └──────┬─────────────┘
               │
        ┌──────▼──────────────┐
        │ Return action       │
        │ [pos, 0, 0, ...]    │
        └─────────────────────┘
```

## Verwendung

### Methode 1: Via Kommandozeile
```bash
cd /Users/leonie/00_Elektrotechnik/Drone\ Racing/lsy_drone_racing
python -m scripts.sim --controller level2_controller --level level2
```

### Methode 2: In Python
```python
from lsy_drone_racing.control import Level2Controller
from lsy_drone_racing.envs import DroneRaceEnv
import gymnasium as gym

# Environment erstellen
env = gym.make(
    "DroneRacing-v0",
    config_path="config/level2.toml"
)

obs, info = env.reset()
controller = Level2Controller(obs, info, env.env.env.unwrapped.config)

# Simulation
for _ in range(1000):
    action = controller.compute_control(obs, info)
    obs, reward, done, truncated, info = env.step(action)
    done = controller.step_callback(action, obs, reward, done, truncated, info)
    
    if done or truncated:
        break

controller.episode_callback()
env.close()
```

### Methode 3: Test-Script
```bash
python /Users/leonie/00_Elektrotechnik/Drone\ Racing/lsy_drone_racing/test_level2_controller.py
```

## Technische Details

### Waypoint-Generierung
1. **Start**: Position [-1.5, 0.75, 0.05]
2. **Gates durchfahren**: Gate-Mittelpunkte als Waypoints
3. **Hindernis-Ausweichung**: Offset ±0.15m basierend auf Hindernis-Nähe
4. **End**: Finale Gate + 0.2m Höhe

### Replanning-Trigger
```python
if (tick - last_replan_tick) < 50:
    return False  # Cooldown
    
if ||new_gates_pos - old_gates_pos|| > 0.01:
    return True  # Gates haben sich bewegt
    
if ||current_pos - desired_pos|| > 0.5:
    return True  # Drohne zu weit weg
```

### Trajectorie
- **Typ**: Cubic Spline durch Waypoints
- **Zeit**: 20 Sekunden für gesamte Strecke
- **Interpolation**: Smooth, mit natürlichen Randbedingungen

## Observation Space

Die Controller erhält diese Daten:

```python
obs = {
    'pos': np.array([x, y, z]),              # Aktuelle Position
    'quat': np.array([qx, qy, qz, qw]),      # Quaternion (Orientierung)
    'vel': np.array([vx, vy, vz]),           # Velocity
    'ang_vel': np.array([wx, wy, wz]),       # Angular velocity
    'target_gate': int,                       # Nächste Gate (-1 wenn fertig)
    'gates_pos': np.array((n_gates, 3)),     # Positionen aller Gates
    'gates_quat': np.array((n_gates, 4)),    # Orientierungen aller Gates
    'gates_visited': np.array((n_gates,)),   # Welche Gates passiert
    'obstacles_pos': np.array((n_obs, 3)),   # Obstacle Positionen
    'obstacles_visited': np.array((n_obs,)), # Welche Obstacles passiert
}
```

## Performance-Tipps

### 1. Verbesserte Trajectorie-Planung
```python
# Aktueller Code: Cubic Spline mit fester Zeit
# Besser: Geschwindigkeits-basierte Timing
def _plan_with_speed_profile(self):
    # Variere Zeit pro Waypoint basierend auf Schwierigkeit
    for i, waypoint in enumerate(waypoints):
        if obstacle_nearby(waypoint):
            time_for_segment = 3.0  # Länger für schwierige Stellen
        else:
            time_for_segment = 1.5  # Kürzer für einfache Stellen
```

### 2. Velocity-Estimation
```python
# Aktuell: Velocity = 0 (lässt niedrigeren Controller entscheiden)
# Besser: Derivative der Spline als Velocity-Target
def compute_control(self, obs, info):
    des_pos = self._get_desired_position(t)
    des_vel = self._trajectory_spline(t, 1)  # 1. Ableitung
    des_acc = self._trajectory_spline(t, 2)  # 2. Ableitung
    action = np.concatenate((des_pos, des_vel, des_acc, [0, 0, 0, 0]))
```

### 3. Bessere Hindernis-Ausweichung
```python
# Aktuell: Simple Offset basierend auf Nähe
# Besser: Ellipse um Drohne zur Hinderniserkennung
# Oder: Rapid-exploring Random Tree (RRT) Path Planning

def _apply_obstacle_avoidance_rrt(self, waypoint):
    # Implementiere RRT für komplexe Szenarien
    pass
```

### 4. Yaw-Control
```python
# Aktuell: Yaw = 0 (nach vorne)
# Besser: Drehe zur nächsten Gate
def _compute_desired_yaw(self):
    current_pos = obs['pos']
    next_waypoint = self._current_waypoints[self._next_waypoint_idx]
    direction = next_waypoint - current_pos
    desired_yaw = np.arctan2(direction[1], direction[0])
    return desired_yaw
```

### 5. Learning-Integration
```python
# Nach erfolgreichem Run:
# - Speichere erfolgreiche Trajectorie
# - Verwende als Basis für nächste Runs
# - Fine-tune mit RL (train_rl.py mit level2 Randomization)
```

## Debug-Tipps

### Visualisierung aktivieren
```python
config.sim.render = True
config.sim.camera = -1  # World view

# Dann wird der render_callback() aufgerufen und zeigt:
# - 🟢 Grüne Punkte: Waypoints
# - 🔴 Rote Linie: Planned trajectory
# - 🔵 Blaue Punkt: Current setpoint
```

### Trajectory-Ausgabe
```python
# In episode_callback():
np.save(f"trajectory_{episode}.npy", self._current_waypoints)
print(f"Waypoints: {self._current_waypoints}")
```

### Replanning-Debugging
```python
def _should_replan(self, obs):
    gate_distance = np.linalg.norm(obs['gates_pos'] - self._gates_pos)
    distance_to_traj = np.linalg.norm(obs['pos'] - desired_pos)
    
    print(f"Gate dist: {gate_distance:.4f}, Traj dist: {distance_to_traj:.4f}")
    
    if distance_to_traj > 0.5:
        print("REPLANNING: Too far from trajectory")
        return True
    return False
```

## Häufige Probleme

| Problem | Ursache | Lösung |
|---------|--------|--------|
| Controller crasht in Hindernis | Ausweichung zu schwach | Erhöhe `_waypoint_offset` |
| Drohne schwingt wild | Waypoints zu nah | Increase smoothing in spline |
| Zu langsam | Trajectory zu lang | Reduziere `_total_time` |
| Instabil nach Replanning | Zu häufiges Replanning | Erhöhe `_replan_cooldown` |
| Gates werden verpasst | Waypoints falsch berechnet | Debug: `_compute_waypoints()` |

## Nächste Schritte

1. **Test & Validierung** (diese Implementierung)
2. **Quantitativer Vergleich** mit StateController
3. **Optimierung** der Parameter basierend auf Test-Ergebnissen
4. **Integration mit RL-Training** (Level 2 Pre-training)
5. **Deployment auf echter Hardware** (RealRaceEnv)

---

Viel Erfolg mit Level 2! 🚁
