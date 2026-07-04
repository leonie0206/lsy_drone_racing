# Level 2 Controller Variants - Comparison Guide

## Controller Overview

Zwei Implementierungen für Level 2:

### 1. **Level2Controller** (`level2_controller.py`)
Die Basis-Implementierung mit adaptiver Pfadplanung.

**Merkmale:**
- ✅ Dynamische Waypoint-Generierung aus Gate-Positionen
- ✅ Hindernis-Ausweichung durch Offset
- ✅ Adaptive Replanning (bei Änderung oder Abweichung)
- ✅ Smooth trajectories via cubic splines
- ⚠️ Nur Position-Befehle (Velocity/Acceleration = 0)

**Geeignet für:**
- Erste Tests und Debugging
- Wenn Lower-Level-Controller stabiler arbeitet
- Performance-kritische Anwendungen (schneller)

**Parameter:**
```python
_total_time = 20.0           # 20s für Strecke
_waypoint_offset = 0.15      # 15cm Ausweichung
_replan_cooldown = 50        # Replanning max alle 1s
_replan_threshold = 0.5      # Replan bei >50cm Abweichung
```

---

### 2. **Level2AdvancedController** (`level2_advanced_controller.py`)
Erweiterte Version mit Velocity- und Accelerations-Estimation.

**Zusätzliche Merkmale:**
- ✅ Alles aus Level2Controller
- ✅ **Velocity-Befehle** (1. Ableitung der Trajectory)
- ✅ **Acceleration-Befehle** (2. Ableitung der Trajectory)
- ✅ **Variable Zeit-Allokation** (mehr Zeit bei Hindernissen)
- ✅ **Yaw-Tracking** (Drohne zeigt zur nächsten Gate)

**Geeignet für:**
- Aggressive Flying mit besserer Tracking
- Wenn Attitude-Controller Velocity-Input verarbeiten kann
- Fine-tuned Performance

**Zusätz-Parameter:**
```python
_use_yaw_tracking = True     # Drohne dreht zur nächsten Gate
_total_time = 18.0           # Kürzere Zeit (aggressiver)
_replan_cooldown = 40        # Häufigeres Replanning
_replan_threshold = 0.4      # Strengere Replanning-Bedingung
```

---

## Vergleichs-Tabelle

| Aspekt | Level2Controller | Level2Advanced |
|--------|------------------|-----------------|
| **Komplexität** | Niedrig | Mittel |
| **Position-Tracking** | ✅ | ✅✅ |
| **Velocity-Info** | ❌ | ✅ |
| **Acceleration-Info** | ❌ | ✅ |
| **Yaw-Control** | ❌ (0°) | ✅ (adaptive) |
| **Zeit-Allokation** | Uniform | Variable |
| **Hindernis-Handling** | Basis | Erweitert |
| **Computation** | Schnell | Mittel |
| **Stabilität** | ✅✅ | ✅ |
| **Schnelligkeit** | Moderat | Hoch |

---

## Verwendung & Vergleich

### Test beide Varianten
```bash
# Basis-Version
python -m scripts.sim --controller level2_controller --level level2 --seed 42

# Erweiterte Version
python -m scripts.sim --controller level2_advanced_controller --level level2 --seed 42
```

### Programmatisch Testen
```python
from lsy_drone_racing.control import Level2Controller, Level2AdvancedController
from lsy_drone_racing.envs import DroneRaceEnv
import gymnasium as gym

# Erstelle Environment
env = gym.make("DroneRacing-v0", config_path="config/level2.toml")
obs, info = env.reset()

# Test beide Controller
for ControllerClass in [Level2Controller, Level2AdvancedController]:
    obs, info = env.reset()
    controller = ControllerClass(obs, info, env.env.env.unwrapped.config)
    
    total_reward = 0.0
    for _ in range(1000):
        action = controller.compute_control(obs, info)
        obs, reward, done, truncated, info = env.step(action)
        total_reward += reward
        done = controller.step_callback(action, obs, reward, done, truncated, info)
        
        if done or truncated:
            break
    
    controller.episode_callback()
    print(f"{ControllerClass.__name__}: Reward = {total_reward:.2f}")

env.close()
```

---

## Welche Version sollte ich nutzen?

### Wähle **Level2Controller** wenn:
- Du gerade anfängst und Debugging machst
- Der Attitude-Controller stabil läuft
- Schnelle Computation wichtig ist
- Du die einfachste Lösung brauchst

### Wähle **Level2AdvancedController** wenn:
- Du aggressive Flight-Performance brauchst
- Der Attitude-Controller Velocity-Inputs verarbeiten kann
- Du Yaw-Tracking willst (Drohne schaut zur Gate)
- Du längerfristige Optimierung betreiben möchtest

---

## Performance-Metriken

Bei Tests solltest du messen:

```python
def test_controller(ControllerClass, n_runs=5):
    results = []
    
    for run_id in range(n_runs):
        env = gym.make("DroneRacing-v0", config_path="config/level2.toml")
        obs, info = env.reset()
        controller = ControllerClass(obs, info, env.env.env.unwrapped.config)
        
        metrics = {
            'run_id': run_id,
            'total_reward': 0.0,
            'gates_passed': 0,
            'max_distance_error': 0.0,
            'collisions': 0,
            'total_steps': 0,
        }
        
        for step in range(1000):
            action = controller.compute_control(obs, info)
            obs, reward, done, truncated, info = env.step(action)
            
            metrics['total_reward'] += reward
            metrics['total_steps'] += 1
            metrics['gates_passed'] = obs.get('target_gate', -1) + 1
            
            if reward < -0.5:  # Approx collision reward
                metrics['collisions'] += 1
            
            done = controller.step_callback(action, obs, reward, done, truncated, info)
            if done or truncated:
                break
        
        controller.episode_callback()
        results.append(metrics)
        env.close()
    
    return results
```

---

## Migration zwischen Versionen

Die APIs sind kompatibel - du kannst einfach den Controller-Namen wechseln:

```python
# Beide funktionieren identisch:
controller1 = Level2Controller(obs, info, config)
controller2 = Level2AdvancedController(obs, info, config)

action = controller1.compute_control(obs)  # Funktioniert mit beiden
```

---

## Debugging & Visualisierung

Aktiviere Rendering in `level2.toml`:
```toml
[sim]
render = true
camera = -1  # World view
```

Dann siehst du:
- 🟢 **Grün**: Waypoints
- 🔴 **Rot**: Planned trajectory
- 🔵 **Blau**: Current target position
- 🟡 **Gelb** (Advanced): Yaw-Richtung

---

## Nächste Entwicklung

Falls nötig, könnte man noch weitere Varianten erstellen:

1. **Level2RLController**: Mit vorgefertigtem RL-Modell
2. **Level2MPC**: Mit Model Predictive Control (ACADOS)
3. **Level2HybridController**: Kombination mehrerer Strategien

Für jetzt sind diese zwei Varianten optimal für Level 2! 🚁
