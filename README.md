# Dubin's Car Multi-Robot Navigation and Control

A comprehensive MATLAB implementation of nonlinear control systems for autonomous Dubin's car models, featuring progressive complexity from single-robot navigation to multi-robot coordination with obstacle avoidance using Control Barrier Functions (CBFs) and Higher-Order Control Barrier Functions (HOCBFs).

## Project Overview

This project explores nonlinear control design for Dubin's car models, a simplified representation of wheeled vehicles that can only move forward and turn at bounded rates. The implementation demonstrates a progressive approach to autonomous navigation:

1. **Single Robot Control** → Basic navigation and obstacle avoidance
2. **Two Robot Coordination** → Inter-robot collision avoidance 
3. **Three Robot Systems** → Multi-agent coordination with priority management
4. **Complex Scenarios** → Combined obstacle and multi-robot navigation

## System Flow and Architecture

### Detailed Operation Flow

#### 1. Dynamic Priority Assignment
- **Distance-based ranking**: Robots closer to their goals receive higher priority
- **Weight assignment**: Priority robots get higher weights (100 > 50 > 1) in the QP cost function
- **Real-time updates**: Priority reassigned continuously as robots progress
- **Goal completion handling**: Priority released when robots reach targets

#### 2. Nominal Control Law (Pure Pursuit)
- **Goal-seeking behavior**: Each robot computes desired heading toward its target
- **Proportional controller**: `u_nominal = kappa * (θ_desired - θ_current)`
- **Angle wrapping**: Ensures shortest rotation path using `atan2`

#### 3. Safety-Critical Control Layer

##### Control Barrier Functions (CBF)
For **robot-to-robot** collision avoidance:
```matlab
h(x) = ||p_i - p_j||² - safe_distance²
```

##### Higher-Order Control Barrier Functions (HOCBF)
For **robot-to-obstacle** avoidance (relative degree 2):
```matlab
h(x) = ||p_robot - p_obstacle||² - radius²
ψ(x) = Lf·h(x) + κ·h(x)  % HOCBF formulation
```

#### 4. Quadratic Programming (QP) Optimization
- **Objective function**: Minimize weighted deviation from nominal control
  ```
  min ||u - u_nominal||²_W
  ```
- **Safety constraints**: Linear inequalities ensuring CBF/HOCBF conditions
  ```
  A·u ≤ b  % Safety constraints
  ```
- **Priority enforcement**: Diagonal weight matrix W implements robot priorities

#### 5. Control Integration and State Update
- **Input saturation**: Respect physical actuator limits
- **Forward Euler integration**: Update robot states
- **Goal checking**: Stop robots upon target arrival
- **Emergency stop**: Velocity reduction for critical proximity

## Implementation Details

### Dubin's Car Model
**State vector**: `[x, y, θ]` (position and heading)
**Dynamics**:
```matlab
x_dot = [v·cos(θ), v·sin(θ), u]'
```
Where:
- `v` = constant forward velocity
- `u` = angular velocity control input

### Key Parameters
- **Velocity**: 1 m/s (constant)
- **Control bounds**: ±3 rad/s
- **Safe distances**: 1-5 m (scenario-dependent)
- **CBF gains**: κ = 1-10
- **Simulation timestep**: 0.1 s

## Simulation Scenarios

### 1. **Single Robot Navigation** (`velocity_control.m`)
- Basic point-to-point navigation
- Dynamic velocity adjustment based on distance to goal
- Two velocity control methods: heading-based and distance-based

### 2. **Obstacle Avoidance** (`obstacle_avoidance.m`)
- Single robot avoiding circular obstacle
- HOCBF implementation with dither mechanism
- Standstill capability at goal

### 3. **Two Robot Coordination** 
- **Without Priority** (`two_robots_crisscross_noPriority.m`): Basic collision avoidance
- **With Priority** (`two_robots_crisscross_withPriority.m`): Dynamic priority system
- **With Obstacle** (`two_robots_crisscross_priority_obstacle.m`): Combined constraints

### 4. **Three Robot Systems**
- **Priority Only** (`three_robots_crisscross_priority.m`): Multi-agent coordination
- **Full System** (`three_robots_crisscross_priority_obstacle.m`): Complete scenario

## Tunable Parameters

### Control Gains
- `kappa`: Proportional gain for nominal controller (1-30)
- `class_K_const`: CBF constraint gain (1-10)

### Safety Parameters
- `safe_distance`: Inter-robot minimum distance (1-5 m)
- `obstacle_radius`: Obstacle size (1.5-2 m)

### Priority Weights
- Priority 1: 100 (highest)
- Priority 2: 50 (medium)  
- Priority 3: 1 (lowest)

### Performance Tuning Tips
- **Higher kappa**: Faster convergence but potential overshooting
- **Larger safe_distance**: More conservative but may cause oscillations
- **Priority weights**: Adjust based on scenario requirements

## References

- **Control Barrier Functions**: Ames et al., "Control Barrier Function Based Quadratic Programs for Safety Critical Systems"
- **Dubin's Car Model**: Dubins, L.E., "On Curves of Minimal Length with a Constraint on Average Curvature"
- **Multi-Robot Coordination**: LaValle, S.M., "Planning Algorithms"