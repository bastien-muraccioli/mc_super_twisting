# SuperTwisting — mc_rtc Global Plugin

A model-based mc_rtc global plugin for external joint-torque estimation and collision detection using a **Super-Twisting sliding-mode observer** on the generalized momentum. It implements two observer variants — a **second-order** (classic super-twisting) and a **third-order** — and optionally fuses estimates with a **force/torque (FT) sensor**. The estimated external torque is fed back to the robot model and used for collision detection.

---

## How It Works

Both observers share the same generalized momentum framework. At each tick:

1. **Computes the generalized momentum and driving term**:

   ```
   p = M q̇
   γ = τ + Cᵀ q̇ - g   [+ Jᵀ F_ext  if FT sensor enabled]
   ```

2. **Runs the selected observer** (see below).

3. **Optionally adds the FT sensor contribution** — the estimated torque from the observer is combined with the joint-space projection of the end-effector wrench `Jᵀ F_ext` (gravity-compensated, rotated to the world frame).

4. **Feeds the estimated torque back to the robot** via `robot.setExternalTorques(τ_ext)` when the plugin is active.

5. **Applies an adaptive threshold** — an `LpfThreshold` low-pass filter tracks `τ_ext` and a collision is flagged when any joint exceeds `filtered_signal ± offset`.

6. **Sets a datastore flag** — writes `true` to `"Obstacle detected"` when a collision is detected and *Collision stop* is enabled.

### Second-Order Super-Twisting Observer

Classic super-twisting on the momentum error `ε = p - p̂`:

```
ṗ̂         = γ + γ₁ |ε|^(1/2) sign(ε) + α₁ ε + τ̂_ext
τ̂̇_ext     = (α₂ I + γ₂ |ε|) sign(ε)
```

The `γ₂ |ε|` adaptive term scales the correction with the magnitude of the momentum error, improving convergence under large disturbances.

### Third-Order Super-Twisting Observer

A third-order extension that also estimates the **derivative** of the external torque, useful when the disturbance varies rapidly:

```
ṗ̂         = γ + γ₁⁽³⁾ |ε|^(2/3) sign(ε) + α₁ ε + τ̂_ext
τ̂̇_ext     = γ₂⁽³⁾ |ε|^(1/3) sign(ε) + τ̇̂_ext
τ̈̇_ext     = γ₃⁽³⁾ sign(ε)
```

Gains are derived from the single tuning parameter `c` (maximum expected disturbance derivative) by default:

```
γ₃⁽³⁾ = 1.1 c,   γ₂⁽³⁾ = (9/2) c^(5/6),   γ₁⁽³⁾ = 3 c^(1/3)
```

---

## Configuration

```yaml
Plugins:
  - SuperTwisting

super_twisting:
  reference_frame:    FT_sensor_wrench    # Body name for the FT sensor frame and Jacobian root
  torque_sensor_name: externalTorqueSensor
  ft_sensor_name:     EEForceSensor

  # Second-order observer gains
  gamma1: 21.0     # |ε|^(1/2) correction gain
  gamma2: 100.0    # Adaptive |ε| gain on the torque update
  alpha1: 100.0    # Proportional momentum correction
  alpha2: 100.0    # Proportional torque update

  # Third-order observer (optional overrides; auto-derived from c if ≤ 0)
  c:                    100.0   # Upper bound on the disturbance derivative
  gamma1_third_order:   0.0     # Auto: 3 * c^(1/3)
  gamma2_third_order:   0.0     # Auto: (9/2) * c^(5/6)
  gamma3_third_order:   0.0     # Auto: 1.1 * c

  # Detection threshold
  threshold_filtering: 0.05                      # LPF coefficient (0–1)
  threshold_offset:    [10.0, 10.0, 10.0, ...]  # Per-joint band (must match joint count)
```

| Parameter | Default | Description |
|---|---|---|
| `reference_frame` | `"FT_sensor_wrench"` | Body name used for the Jacobian and FT sensor frame |
| `ft_sensor_name` | `"EEForceSensor"` | FT sensor name in the robot description |
| `gamma1` / `gamma2` | `21.0` / `100.0` | Second-order observer correction gains |
| `alpha1` / `alpha2` | `100.0` / `100.0` | Adaptive proportional gains |
| `c` | `100.0` | Tuning constant for third-order gain auto-derivation |
| `threshold_filtering` | `0.05` | LPF coefficient for adaptive threshold |
| `threshold_offset` | `10.0` (all joints) | Per-joint detection band half-width |

Changing any gain in the GUI resets `p̂`, `τ̂_ext`, and `τ̂̇_ext` to zero to avoid transients.

---

## Runtime GUI

The plugin exposes a panel under **Plugins → SuperTwisting**:

| Control | Description |
|---|---|
| `Is estimation feedback active` | Enable/disable feeding `τ_ext` back to the robot model |
| `Is third order active` | Switch between second- and third-order observers |
| `Use FT Sensor` | Enable/disable FT sensor fusion |
| `gamma1`, `gamma2`, `alpha1`, `alpha2` | Second-order gains (reset observer on change) |
| `gamma1/2/3_third_order`, `c` | Third-order gains; setting `c` recomputes all three (reset on change) |
| `Threshold offset` | Per-joint detection band half-width |
| `Threshold filtering` | LPF coefficient for adaptive threshold |
| `jointShown` | Joint index to display in plots |
| `Collision stop` | Enable/disable writing the detection result to the datastore |
| `Verbose` | Print per-joint collision messages to the console |
| `Add plot` | Open live plots (see below) |

### Live Plots

| Plot | Contents |
|---|---|
| `Momentum` | `p`, `p̂` (2nd order), `p̂` (3rd order) for the selected joint |
| `Momentum error` | `ε` for 2nd and 3rd order observers |
| `Torque estimation` | `τ̂_ext` for 2nd and 3rd order |
| `Torque estimation with FT Sensor` | `τ_ext` used, FT ground truth `Jᵀ F`, estimates with adaptive thresholds |
| `Gamma` | Driving term `γ` |
| `tau_ext_dot_hat` | Estimated disturbance derivative (3rd order only) |

---

## Logged Entries

| Key | Type | Description |
|---|---|---|
| `SuperTwisting_p` | `VectorXd` | Generalized momentum `p` |
| `SuperTwisting_p_hat` | `VectorXd` | Estimated momentum `p̂` (2nd order) |
| `SuperTwisting_p_error` | `VectorXd` | Momentum error `ε` (2nd order) |
| `SuperTwisting_tau_ext_hat` | `VectorXd` | Estimated external torque (2nd order, no FT) |
| `SuperTwisting_tau_ext_hat_dot` | `VectorXd` | Rate of change of `τ̂_ext` (2nd order) |
| `SuperTwisting_tau_ext_hat_ft_sensor` | `VectorXd` | Estimated external torque (2nd order, with FT) |
| `SuperTwisting_tau_ext` | `VectorXd` | External torque sent to the robot |
| `SuperTwisting_gamma` | `VectorXd` | Driving term `γ` |
| `SuperTwisting_threshold_high/low` | `VectorXd` | Adaptive detection bounds |
| `SuperTwisting_obstacle_detected` | `bool` | Collision detection flag |
| `SuperTwisting_third_order_p_hat` | `VectorXd` | Estimated momentum (3rd order) |
| `SuperTwisting_third_order_p_error` | `VectorXd` | Momentum error (3rd order) |
| `SuperTwisting_third_order_tau_ext_hat` | `VectorXd` | Estimated external torque (3rd order, no FT) |
| `SuperTwisting_third_order_tau_ext_hat_ft_sensor` | `VectorXd` | Estimated external torque (3rd order, with FT) |
| `SuperTwisting_third_order_tau_ext_hat_dot` | `VectorXd` | Rate of change of `τ̂_ext` (3rd order) |
| `SuperTwisting_third_order_tau_ext_dot_hat` | `VectorXd` | Estimated disturbance derivative `τ̇̂_ext` |

---

## Datastore Interface

| Key | Type | Description |
|---|---|---|
| `"Obstacle detected"` | `bool` | `true` when any joint's `τ_ext` exits the adaptive band; only written when *Collision stop* is enabled |
| `"extTorquePlugin"` | `vector<string>` | Shared registry of active torque-estimating plugins; `SuperTwistingEstimator` is added/removed each tick based on `plugin_active` |
| `"SuperTwisting::isActive"` | callable `() → bool` | Returns the current `plugin_active` state |
| `"SuperTwisting::toggleActive"` | callable `() → void` | Toggles `plugin_active` |

All entries are created on `init` if they do not already exist.

---

## Dependencies

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)