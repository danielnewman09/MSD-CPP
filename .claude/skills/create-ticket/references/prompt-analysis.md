# Prompt Analysis Reference

Keyword mappings and heuristics for analyzing a free-form feature description.

## Math Design Detection

Set `Requires Math Design: Yes` if the prompt contains ANY of these keywords (case-insensitive):

### Physics Keywords
- `physics`, `collision`, `inertia`, `quaternion`, `tensor`, `eigenvalue`
- `impulse`, `friction`, `torque`, `angular`, `momentum`, `rigid body`
- `kinetic energy`, `potential energy`, `gravitational`
- `force`, `acceleration` (in physics context, not UI)
- `dynamics`, `kinematics`, `statics`

### Math Keywords
- `Lagrangian`, `Jacobian`, `Hamiltonian`, `Hessian`
- `numerical stability`, `convergence`, `divergence`
- `integration` (numerical, not software integration)
- `differential equation`, `ODE`, `PDE`
- `matrix`, `eigenvector`, `eigendecomposition`
- `interpolation`, `extrapolation`, `spline`
- `convex hull`, `Voronoi`, `Delaunay`
- `GJK`, `EPA`, `SAT`, `Minkowski`

### Symbol Keywords
- `ω` (omega), `α` (alpha), `τ` (tau), `∑` (sigma)
- `½mv²`, `F=ma`, LaTeX-style equations

## Language Detection

### Python Detection
Set Languages to include `Python` if the prompt mentions:
- `python`, `pybind`, `pybind11`, `pytest`
- `flask`, `fastapi`, `uvicorn`, `django`
- `pandas`, `numpy`, `scipy`, `matplotlib`
- `replay server`, `MCP server`, `analysis script`
- `.py` file references

### Frontend Detection
Set Languages to include `Frontend` if the prompt mentions:
- `frontend`, `web`, `browser`, `dashboard`
- `React`, `Vue`, `Angular`, `Svelte`
- `Three.js`, `WebGL`, `WebSocket client`, `WebAssembly`, `WASM`
- `HTML`, `CSS`, `JavaScript`, `TypeScript`
- `UI`, `user interface`, `visualization` (in web context)
- `Vite`, `webpack`, `npm`, `node`

### Multi-Language
If both Python and Frontend are detected, or any combination of C++/Python/Frontend, set Languages to the comma-separated list. E.g., `C++, Python, Frontend`.

## Component Detection

Map prompt keywords to target components:

| Keywords | Component |
|----------|-----------|
| `simulation`, `physics`, `solver`, `collision`, `rigid body`, `constraint`, `dynamics` | `msd-sim` |
| `GUI`, `render`, `graphics`, `shader`, `OpenGL`, `Vulkan`, `camera`, `3D`, `visualization` | `msd-gui` |
| `asset`, `mesh`, `geometry`, `convex hull`, `factory`, `model` | `msd-assets` |
| `transfer`, `record`, `DTO`, `database`, `serialization`, `pybind` | `msd-transfer` |
| `executable`, `main`, `application`, `startup` | `msd-exe` |
| `asset generation`, `generate_assets`, `procedural` | `msd-asset-gen` |
| `replay`, `recording`, `playback`, `debug replay` | `replay` |
| `benchmark`, `profiling`, `performance`, `analysis` | `analysis` |
| `traceability`, `design decision`, `indexer` | `scripts/traceability` |

If multiple components are detected, list them comma-separated.
If no component keywords are found, use `TBD`.

## Complexity Estimation

### Small
- Keywords: `simple`, `small`, `minor`, `tweak`, `quick`, `trivial`
- Signals: Single-file change, rename, config change, adding a single method

### Medium (default)
- Keywords: `add`, `implement`, `create`, `new feature`
- Signals: New class, new subsystem interaction, moderate scope

### Large
- Keywords: `refactor`, `redesign`, `overhaul`, `subsystem`, `architecture`
- Signals: Multiple files, new interfaces, cross-component changes

### XL
- Keywords: `rewrite`, `migration`, `framework`, `engine`
- Signals: Fundamental architectural changes, new subsystem from scratch

## Priority Detection

### Critical
- Keywords: `critical`, `urgent`, `blocker`, `crash`, `data loss`, `security`

### High
- Keywords: `important`, `high priority`, `needed`, `regression`, `broken`

### Medium (default)
- No specific priority keywords detected

### Low
- Keywords: `nice to have`, `low priority`, `optional`, `someday`, `cleanup`
