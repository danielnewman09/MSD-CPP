# Frontend Design: Live Simulation Cleanup (0072e)

**Ticket**: [0072e_live_simulation_cleanup](../../../../tickets/0072e_live_simulation_cleanup.md)
**Date**: 2026-02-21
**Status**: Initial Frontend Design
**Depends On**:
- [design.md](../design.md) (APPROVED WITH NOTES)
- [integration-design.md](../integration-design.md) (APPROVED WITH MANDATORY ADDITION)

---

## Summary

This document specifies the frontend architecture change for 0072e. The scope is a single
surgical modification to `replay/static/js/live-app.js`: the `onStartSimulation()` function
must conditionally omit `mass`, `restitution`, and `friction` fields from the `configure`
message payload for environment objects (FR-6).

No new modules, no new HTML structure, no CSS changes, no Three.js scene graph changes.
The change is confined to one function in one file.

---

## Integration Contract Reference

- Integration design: `docs/designs/0072e_live_simulation_cleanup/integration-design.md`
- Contract 2, FR-6: `configure` message payload must omit `mass`, `restitution`, `friction`
  for environment objects. Server applies Pydantic defaults when fields are absent.

### Pre-0072e `configure` Wire Format

```json
{
  "type": "configure",
  "objects": [
    {
      "asset_name": "cube",
      "position": [0.0, 0.0, 5.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "inertial",
      "mass": 10.0,
      "restitution": 0.8,
      "friction": 0.5
    },
    {
      "asset_name": "plane",
      "position": [0.0, 0.0, 0.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "environment",
      "mass": 10.0,       // meaningless; sent anyway
      "restitution": 0.8,
      "friction": 0.5
    }
  ]
}
```

### Post-0072e `configure` Wire Format (FR-6)

```json
{
  "type": "configure",
  "objects": [
    {
      "asset_name": "cube",
      "position": [0.0, 0.0, 5.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "inertial",
      "mass": 10.0,
      "restitution": 0.8,
      "friction": 0.5
    },
    {
      "asset_name": "plane",
      "position": [0.0, 0.0, 0.0],
      "orientation": [0.0, 0.0, 0.0],
      "object_type": "environment"
      // mass, restitution, friction omitted — server applies defaults
    }
  ]
}
```

---

## Module Architecture

No new modules. One existing module is modified.

### Modified: `replay/static/js/live-app.js`

**Scope of change**: `onStartSimulation()` function, lines 254–262 (the `objects` map)

The only change is replacing the unconditional object spread with a conditional builder that
branches on `entry.object_type`:

```javascript
// BEFORE (current code — unconditional):
const objects = spawnList.map(entry => ({
    asset_name:  entry.asset_name,
    position:    entry.position,
    orientation: entry.orientation,
    object_type: entry.object_type,
    mass:        entry.mass,       // sent even for environment objects
    restitution: entry.restitution,
    friction:    entry.friction,
}));

// AFTER (FR-6 — conditional):
const objects = spawnList.map(entry => {
    const obj = {
        asset_name:  entry.asset_name,
        position:    entry.position,
        orientation: entry.orientation,
        object_type: entry.object_type,
    };
    if (entry.object_type === 'inertial') {
        obj.mass        = entry.mass;
        obj.restitution = entry.restitution;
        obj.friction    = entry.friction;
    }
    return obj;
});
```

**Why this pattern**: Object mutation (property assignment after creation) is idiomatic
JavaScript for conditionally including properties. The alternative — `entry.object_type ===
'inertial' ? {..., mass, restitution, friction} : {...}` — duplicates the base fields in two
arms of a ternary. The mutation approach avoids duplication and matches the spirit of the
design document (ticket, line 187–201).

**Consistency with UI**: The mass input group is already hidden for environment objects via
`elMassGroup.classList.add('hidden')` in `onObjectTypeChange()`. The payload change aligns
wire protocol behavior with what the UI already communicates to the user: environment objects
do not have configurable mass.

---

## State Management

No state changes. The `spawnList` entries already store `mass`, `restitution`, and `friction`
for all objects (they are set in `onAddObject()` regardless of object type). The change does
not touch `spawnList` — it only affects what is sent over the wire at simulation start time.

This means the UI continues to store physics values for environment objects internally (in
`spawnList`), but does not transmit them. This is correct: the values are irrelevant for
rendering and the server ignores them anyway (Pydantic defaults apply). If a future ticket
introduces restitution/friction controls for environment objects (affecting collision response),
the values are already captured in `spawnList` and can be included by removing the conditional.

---

## No HTML or CSS Changes

The `live.html` and `live.css` files are unchanged. The mass input group
(`#mass-group`) is already hidden for environment objects via `elMassGroup.classList.add('hidden')`
in `onObjectTypeChange()`. No new UI controls are needed.

The `scene.js` module is unchanged — `SceneManager` already handles environment objects
correctly (per the constraint in the ticket: "Do not change the SceneManager in scene.js").

---

## No Three.js Scene Graph Changes

The `handleFrame()` function in `live-app.js` passes each state entry to
`sceneManager.updateFrame(frameData)`. After FR-1 (C++ side), frame entries will include
`is_environment: true` for environment bodies. The `SceneManager.updateFrame()` code uses
`body_id` to look up mesh entries and update position/orientation. It does not branch on
`is_environment`. The new field is ignored. No `live-app.js` changes are needed to consume
FR-1's extended frame data.

---

## WebSocket Client Design

No changes to WebSocket connection management, message dispatch, or error handling.
The `configure` message is the only artifact affected by this ticket.

### Message Handler Impact Summary

| Message Type | Handler | Change |
|-------------|---------|--------|
| `metadata` | `handleMetadata()` | No change — already handles environment bodies in `msg.bodies` |
| `frame` | `handleFrame()` | No change — `SceneManager` ignores `is_environment` field |
| `complete` | `handleComplete()` | No change |
| `error` | `handleServerError()` | No change |
| `configure` (outgoing) | `onStartSimulation()` | **Modified** — conditional physics fields (FR-6) |

---

## Files to Modify

| File | Change | FR |
|------|--------|----|
| `replay/static/js/live-app.js` | Replace unconditional `objects` map in `onStartSimulation()` with conditional builder that omits `mass`/`restitution`/`friction` for environment objects | FR-6 |

---

## Files NOT Modified

| File | Reason |
|------|--------|
| `replay/static/live.html` | No new UI controls needed |
| `replay/static/css/live.css` | No visual changes |
| `replay/static/js/scene.js` | Ticket constraint: "Do not change the SceneManager in scene.js" |

---

## Test Strategy

No automated frontend tests exist in this codebase (per the integration design: "No automated
tests for FR-6"). The change is verified by code inspection:

1. The conditional `if (entry.object_type === 'inertial')` correctly gates the three fields.
2. The `obj` base object contains only the four fields that are always sent.
3. The inertial branch adds `mass`, `restitution`, `friction` from `entry`.
4. No other code path in `live-app.js` sends the `configure` message — `onStartSimulation` is
   the only producer.

Manual verification: start a simulation with one inertial and one environment object, open
browser DevTools Network tab, inspect the WebSocket frames, confirm the environment object
entry lacks `mass`/`restitution`/`friction`.

---

## Integration Contract Coverage

| Contract | Requirement | Coverage |
|----------|-------------|----------|
| Contract 2 — FR-6 | Environment objects omit `mass`/`restitution`/`friction` from configure payload | `onStartSimulation()` conditional builder; verified by code inspection |

---

## Open Questions

None. All design decisions are resolved.

### Resolved

1. **Should `mass`/`restitution`/`friction` be removed from `spawnList` entries for environment
   objects?** — No. The `spawnList` is internal client state. Removing the values would require
   changes to `onAddObject()` and `renderSpawnList()` for no functional benefit. The values are
   harmlessly stored and simply not transmitted.

2. **Should the `onAddObject()` function skip capturing mass/restitution/friction for environment
   objects?** — No, for the same reason as above. The UI hides the mass input for environment
   objects, but still reads `elMassInput.value` (harmlessly). Changing this is out of scope and
   would require modifying the `spawnList` entry schema.

3. **Does `handleFrame()` need updating to handle `is_environment` on frame entries (FR-1)?**
   — No. `SceneManager.updateFrame()` dispatches on `body_id` only. The new `is_environment`
   field is ignored. No frontend code change is needed to consume FR-1's extended frame data.

---

## Frontend Design Review

**Reviewer**: Frontend Design Reviewer
**Date**: 2026-02-21
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Design Conformance

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| All integration contract requirements addressed | ✓ | FR-6 fully specified; explicitly notes no change needed for FR-1 frame handling |
| Scope correctly bounded | ✓ | Only `live-app.js` modified; `scene.js`, `live.html`, `live.css` explicitly excluded with rationale |
| No new modules introduced unnecessarily | ✓ | Change is one function; no module boundary warranted |

#### JavaScript Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Idiomatic pattern for conditional properties | ✓ | Object mutation (property assignment after creation) is correct idiomatic JavaScript for conditional properties; avoids ternary duplication |
| No state regression | ✓ | `spawnList` internal state is not modified; physics values are retained internally for potential future use |
| Consistent with existing UI behavior | ✓ | Mass input already hidden for environment objects via `elMassGroup.classList.add('hidden')` — payload change aligns wire protocol with UI |
| No Three.js scene graph changes needed | ✓ | `SceneManager.updateFrame()` already handles environment body entries via `body_id` lookup; `is_environment` field (FR-1) is ignored |

#### Integration Contract Alignment

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Post-0072e configure payload matches integration design Contract 2 | ✓ | Environment objects omit `mass`/`restitution`/`friction`; server Pydantic defaults apply |
| FR-1 frame handling verified no-op | ✓ | Explicitly confirmed `handleFrame()` needs no changes for FR-1 extended frame data |

### Risks Identified

None. The change is one function, four lines added (the conditional branch), no new state,
no new DOM manipulation, no WebSocket protocol impact beyond the intended payload reduction.

### Summary

The frontend design is minimal and correct. The single change — replacing an unconditional
`objects` map with a conditional builder in `onStartSimulation()` — is the smallest possible
implementation of FR-6. All other aspects of the frontend (scene rendering, WebSocket lifecycle,
UI state, Three.js scene graph) are unaffected. The design is approved for progression to
Implementation.
