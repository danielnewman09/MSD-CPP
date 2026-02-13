# Ticket 0060c: ReplayMatchers — GTest Custom Matchers

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Feature / Testing
**Priority**: Medium
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0060_replay_integrated_gtest](0060_replay_integrated_gtest.md)
**Depends On**: [0060b_recording_query_api](0060b_recording_query_api.md)

---

## Overview

Create GTest custom matchers that wrap `RecordingQuery` to provide physics-invariant assertions with clear failure messages. These matchers enable expressive test assertions like:

```cpp
EXPECT_THAT(recordingPath().string(), EnergyConservedWithin(0.05));
EXPECT_THAT(recordingPath().string(), NeverPenetratesBelow(cubeId, -0.6));
EXPECT_THAT(recordingPath().string(), BodyComesToRest(cubeId, 0.1));
```

On failure, matchers report the actual measured value (e.g., "max energy drift was 0.12") for easy debugging.

---

## Requirements

### R1: EnergyConservedWithin

```cpp
MATCHER_P(EnergyConservedWithin, tolerance, ...)
```

- Takes `tolerance` (double) — maximum acceptable relative energy drift
- Argument: recording database path (std::string)
- Creates `RecordingQuery`, calls `maxEnergyDrift()`
- Passes if `maxEnergyDrift() < tolerance`
- On failure: reports `"max energy drift was X"`

### R2: NeverPenetratesBelow

```cpp
MATCHER_P2(NeverPenetratesBelow, bodyId, zMin, ...)
```

- Takes `bodyId` (uint32_t) and `zMin` (double) — floor z-threshold
- Creates `RecordingQuery`, calls `minZ(bodyId)`
- Passes if `minZ(bodyId) >= zMin`
- On failure: reports `"min z was X"`

### R3: BodyComesToRest

```cpp
MATCHER_P2(BodyComesToRest, bodyId, speedThreshold, ...)
```

- Takes `bodyId` (uint32_t) and `speedThreshold` (double)
- Creates `RecordingQuery`, calls `speedHistory(bodyId)`
- Passes if the final speed value is below `speedThreshold`
- On failure: reports `"final speed was X"`

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd/msd-sim/test/Replay/ReplayMatchers.hpp` | Header-only GTest custom matchers |

### Modified Files
None — header-only, included by test files.

---

## Interface

```cpp
#pragma once

#include <gmock/gmock.h>
#include "RecordingQuery.hpp"

namespace msd_sim::test {

MATCHER_P(EnergyConservedWithin, tolerance,
          "energy conserved within " + testing::PrintToString(tolerance)) {
  RecordingQuery q{arg};
  double drift = q.maxEnergyDrift();
  *result_listener << "max energy drift was " << drift;
  return drift < tolerance;
}

MATCHER_P2(NeverPenetratesBelow, bodyId, zMin,
           "body " + testing::PrintToString(bodyId) +
           " never penetrates below z=" + testing::PrintToString(zMin)) {
  RecordingQuery q{arg};
  double actualMinZ = q.minZ(bodyId);
  *result_listener << "min z was " << actualMinZ;
  return actualMinZ >= zMin;
}

MATCHER_P2(BodyComesToRest, bodyId, speedThreshold,
           "body " + testing::PrintToString(bodyId) +
           " comes to rest below speed " + testing::PrintToString(speedThreshold)) {
  RecordingQuery q{arg};
  auto speeds = q.speedHistory(bodyId);
  double finalSpeed = speeds.empty() ? 0.0 : speeds.back();
  *result_listener << "final speed was " << finalSpeed;
  return finalSpeed < speedThreshold;
}

}  // namespace msd_sim::test
```

---

## Test Plan

### Unit Tests

```cpp
TEST(ReplayMatchers, EnergyConservedWithin_PassesWhenDriftBelowTolerance)
TEST(ReplayMatchers, EnergyConservedWithin_FailsWhenDriftExceedsTolerance)
TEST(ReplayMatchers, NeverPenetratesBelow_PassesWhenAboveThreshold)
TEST(ReplayMatchers, NeverPenetratesBelow_FailsWhenBelowThreshold)
TEST(ReplayMatchers, BodyComesToRest_PassesWhenFinalSpeedBelowThreshold)
TEST(ReplayMatchers, BodyComesToRest_FailsWhenStillMoving)
```

These tests create recording databases with known values and verify matcher behavior.

---

## Acceptance Criteria

1. [ ] **AC1**: `EnergyConservedWithin` passes/fails correctly based on energy drift
2. [ ] **AC2**: `NeverPenetratesBelow` passes/fails correctly based on min z position
3. [ ] **AC3**: `BodyComesToRest` passes/fails correctly based on final speed
4. [ ] **AC4**: All matchers report the actual measured value in failure messages
5. [ ] **AC5**: Matchers handle edge cases (empty recordings, zero initial energy)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
