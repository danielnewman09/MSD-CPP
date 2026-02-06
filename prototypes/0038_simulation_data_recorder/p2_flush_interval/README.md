# Prototype P2: Flush Interval Sensitivity Analysis

## Question
What flush interval balances memory usage vs. write latency?

## Success Criteria
Identify interval that keeps buffer under 10MB while maintaining <1s write latency

## Approach
Simulate continuous recording at 60 FPS with 10 objects per frame for 10 seconds (6000 records total).
Test flush intervals: 10ms, 50ms, 100ms, 500ms.
Measure buffer memory usage and flush latency.

## Build and Run

```bash
cd prototypes/0038_simulation_data_recorder/p2_flush_interval
c++ -std=c++20 -O2 main.cpp -lsqlite3 -o p2_flush_interval
./p2_flush_interval
```

## Expected Output
- Buffer size and flush latency measurements for each interval
- Summary table with all metrics
- Pass/fail evaluation and recommended interval
