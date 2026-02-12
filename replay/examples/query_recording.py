#!/usr/bin/env python3
"""
Ticket: 0056k_example_workflow_test_recording
Demonstrate msd_reader API usage with test recording database
"""

import sys
from pathlib import Path

try:
    import msd_reader
except ImportError:
    print("ERROR: msd_reader module not found.")
    print("Build with: cmake --build --preset debug-pybind-only")
    sys.exit(1)


def main():
    # Default to test recording
    db_path = Path("recordings/test_cube_drop.db")
    if len(sys.argv) > 1:
        db_path = Path(sys.argv[1])

    if not db_path.exists():
        print(f"ERROR: Database not found: {db_path}")
        print("\nGenerate one with:")
        print("  ./build/Debug/debug/generate_test_recording recordings/test_cube_drop.db")
        sys.exit(1)

    print(f"Opening database: {db_path}\n")
    db = msd_reader.Database(str(db_path))

    # List frames and timestamps
    print("=== FRAMES ===")
    frames = db.select_all_frames()
    print(f"Total frames: {len(frames)}")
    if frames:
        print(f"First frame: id={frames[0].id}, time={frames[0].simulation_time:.3f}s")
        print(f"Last frame:  id={frames[-1].id}, time={frames[-1].simulation_time:.3f}s")
    print()

    # Query body metadata
    print("=== BODY METADATA ===")
    metadata = db.select_all_static_assets()
    print(f"Total bodies: {len(metadata)}")
    for asset in metadata:
        print(f"  Body {asset.body_id}: mass={asset.mass:.2f}kg, "
              f"restitution={asset.restitution}, friction={asset.friction}")
    print()

    # Get states for first frame (exercises body_id FK property)
    if frames:
        frame_id = frames[0].id
        print(f"=== STATES (Frame {frame_id}) ===")
        states = db.select_inertial_states_by_frame(frame_id)
        print(f"Total states: {len(states)}")
        for state in states:
            print(f"  Body {state.body_id}: pos=({state.position.x:.2f}, "
                  f"{state.position.y:.2f}, {state.position.z:.2f}), "
                  f"frame_id={state.frame_id}")
        print()

    # Get collisions for a later frame (exercises contacts RepeatedField property)
    if len(frames) > 10:
        frame_id = frames[10].id
        print(f"=== COLLISIONS (Frame {frame_id}) ===")
        collisions = db.select_collisions_by_frame(frame_id)
        print(f"Total collision pairs: {len(collisions)}")
        for collision in collisions:
            contacts = collision.contacts  # RepeatedField .data property
            print(f"  Bodies {collision.body_a_id} <-> {collision.body_b_id}: "
                  f"{len(contacts)} contacts, frame_id={collision.frame_id}")
            if contacts:
                print(f"    Contact 0: depth={contacts[0].depth:.4f}m")
        print()

    # Get system energy timeseries (exercises frame_id FK property)
    print("=== SYSTEM ENERGY ===")
    sys_energy = db.select_all_system_energy()
    print(f"Total energy records: {len(sys_energy)}")
    if len(sys_energy) >= 2:
        print(f"  Frame {sys_energy[0].frame_id}: E={sys_energy[0].total_system_e:.3f}J")
        print(f"  Frame {sys_energy[-1].frame_id}: E={sys_energy[-1].total_system_e:.3f}J")
        print(f"  Total delta: {sys_energy[-1].total_system_e - sys_energy[0].total_system_e:.6f}J")
    print()

    print("SUCCESS: All queries completed.")


if __name__ == "__main__":
    main()
