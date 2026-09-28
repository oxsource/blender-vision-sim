#!/usr/bin/env python3
"""Export a high-rate Road Scene motion oracle without rendering video.

Example:
    python3 scripts/export_motion_rate.py \\
      --clip /path/to/bloop/video --out dist/motion_rate_100hz --fps 100

The output frames.csv is a motion-only oracle at the requested rate. It is not
paired with a video; the original clip remains the image source for AVM replay.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "addons", "opencv_camera"))

from core.scenes import drive_path, road_path, road_track  # noqa: E402


def read_oracle(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def make_plan(clip, fps):
    motion = clip["motion"]
    road = clip.get("road", {})
    vehicle = clip["vehicle"]["body"]
    track = road_track.preset_track(
        road.get("preset", "compact"),
        center=True,
        curve_bank_deg=float(road.get("curve_bank_deg", 0.0)),
    )
    zones = ()
    if road.get("crosswalk", False):
        zone = road_track.crosswalk_slow_zone(track)
        if zone is not None:
            zones = (zone,)

    profile = motion.get("profile", "scenario")
    if profile == "parking" or road.get("parking", False):
        radius = (float(motion.get("road_width_m", road.get("road_width_m", 7.0))) / 2.0 +
                  float(motion.get("shoulder_m", road.get("shoulder_m", 1.0))) +
                  float(vehicle["length"]) / 2.0)
        return road_path.parking_plan(
            track,
            s_entry=float(motion.get("start_distance_m", radius + 2.0)),
            radius=radius,
            cruise=float(motion.get("cruise_speed_mps", 7.0)),
            parking_speed=float(motion.get("parking_speed_mps", 2.0)),
            slow_speed=float(motion.get("slow_speed_mps", 3.5)),
            accel=float(motion.get("accel_mps2", 2.5)),
            decel=float(motion.get("decel_mps2", 2.5)),
            fps=fps,
            slow_zones=zones,
        )

    return road_path.plan(
        track,
        speed=float(motion.get("cruise_speed_mps", 7.0)),
        direction=motion.get("direction", "forward"),
        profile=profile,
        accel=float(motion.get("accel_mps2", 2.5)),
        decel=float(motion.get("decel_mps2", 2.5)),
        fps=fps,
        loops=float(motion.get("loops", 1.0)),
        start_distance=float(motion.get("start_distance_m", 0.0)),
        slow_speed=float(motion.get("slow_speed_mps", 3.5)),
        slow_zones=zones,
    )


def mounts_from_clip(clip):
    mounts = {}
    for camera in clip.get("cameras", []):
        name = camera.get("camera")
        mount = camera.get("mount", {})
        if not name or "location" not in mount or "rotation_deg" not in mount:
            continue
        mounts[name] = drive_path.Mount(
            tuple(float(x) for x in mount["location"]),
            tuple(float(x) for x in mount["rotation_deg"]),
        )
    return mounts


def compare_plan(plan, oracle):
    if len(plan.frames) != len(oracle):
        raise ValueError(f"10 Hz reconstruction has {len(plan.frames)} rows; "
                         f"oracle has {len(oracle)}")
    worst = {key: 0.0 for key in ("time_s", "speed_mps", "position_m", "yaw_deg",
                                  "steering_deg")}
    for frame, row in zip(plan.frames, oracle):
        worst["time_s"] = max(worst["time_s"], abs(frame.time - float(row["time_s"])))
        worst["speed_mps"] = max(worst["speed_mps"], abs(frame.speed - float(row["speed_mps"])))
        worst["position_m"] = max(
            worst["position_m"], math.hypot(frame.x - float(row["x_m"]),
                                             frame.y - float(row["y_m"])))
        yaw_error = (frame.yaw - float(row["yaw_deg"]) + 180.0) % 360.0 - 180.0
        worst["yaw_deg"] = max(worst["yaw_deg"], abs(yaw_error))
        worst["steering_deg"] = max(
            worst["steering_deg"], abs(frame.steering_deg - float(row["steering_deg"])))
    return worst


def write_motion_samples(path, plan):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        handle.write(road_path.motion_samples_csv_text(plan))


def f_world_truth(row, first, center_to_rear):
    offset_x, offset_y, _ = (float(value) for value in center_to_rear)
    yaw = math.radians(float(row["yaw_deg"]))
    first_yaw = math.radians(float(first["yaw_deg"]))

    def rear_xy(source_row, source_yaw):
        x = float(source_row["x_m"])
        y = float(source_row["y_m"])
        return (x + offset_x * math.cos(source_yaw) - offset_y * math.sin(source_yaw),
                y + offset_x * math.sin(source_yaw) + offset_y * math.cos(source_yaw))

    rear_x, rear_y = rear_xy(row, yaw)
    first_x, first_y = rear_xy(first, first_yaw)
    dx, dy = rear_x - first_x, rear_y - first_y
    c0, s0 = math.cos(first_yaw), math.sin(first_yaw)
    return (-c0 * dx - s0 * dy,
            -s0 * dx + c0 * dy,
            yaw - first_yaw)


def evaluate_motion_samples(plan, target_frames, center_to_rear, wheel_base):
    """Integrate each signal stream at sample times and score at video times."""
    offset_x, offset_y, _ = (float(value) for value in center_to_rear)
    truth = [f_world_truth(row, target_frames[0], center_to_rear) for row in target_frames]
    rear0 = truth[0]
    x_left, y_forward = -offset_x, offset_y
    c, s = math.cos(rear0[2]), math.sin(rear0[2])
    center_x = rear0[0] - (c * x_left + s * y_forward)
    center_y = rear0[1] - (-s * x_left + c * y_forward)
    yaw = rear0[2]

    samples = []
    for frame in plan.frames:
        steering = -frame.steering_deg if frame.gear == road_path.GEAR_REVERSE else frame.steering_deg
        samples.append((frame.time, frame.speed, frame.gear, steering))

    target_times = [float(row["time_s"]) for row in target_frames]
    last_time = samples[0][0]
    cursor = 0
    aligned_frames = 0
    reconstructed = []
    for target_time in target_times:
        while cursor < len(samples) and samples[cursor][0] <= target_time + 1e-5:
            time_s, speed, gear, steering = samples[cursor]
            dt = time_s - last_time
            if dt > 0.0:
                velocity = 0.0 if gear == road_path.GEAR_PARKED else speed
                if gear == road_path.GEAR_REVERSE:
                    velocity = -velocity
                omega = velocity / wheel_base * math.tan(math.radians(steering))
                yaw += omega * dt
                center_x += velocity * math.sin(yaw) * dt
                center_y += velocity * math.cos(yaw) * dt
            last_time = time_s
            cursor += 1
        if cursor > 0 and abs(samples[cursor - 1][0] - target_time) <= 1e-5:
            aligned_frames += 1

        c, s = math.cos(yaw), math.sin(yaw)
        rear_x = center_x + c * x_left + s * y_forward
        rear_y = center_y - s * x_left + c * y_forward
        reconstructed.append((rear_x, rear_y, yaw))

    errors = []
    for index, (expected, actual) in enumerate(zip(truth, reconstructed)):
        position = math.hypot(actual[0] - expected[0], actual[1] - expected[1])
        yaw_error = abs((actual[2] - expected[2] + math.pi) % (2.0 * math.pi) - math.pi)
        errors.append({"frame": index, "segment": target_frames[index]["segment"],
                       "position_m": position, "yaw_deg": math.degrees(yaw_error)})
    count = len(errors)
    max_position = max(errors, key=lambda row: row["position_m"])
    max_yaw = max(errors, key=lambda row: row["yaw_deg"])
    return {
        "video_frame_time_alignment": f"{aligned_frames}/{len(target_frames)}",
        "end_position_m": errors[-1]["position_m"],
        "end_yaw_deg": errors[-1]["yaw_deg"],
        "rms_position_m": math.sqrt(sum(row["position_m"] ** 2 for row in errors) / count),
        "max_position_m": max_position["position_m"],
        "max_position_frame": max_position["frame"],
        "max_position_segment": max_position["segment"],
        "max_yaw_deg": max_yaw["yaw_deg"],
        "max_yaw_frame": max_yaw["frame"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clip", required=True, help="source directory with clip.json and frames.csv")
    parser.add_argument("--out", required=True, help="output directory (no video is rendered)")
    parser.add_argument("--fps", type=float, default=100.0, help="motion sample rate, default 100 Hz")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")

    clip_dir = os.path.abspath(args.clip)
    out_dir = os.path.abspath(args.out)
    with open(os.path.join(clip_dir, "clip.json"), encoding="utf-8") as handle:
        clip = json.load(handle)
    source_oracle = read_oracle(os.path.join(clip_dir, "frames.csv"))

    source_plan = make_plan(clip, float(clip["fps"]))
    source_match = compare_plan(source_plan, source_oracle)
    if (source_match["position_m"] > 1e-4 or source_match["yaw_deg"] > 1e-3 or
            source_match["speed_mps"] > 1e-4 or source_match["time_s"] > 1e-5):
        raise RuntimeError(f"reconstructed source plan does not match oracle: {source_match}")

    high_rate_plan = make_plan(clip, args.fps)
    wheel_base = float(clip["vehicle"].get("axles", {}).get("wheel_base", 3.2))
    signal_eval = {
        "model": "signed-speed bicycle; centre integrated, converted to rear axle",
        "sample_time_semantics": "integrate each timestamped sample interval; equal timestamps add no motion",
        "video_frames": len(source_oracle),
        "signal_rate_hz": float(args.fps),
        "baseline_rate_hz": float(clip["fps"]),
        "baseline_10hz": evaluate_motion_samples(source_plan, source_oracle,
                                                   clip["vehicle"]["center_to_rear_axle"],
                                                   wheel_base),
        "high_rate": evaluate_motion_samples(high_rate_plan, source_oracle,
                                              clip["vehicle"]["center_to_rear_axle"],
                                              wheel_base),
    }
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "frames.csv"), "w", encoding="utf-8", newline="") as handle:
        handle.write(road_path.csv_text(high_rate_plan, mounts_from_clip(clip)))
    write_motion_samples(os.path.join(out_dir, "motion_samples.csv"), high_rate_plan)
    with open(os.path.join(out_dir, "signal_rate_evaluation.json"), "w",
              encoding="utf-8") as handle:
        json.dump(signal_eval, handle, indent=2)
        handle.write("\n")

    manifest = {
        "kind": "motion-only high-rate oracle; no video rendered",
        "source_clip": clip_dir,
        "source_video_fps": float(clip["fps"]),
        "motion_sample_fps": float(args.fps),
        "motion_sample_count": len(high_rate_plan.frames),
        "duration_s": high_rate_plan.duration,
        "distance_m": high_rate_plan.distance,
        "source_reconstruction_max_error": source_match,
        "signal_rate_evaluation": signal_eval,
        "video_pairing": False,
        "motion_samples_columns": ["sample", "time_s", "speed_mps", "gear", "steering_deg"],
        "motion_samples_steering": "raw front-wheel angle; reverse sign restored",
    }
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
