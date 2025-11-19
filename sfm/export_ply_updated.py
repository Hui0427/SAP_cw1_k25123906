import numpy as np
import os
import json
from datetime import datetime


def find_latest_reconstruction():
    """Find the latest SfM reconstruction .npy file and its metadata."""
    results_dir = "/home/xinyue/ros2_ws/src/sfm/results"

    # Find all .npy files with prefix 'sfm_points_'
    npy_files = [
        f for f in os.listdir(results_dir)
        if f.endswith(".npy") and f.startswith("sfm_points_")
    ]
    if not npy_files:
        raise FileNotFoundError("No reconstruction .npy files found")

    # Sort by filename (timestamp in the name) and take the latest one
    latest_npy = sorted(npy_files)[-1]
    latest_timestamp = latest_npy.replace("sfm_points_", "").replace(".npy", "")

    # Corresponding metadata file
    metadata_file = f"sfm_metadata_{latest_timestamp}.json"

    return (
        os.path.join(results_dir, latest_npy),
        os.path.join(results_dir, metadata_file),
    )


def load_and_process_points(points_path, metadata_path):
    """Load and inspect the reconstructed point cloud and its metadata."""
    # Load point cloud
    points = np.load(points_path)
    print(f"Raw point cloud shape: {points.shape}")

    # If points are homogeneous (N x 4), drop the last coordinate
    if points.shape[1] == 4:
        points = points[:, :3]
        print("Converted from homogeneous to 3D coordinates")

    # Load metadata
    with open(metadata_path, "r") as f:
        metadata = json.load(f)

    print(
        f"Reconstruction info: {metadata['num_points']} points, "
        f"{metadata['num_poses']} camera poses"
    )
    print(f"Camera intrinsics: {metadata['camera_intrinsics']}")

    return points, metadata


def filter_and_scale_points(points):
    """
    Filter outliers and scale the point cloud to a workspace-friendly size.

    - Outlier removal: remove points beyond the 95th percentile distance
      from the centroid.
    - Scaling: rescale so the maximum extent is ~0.3 m.
    """
    # Compute centroid
    centroid = np.mean(points, axis=0)
    print(f"Point cloud centroid: {centroid}")

    # Distance of each point to centroid
    distances = np.linalg.norm(points - centroid, axis=1)

    # Remove outliers: keep points within 95th percentile distance
    distance_threshold = np.percentile(distances, 95)
    filtered_points = points[distances <= distance_threshold]
    print(
        f"After outlier removal: {len(filtered_points)} points "
        f"(removed {len(points) - len(filtered_points)} outliers)"
    )

    # Scale the point cloud so its largest dimension is about 0.3 m
    current_range = np.ptp(filtered_points, axis=0)  # peak-to-peak range
    max_range = np.max(current_range)

    if max_range > 0:
        scale_factor = 0.3 / max_range
        scaled_points = filtered_points * scale_factor
        print(f"Scale factor: {scale_factor:.4f}")
        print(
            "Scaled bounds: "
            f"X[{np.min(scaled_points[:, 0]):.3f}, {np.max(scaled_points[:, 0]):.3f}], "
            f"Y[{np.min(scaled_points[:, 1]):.3f}, {np.max(scaled_points[:, 1]):.3f}], "
            f"Z[{np.min(scaled_points[:, 2]):.3f}, {np.max(scaled_points[:, 2]):.3f}]"
        )
    else:
        scaled_points = filtered_points
        print("Point cloud range is too small, skipping scaling.")

    return scaled_points


def save_ply(points, filename):
    """Save a 3D point cloud to an ASCII PLY file."""
    with open(filename, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def main():
    try:
        # Locate the latest reconstruction files
        points_path, metadata_path = find_latest_reconstruction()
        print(f"Using latest reconstruction file: {os.path.basename(points_path)}")

        # Load and inspect the point cloud
        points, metadata = load_and_process_points(points_path, metadata_path)

        # Filter and scale the point cloud
        processed_points = filter_and_scale_points(points)

        # Output filenames (PLY + processed NPY) based on metadata timestamp
        timestamp = metadata["timestamp"]
        output_dir = "/home/xinyue/ros2_ws/src/sfm/results"

        output_ply = os.path.join(output_dir, f"processed_sfm_points_{timestamp}.ply")
        save_ply(processed_points, output_ply)
        print(f" Saved PLY file: {output_ply}")

        output_npy = os.path.join(output_dir, f"processed_sfm_points_{timestamp}.npy")
        np.save(output_npy, processed_points)
        print(f" Saved processed point cloud (npy): {output_npy}")

        # Centroid for grasping demo
        centroid = np.mean(processed_points, axis=0)
        print("\n Object pose for grasping demo:")
        print(f"   Object centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")
        print(
            "   Suggested grasping position: "
            f"[{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}] "
            "+ robot base offset"
        )

    except Exception as e:
        print(f" Error: {e}")
        print("Please check:")
        print("1. Reconstruction result files exist")
        print("2. File paths are correct")
        print("3. Numpy array format is valid")


if __name__ == "__main__":
    main()

