"""Remote LiDAR Point-Cloud Snapshot Service.

Modules:
    decoder       - PointCloud2 -> numpy decoder (no ROS import, unit-testable)
    store         - latest-message holder + immutable snapshot capture
    ros_subscriber- rclpy subscriber node running in a background thread
    renderer      - Open3D headless renderer with Matplotlib fallback
    jobs          - bounded snapshot job/registry with TTL cleanup
    api           - FastAPI application
    config        - configuration (env vars + optional JSON file)
"""

__version__ = "0.1.0"
