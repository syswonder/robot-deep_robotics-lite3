# Lite3 simulated lidar provider

This provider registers the LaserScan and PointCloud2 topics already produced
by the MuJoCo bridge. It validates topic type and activity during Robonix
initialization and never starts a device driver or opens a hardware endpoint.
