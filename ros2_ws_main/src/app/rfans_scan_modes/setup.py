# claude: 2026-10-01 新設
from setuptools import setup

package_name = "rfans_scan_modes"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reRoBot",
    maintainer_email="s25c1124xr@chibatech.ac.jp",
    description="3D LiDAR point cloud to LaserScan with selectable per-bin reduction",
    license="MIT",
    entry_points={"console_scripts": ["scan_modes = rfans_scan_modes.node:main"]},
)
