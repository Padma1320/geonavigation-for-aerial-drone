from setuptools import find_packages, setup

package_name = "geonav_geospatial_bridge"

setup(
    name=package_name,
    version="0.0.2",

    packages=find_packages(),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],

    install_requires=["setuptools"],
    zip_safe=True,

    maintainer="padma",
    maintainer_email="padma@example.com",

    description=(
        "GeoNav-AI integrated geospatial semantic "
        "segmentation, landing candidate selection, "
        "and ROS2 mission-goal interface"
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [

            # Existing JSON -> ROS goal bridge
            "landing_goal_publisher = "
            "geonav_geospatial_bridge."
            "landing_goal_publisher:main",

            # NEW:
            # aerial image -> trained segmentation ->
            # candidate selection -> ROS goal
            "geospatial_ai_mission = "
            "geonav_geospatial_bridge."
            "geospatial_ai_mission:main",
        ],
    },
)
