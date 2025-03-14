from setuptools import find_packages, setup

package_name = "system_health"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "psutil"],
    zip_safe=True,
    maintainer="Andrew Johnson",
    maintainer_email="andrewmjohnson549@gmail.com",
    description="A package containing a node for monitoring the system health of the robot computer",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_monitor_node = system_health.system_monitor_node:main"
        ],
    },
)
