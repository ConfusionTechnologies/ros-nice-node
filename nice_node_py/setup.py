from setuptools import setup

package_name = "nice_node_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=["nice_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="John-Henry Lim",
    maintainer_email="42513874+Interpause@users.noreply.github.com",
    description="webserver-inspired ROS2 API",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
