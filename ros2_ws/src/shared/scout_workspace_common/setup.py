from setuptools import find_packages, setup


package_name = "scout_workspace_common"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="scout_offroad",
    maintainer_email="todo@todo.todo",
    description="Shared workspace discovery and runtime-path helpers for Scout navigation packages.",
    license="BSD",
)
