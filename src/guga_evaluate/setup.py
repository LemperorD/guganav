from glob import glob
from setuptools import find_packages, setup


package_name = "guga_evaluate"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Transistor Team",
    maintainer_email="transistor@example.com",
    description="Robot-side navigation data recorder and evaluator for guganav.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "evaluate_node = guga_evaluate.evaluator_node:main",
            "visualize_node = guga_evaluate.visualizer_node:main",
        ],
    },
)
