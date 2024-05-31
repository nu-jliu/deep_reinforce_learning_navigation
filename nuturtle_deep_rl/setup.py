from setuptools import find_packages, setup
from glob import glob

package_name = "nuturtle_deep_rl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    py_modules=[(package_name + ".nuturtle_env")],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jliu",
    maintainer_email="JingkunLiu2025@u.northwestern.edu",
    description="Deep Reinforce Learning Implementation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["deep_rl_node = nuturtle_deep_rl.nuturtle_drl:main"],
    },
)
