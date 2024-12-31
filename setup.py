# setup.py
from setuptools import setup, find_packages

setup(
    name="pyzlg_dexhand",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pyyaml",
        "ipython",
        "matplotlib",
        "pandas",
        "ros_compat @ git+https://gitee.com/dexrobot/ros_compat.git",
    ],
    extras_require={
        "test": ["pytest", "pytest-cov"],
        "docs": [
            "sphinx>=8.1.3",
            "sphinx-rtd-theme>=1.3.0",
            "sphinx-autodoc-typehints",
        ],
    },
    package_data={
        'pyzlg_dexhand': ['../lib/*'],
    },
    include_package_data=True,
)
