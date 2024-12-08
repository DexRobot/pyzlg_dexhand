# setup.py
from setuptools import setup, find_packages

setup(
    name="pyzlg_dexhand",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pyyaml"
    ],
    extras_require={
        'test': [
            'pytest',
            'pytest-cov'
        ],
    }
)
