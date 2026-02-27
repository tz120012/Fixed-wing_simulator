from setuptools import setup, find_packages

setup(
    name="fixed_wing_simulator",
    version="1.0.0",
    description="Professional Fixed-Wing UAV Simulation and Control Platform",
    author="Fixed-Wing Sim Team",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.10",
    install_requires=[
        "numpy>=1.24",
        "scipy>=1.11",
        "matplotlib>=3.7",
        "plotly>=5.18",
        "pyyaml>=6.0",
        "pandas>=2.0",
    ],
    extras_require={
        "dev": ["pytest>=7.4"],
    },
)
