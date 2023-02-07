from setuptools import setup, find_packages

setup(
    packages=find_packages(),
    name="pyintercom",
    version="0.1.0",
    install_requires=["pyserial", "pyyaml"]
)
