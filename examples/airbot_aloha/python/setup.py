from distutils.core import setup
from setuptools import find_packages

setup(
    name="airbot_aloha",
    version="2.9.0",
    packages=find_packages(),
    package_dir={"": "."},
    install_requires=["numpy>=1.20.0", "h5py-cache", "opencv-python", "h5py"],
    license="MIT License",
    author="Discover Robotics",
    description="Used for demonstrate and replay.",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
