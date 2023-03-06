# Sensing Aruco

A Python package for detecting aruco markers.

## Prerequisites

The following lines provide the Python prerequisites for Python 3.

1) Uninstall opencv-python if it is installed. Having opencv-python and opencv-contrib-python will conflict with each other.
```
# Use the following for Python 3.
python3 -m pip uninstall opencv-python
```

2) Install the following Python packages.
```
# Use the following for Python 3.
python3 -m pip install opencv-contrib-python>=4.6.0.66 scipy>=1.9.0 --user
```

## Installation

Perform the following steps to install the sense-aruco repository.
1) Clone the sensing-aruco repository.
```
cd <repos dir>
git clone https://github.com/troiwill/sensing-aruco
```

2) Build and install the Python package.
```
cd sensing-aruco
python3 -m build
python3 -m pip install dist/*.whl
```
