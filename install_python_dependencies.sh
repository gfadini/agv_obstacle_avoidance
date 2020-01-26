#!/bin/sh

# Author : Gabriele Fadini
# Installs the dependencies for the AGV simulation POSIX compliant

command -v python 2>&1 || { echo >&2 "I require 3.x but cannot find it in your path"; exit 1; }
command -v pip 2>&1 || {echo >&2 "pip is required to run this installer"; exit 1; } 

echo 'Installing the python requirements'

python -m pip install scipy --user
python -m pip install numpy --user
python -m pip install matplotlib --user
python -m pip install networkx --user
python -m pip install sympy --user
python -m pip install terminalplot --user
python -m pip install casadi --user

echo 'Checking other optional dependencies'
command -v ffmpeg 2>&1 || { echo >&2 "I require ffmpeg but cannot find it in your path."; exit 1; }
echo 'ffmpeg found OK'

echo 'All depencencies resolved, hurray!'
