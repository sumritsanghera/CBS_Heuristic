# Conflict Based Search with Hueristics

This project implements Conflict Based Search (CBS) with the Conflict Graph, Dependency Graph, and Weighted Dependency Graph Heuristics.

## Installation

This project requires a Python 3 installation with the numpy and matplotlib packages. On Ubuntu
Linux, download python by using:

```bash
sudo apt install python3 python3-numpy python3-matplotlib
```
On Mac OS X, download Anaconda 2019.03 with Python 3.7 from [Anaconda](https://www.anaconda.com/distribution/#download-section/) and follow the installer. You can verify your installation by
using:
```bash
python3 --version
```
Before running, please use the package manager [pip](https://pip.pypa.io/en/stable/) to install the following required packages:

```bash
$ pip install networkx
```

## Running

The project is run by the following command:

```bash
python run_experiments.py --instance instances/x.txt --solver CBS
```
where "x" is the name of the instance you would like to run

Please note that independently created test instances are in instances\test_instances folder.

In order to change the heuristic, please go to line 145 of cbs.py and change `heuristic_option` as per instructions listed.
