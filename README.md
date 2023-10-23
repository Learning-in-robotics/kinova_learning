# Kinova_learning
Learning simple tasks with Kinova Gen3 robot

## Setup

### Clone 

```bash
mkdir -p ~/kinova_learning && cd ~/kinova_learning

git clone --recurse-submodules -j8 git@github.com:Learning-in-robotics/kinova_learning.git .
```

### Create virtual environment

- Create a virual environment with python >=3.10

```bash
cd ~/kinova_learning

conda create -n venv_name -f environment.yml
```

### Install the packages
```bash
cd ~/kinova_learning

(venv) pip install -e . --use-pep517
```

- `--use-pep517` is used to inlucde all the data files in the package in editable install.

## Test

### To load the kinova arm in mujoco using robosuite

```bash
cd ~/kinova_learning

(venv) python3 kinova_gym/kinova_gym/utils/robosuite_test.py
```