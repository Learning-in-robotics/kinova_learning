# Kinova_learning
Learning simple tasks with Kinova Gen3 robot

## Setup

### Clone 

```bash
mkdir -p ~/kinova_learning && cd ~/kinova_learning

git clone git@github.com:Learning-in-robotics/Kinova_learning.git
```

### Create virtual environment

- Create a conda environment with python >=3.10


### Install the packages
```bash
cd ~/kinova_learning

pip install -e .
```

## Test

### To load the kinova arm in pybullet

```bash
cd ~/kinova_learning

python3 kinova_gym/utils/load_robot_test.py
```

### To load the kinova arm in mujoco

```bash
cd ~/kinova_learning/kinova_gym

python3 utils/load_robot_mujoco.py
```