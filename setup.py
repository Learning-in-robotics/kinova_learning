from setuptools import setup, find_packages

# install Kinova_learning and robosuite packcages as editable
setup(name='kinova_learning',
      version='0.0.1',
      install_requires=['gymnasium', 'pybullet', "mujoco", "tqdm", "tensorboard", "attrdict"],
      eager_resources=["*"],
      include_package_data=True,
      package_dir={'':'.', 'kinova_gym': 'kinova_gym/kinova_gym', 'robosuite': 'robosuite/robosuite'},
      packages=find_packages("kinova_gym") + [package for package in find_packages("robosuite") if package.startswith("robosuite")],
)