# VR-PPL: Video-based Robot Pick & Place Learning

## Team Members

* Anudeep Sajja
* Chaitanya Gumudala
* Hamsa Datta Perur
* Vamsi Krishna Kalagaturu

## Project Description

Our project, VR-PPL, aims to address the challenge of enabling Kinova Gen3 robots to autonomously perform pick and place tasks through robot learning and computer vision.

* Problem description
  *  The problem we are tackling involves the fundamental robotic task of picking objects from one location and placing them in another. While this may seem simple to humans, it remains a complex challenge for robots due to the variability in object shapes, sizes, and placements, making it a quintessential problem in robotics. Furthermore, robots often need to adapt to dynamic and unstructured environments, making it essential to teach them the skill of picking and placing in a versatile manner. 
    
* Relevance
  *  This problem is highly relevant for robotics for several reasons. First, automation and robotics have become integral to the manufacturing, logistics, and healthcare industries. Efficient pick-and-place operations can significantly enhance productivity and reduce operational costs in these sectors. Secondly, robots can play a vital role in tasks that are dangerous or impractical for humans, such as handling hazardous materials or repetitive tasks, further highlighting the relevance of solving this problem.
    
* Why learning might be a suitable solution to the problem?
  * Learning-based approaches are particularly suitable for addressing this challenge. Learning allows robots to adapt and generalize from a wide range of scenarios and variations they encounter in the real world. By employing machine learning and computer vision, we can teach the Kinova Gen3 robot to recognize and adapt to different objects, pick them up with precision, and place them in a desired location with dexterity. The ability to learn from videos makes the system flexible and versatile, as it can continuously improve its performance based on real-world experiences, ultimately leading to more capable and autonomous robots in pick-and-place applications.

## Learning Techniques

* Our approach is inspired by the DexMV pipeline [1]. The authors use imitation and reinforcement learning techniques to learn dexterous manipulation from videos for simple pick-and-place tasks. 
* In our project, we would like to extend their approach to a robot manipulator. The authors only consider the hands for manipulation. Instead, we plan to consider the complete arm movements of humans during a pick-and-place task and let the manipulator learn the motions.
* Further, we plan to combine that with reinforcement learning to generalize the task for various objects and their locations.
* Our objectives are twofold:
  - Learn a policy for picking and placing any object in a target location through expert demonstrations and interactions with the environment.
  - Extend the policy to stack a cube on top of another cube.

## Learning Data

* We plan to collect our own data to reduce the time to adapt to other datasets.
* We will collect videos of humans picking a cube from a table and placing it in a predefined position on the table.
* The videos will include the usage of only one arm that demonstrates this pick-and-place action.
* We plan to use an RGB-D camera to collect these videos. Depth data is also recorded as part of the demonstration.   
* Then, we will extract the key points required from the videos and map them to the Kinova Gen3 arm.

## Expected Project Outcomes
* A simulation environment consisting of the Kinova Gen3 arm in MuJoCo and its control.
* A trained policy of pick-and-place tasks learned through passive observations from humans and interactions with the environment.
* Policy transfer to a real robot (sim-to-real), thus dealing with the embodiment mismatch problem.* 

## Evaluation Plan

* Given that any object (from the chosen dataset) is within the manipulator's workspace and can be grasped, the robot should pick it up and place it in a desired target location in the simulation.
* Given any two colored cubes on the table, the robot should pick any cube and place it in a target location, and then it should pick another cube and stack it on top of the first cube in the simulation.
* Execute the learned policy (pick-and-place or cube stacking) to the real robot. 


## References

[1] - Qin, Y., Wu, Y. H., Liu, S., Jiang, H., Yang, R., Fu, Y., & Wang, X. (2022, October). Dexmv: Imitation learning for dexterous manipulation from human videos. In European Conference on Computer Vision (pp. 570-587). Cham: Springer Nature Switzerland.
[2] - https://mujoco.org/

