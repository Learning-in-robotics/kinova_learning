# VR-PPL: Video-based Robot Pick & Place Learning

## Team Members

* Anudeep Sajja
* Chaitanya Gumudala
* Hamsa Datta Perur
* Vamsi Krishna Kalagaturu

## Project Description

Our project, VR-PPL, aims to address the challenge of enabling Kinova Gen3 robots to autonomously perform pick and place tasks through robot learning and computer vision.

* Problem description
  *  The problem we are tackling involves the fundamental robotic task of picking objects from one location and placing them in another location. While this may seem simple to humans, it remains a complex challenge for robots due to the variability in object shapes, sizes, and placements, making it a quintessential problem in robotics. Furthermore, robots often need to adapt to dynamic and unstructured environments, making it essential to teach them the skill of pick and place in a versatile manner. 
    
* Relevance
  *  This problem is highly relevant for robotics for several reasons. First, automation and robotics have become integral to industries such as manufacturing, logistics, and healthcare. Efficient pick and place operations can significantly enhance productivity and reduce operational costs in these sectors. Secondly, robots can play a vital role in tasks that are dangerous or impractical for humans, such as handling hazardous materials or repetitive tasks, further highlighting the relevance of solving this problem.
    
* Why learning might be a suitable solution to the problem?
  * Learning-based approaches are particularly suitable for addressing this challenge. Learning allows robots to adapt and generalize from a wide range of scenarios and variations they encounter in the real world. By employing machine learning and computer vision, we can teach the Kinova Gen3 robot to recognize and adapt to different objects, pick them up with precision, and place them in a desired location with dexterity. The ability to learn from videos makes the system flexible and versatile, as it can continuously improve its performance based on real-world experiences, ultimately leading to more capable and autonomous robots in pick and place applications.

## Learning Techniques


Briefly describe which concrete learning technique(s) you plan to use to achieve the goals of your project.

If you plan to adapt a concrete approach from the literature (e.g. you want to follow a specific paper in your implementation), provide a brief description of the approach and why it might be suitable for your problem.

Our approach is inspired from the DexMV pipeline [1]. The authors use imitation learing technique to learn dexterous manipulation from videos for simple pick and place task. In our proejct we would like to 

## Learning Data

* We plan to collect our own data to reduce the time to adapt to other datasets.
* We will collect videos of humans picking a cube from a table and placing it in a predefined position on the table.
* The videos will include the usage of only one arm that demonstrates this pick-and-place action.
* We plan to use an RGB-D camera to collect these videos. Depth data is also recorded as part of the demonstration.   
* Then, we will extract the key points required from the videos and map them to the Kinova Gen3 arm.

## Expected Project Outcomes
* A simulation environment consisting of the Kinova Gen3 arm in MuJoCo and its control.
* A trained policy of pick and place learned through passive observations.
* Policy transfer to a real robot (sim-to-real), thus dealing with the embodiment mismatch problem.* 

## Evaluation Plan

Shortly describe how you plan to evaluate the learning success in your problem (e.g. if you train a robot policy, how you will evaluate its generalisation capabilities).


## References

[1] - Qin, Y., Wu, Y. H., Liu, S., Jiang, H., Yang, R., Fu, Y., & Wang, X. (2022, October). Dexmv: Imitation learning for dexterous manipulation from human videos. In European Conference on Computer Vision (pp. 570-587). Cham: Springer Nature Switzerland.
[2] - https://mujoco.org/

