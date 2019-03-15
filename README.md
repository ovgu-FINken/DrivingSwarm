# Driving Swarm
All things driving swarm with global issue tracker and goals.

## Goals
* Research swarm-robotic behaviour with advanced robots
* Swarm Applications
  * Foraging
  * Collective Search
  * Dispersion/Formation
* Robotic Topics
  * World Representation in the Context of Swarm Robotics
  * Collective Trajectory Planning
* Swarm Topics
  * Integration of long term behaviour and short term planning
  * Integration of SI algorithms in robotics applications
  
 
## Milestones

* Basic Architecture
  * Multi-Master ROS
  * ROS-Node controlling the behaviour
  * Nav-Stack for robot navigation
  * Basic behaviour with random movement and collision avoidance
  * Central point of control for the swarm (starting and stopping the robots, task distribution)
  
  
* Simulation
  * Multiple robots in the same environment
  * Possibly not with multi-master ros but different prefixes per robot conrtoller
  * Same codebase as the real robots
  * Possible frameworks Gazebo (V-Rep, Argos)
  
* Unified World Representation
  * Have one world reresentation that works for the swarm
  * Semantic Maps, Local Dynamic Maps, ...
  * Needs to include - actions are based on representation not on physical sensors, distinguishes between static environment, dynamic environment and other robots/actors
  * Stretch-Goal - representation contains virtual objects (i.e. pheromones, semantic information)
  
* Collective Path Planning
  * Solve movement conflicts by collectively agreeing on a trajectory for each robot in a neighbourhood
  * Different Strategies for solving conflicts based on neighbourhood size and interconnectedness of the swarm

* Full 30 robot swarm
  * Buy and assemble another 27 robots
  
* Implement Collective Behaviour
  * Formation Behaviours
  * Foraging (Indian Cross)
  * Collective Search (Minimize Distance to Cable Car)

## Stucture of the Repository

* **All** packages required for the Driving Swarm robot should be included as submodules
* TODO: Make a tutorial on how to include the set up the files to get a worgking robot