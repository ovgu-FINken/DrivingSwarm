# Driving Swarm
All things driving swarm with global issue tracker and goals.

## Goals
* Research swarm-robotic behaviour with robots capable of mapping and planning
* Swarm Applications
  * Foraging
  * Collective Search
  * Dispersion/Formation
* Robotic Topics
  * World Representation in the Context of Swarm Robotics
  * Planning agents within swarms (Cooperative planning, emergent effects in plans and plan execution)
  * Localisation in Robotic Swarms
* Swarm Topics
  * Integration of long term behaviour and short term planning
  * Integration of SI algorithms in robotics applications
  * Swarms with diveristy in the behaviour
  
 
## Milestones

- Basic Architecture (work in progress)
  - [ ] Multi-Master ROS
  - [ ] ROS-Node controlling the behaviour
  - [x] Nav-Stack for robot navigation
  - [ ] Basic behaviour with random movement and collision avoidance
  - [x] Central point of control for the swarm (starting and stopping the robots, task distribution)
  - [ ] Optional Goal- Integrate Localisation
  
- Simulation (currently working with Gazebo, slow for many robots)
  - [x] Multiple robots in the same environment
  - [x] Possibly not with multi-master ros but different prefixes per robot conrtoller
  - [x] Same codebase as the real robots
   
* Unified World Representation
  * Have one world reresentation that works for the swarm
  * Semantic Maps, Local Dynamic Maps, ...
  * Needs to include - actions are based on representation not on physical sensors, distinguishes between static environment, dynamic environment and other robots/actors
  * Stretch-Goal - representation contains virtual objects (i.e. pheromones, semantic information)
  * Gossiping protocols for spacial information
  
* Collective Path Planning
  * Solve movement conflicts by collectively agreeing on a trajectory for each robot in a neighbourhood
  * Different Strategies for solving conflicts based on neighbourhood size and interconnectedness of the swarm
  * See how different planners can deal with one scenario
  
  
* Localisation
  * Multilateration
  * Camera Tracking
  * Sensor fusion
  * AMCL with MAP (localisation part of SLAM)
  
* Full 30 robot swarm
  * Buy and assemble another 27 robots
  * currently 3 (plus 10) 
  
* Implement Collective Behaviour
  * Formation Behaviours
  * Foraging (Indian Cross)
  * Collective Search (e.g. Minimize Distance to Cable Car)

## Stucture of the Repository

* **All** packages required for the Driving Swarm robot should be included as submodules
* gitman can be used to check out submodules
