# Crowd Simulator

A crowd simulator API with a default demo scene. The main python API can be used in a few ways. There is an examples folder showing a few use-cases. Users can also switch between social forces and PAM as the crowds algorithm. 

The extension will load an populate some demo configuration. It will create an Xform called CrowdGoals.  To add a goal for the crowd. Create an xform under the "CrowdGoals". We automatically check for those prims as the method of determing crowd goals.  For every additional xform under CrowdGoals, we evenly split the crowd to assign those goals. 

There are two options for the crowd objects. The default uses geompoints, which is faster and runs its own integration of the forces for position and velocity. It does not interact with any physics in the scene.  Alternatively the "Rigid Body" can be used, which creates physical spheres that interact with the scene.  

Press Play to see the crowd. 

The default number of demo agents is a 3x3 grid (9 agents). The current simulator in python may struggle above 25 agents, depending on CPU configuration. 

We plan to support more methods in the future, as well as more crowd simulators.  Contributions are welcome.
