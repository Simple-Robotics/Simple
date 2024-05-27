# The Simple Simulator: Simulation Made Simple

**Simple** is a new (differentiable) physical engine based on recent progress on solving contact simulation and leveraging [Pinocchio](https://github.com/stack-of-tasks/pinocchio) for fast dynamics computations and [HPP-FCL](https://github.com/humanoid-path-planner/hpp-fcl/) for efficient collision detection.
While first targetting robotics applications, **Simple** can be exploited in many other contexts: video games, system design, graphical animations, biomechanics, etc.

**Simple** is developed by the [WILLOW team](https://www.di.ens.fr/willow/) at [Inria](https://www.inria.fr/en).
The code associated with **Simple** will be released as open-source in the next few weeks. Stay tuned.

## The core team

The following persons actively took part in the development of **Simple**:
- [Justin Carpentier](https://jcarpent.github.io/) (Inria): core developer and project instigator
- [Quentin Le Lidec](https://quentinll.github.io/) (Inria): core developer
- [Louis Montaut](https://lmontaut.github.io/) (Inria/CTU): core developer
- [Joris Vaillant](https://github.com/jorisv/) (Inria): core developer

External contributions are more than welcome. If you have contributed to the development of Simple, feel free to add your name.

## Associated scientific and technical publications

**Simple** is built on active research around understanding and enhancing physical simulation. 
Interested readers can learn more about the algorithmic and computational foundations of **Simple** by reading these publications:

- Le Lidec, Q., Montaut, L. & Carpentier, J. (2024, July). [From Compliant to Rigid Contact Simulation: a Unified and Efficient Approach](https://hal.science/hal-04588906). In RSS 2024-Robotics: Science and Systems.
- Montaut, L., Le Lidec, Q., Petrik, V., Sivic, J., & Carpentier, J. (2024). [GJK++: Leveraging Acceleration Methods for Faster Collision Detection](https://hal.science/hal-04070039/). IEEE Transactions on Robotics.
- Sathya, A., & Carpentier, J. (2024). [Constrained Articulated Body Dynamics Algorithms](https://hal.science/hal-04443056/). Under review.
- Montaut, L., Le Lidec, Q., Bambade, A., Petrik, V., Sivic, J., & Carpentier, J. (2023, May). [Differentiable collision detection: a randomized smoothing approach](https://hal.science/hal-03780482/). In 2023 IEEE International Conference on Robotics and Automation (ICRA).
- Le Lidec, Q., Jallet, W., Montaut, L., Laptev, I., Schmid, C., & Carpentier, J. (2023). [Contact models in robotics: a comparative analysis](https://hal.science/hal-04067291/). Under review.
- Montaut, L., Le Lidec, Q., Petrik, V., Sivic, J., & Carpentier, J. (2022, June). [Collision Detection Accelerated: An Optimization Perspective](https://hal.science/hal-03662157/). In Robotics: Science and Systems (RSS 2O22).
- Carpentier, J., Budhiraja, R., & Mansard, N. (2021, July). [Proximal and sparse resolution of constrained dynamic equations](https://hal.science/hal-03271811/). In Robotics: Science and Systems (RSS 2021).
- Carpentier, J., & Mansard, N. (2018, June). [Analytical derivatives of rigid body dynamics algorithms](https://hal.science/hal-01790971/). In Robotics: Science and systems (RSS 2018).
