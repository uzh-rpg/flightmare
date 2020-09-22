# Flightmare

![Build Status](https://github.com/uzh-rpg/flightmare/workflows/CPP_CI/badge.svg) ![clang format](https://github.com/uzh-rpg/flightmare/workflows/clang_format/badge.svg)

**Flightmare** is a flexible modular quadrotor simulator.
Flightmare is composed of two main components: a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation.
Those two components are totally decoupled and can run independently from each other. 
Flightmare comes with several desirable features: (i) a large multi-modal sensor suite, including an interface to extract the 3D point-cloud of the scene; (ii) an API for reinforcement learning which can simulate hundreds of quadrotors in parallel; and (iii) an integration with a virtual-reality headset for interaction with the simulated environment.
Flightmare can be used for various applications, including path-planning, reinforcement learning, visual-inertial odometry, deep learning, human-robot interaction, etc.

**[Website](https://uzh-rpg.github.io/flightmare/)** 

[![IMAGE ALT TEXT HERE](./docs/flightmare_main.png)](https://youtu.be/m9Mx1BCNGFU)

## Installation
Installation instructions can be found in our [Wiki](https://github.com/uzh-rpg/flightmare/wiki).
  
## Updates
 *  04.09.2020 Release Flightmare

## Publication

If you use this code in a publication, please cite the following paper **[PDF](https://arxiv.org/abs/2009.00563)**

```
@article{yunlong2020flightmare,
  title={Flightmare: A Flexible Quadrotor Simulator},
  author={Song, Yunlong and Naji, Selim and Kaufmann, Elia and Loquercio, Antonio and Scaramuzza, Davide},
  journal={arXiv preprint arXiv:2009.00563},
  year={2020}
}
```

## License
This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
