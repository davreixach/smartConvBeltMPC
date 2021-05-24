# MPC for a Smart Conveyor Belt System

This repository provides code to reproduce results from the paper: [Model Based Predictive Control for a Smart Conveyor Belt System](https://github.com/davreixach/smartConvBeltMPC/blob/main/mpc_smart_conveyor_belt-reixach.pdf).

Here is the method summarized:

Formulation LMPC                         | Formulation NMPC  
-----------------------------------------|-----------------------------------------
<img src="https://github.com/davreixach/smartConvBeltMPC/blob/main/smartConvBeltMPClatex/images/problem.png" width="300"> <img src="https://github.com/davreixach/smartConvBeltMPC/blob/main/smartConvBeltMPClatex/images/equations1.png" width="300"> | <img src="https://github.com/davreixach/smartConvBeltMPC/blob/main/smartConvBeltMPClatex/images/equations2.png" width="400">
<!-- -----------------------------------------|----------------------------------------- -->
Results LMPC                             | Results NMPC    
-----------------------------------------|-----------------------------------------
<img src="https://github.com/davreixach/smartConvBeltMPC/blob/main/smartConvBeltMPClatex/images/nmpc_xy.png" width="400"> | <img src="https://github.com/davreixach/smartConvBeltMPC/blob/main/smartConvBeltMPClatex/images/lmpc_xy.png" width="400">


### Preliminaries
---

1. Clone the repository
    ```shell
    $ git clone https://github.com/davreixach/smartConvBeltMPC.git
    $ cd smartConvBeltMPC/code
    ```

2. Install ACADO Toolkit https://acado.github.io/index.html
