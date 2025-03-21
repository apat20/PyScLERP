# PyScLERP
Python code to compute a path in SE(3), given an initial and a final pose, using Screw Linear Interpolation (ScLERP) also known as Dual Quaternion Interpolation. Examples of using ScLERP for computing a path in SE(3) for the tasks of pivoting and sliding a cuboid can be seen below:
![](https://github.com/apat20/PyScLERP/blob/main/gifs/PyScLERP_gifs_v1.gif)

Please note that both the examples above are single constant screw motions. The key advantage of using Screw Linear Interpolation (ScLERP) is that it allows to satisfy the task-related constraints kinematically. For example, while pivoting a cuboid about one of its edges, we have to ensure that edge maintains contact with the environment throughout the motion. 

More generally, as per Chasles' theorem, any rigid body motion in SE(3) can be approximated arbitrarily closely by a sequence of constant screw motions [3].
For more details please refer to our papers and if you find them useful please cite our work: 

```
@inproceedings{sarker2020screw,
  title={On screw linear interpolation for point-to-point path planning},
  author={Sarker, Anik and Sinha, Anirban and Chakraborty, Nilanjan},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={9480--9487},
  year={2020},
  organization={IEEE}
}
```

```
@inproceedings{laha2021point,
  title={Point-to-point path planning based on user guidance and screw linear interpolation},
  author={Laha, Riddhiman and Rao, Anjali and Figueredo, Luis FC and Chang, Qing and Haddadin, Sami and Chakraborty, Nilanjan},
  booktitle={International Design Engineering Technical Conferences and Computers and Information in Engineering Conference},
  volume={85451},
  pages={V08BT08A010},
  year={2021},
  organization={American Society of Mechanical Engineers}
}
```

```
@inproceedings{fakhari2021motion,
  title={Motion and force planning for manipulating heavy objects by pivoting},
  author={Fakhari, Amin and Patankar, Aditya and Chakraborty, Nilanjan},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={9393--9400},
  year={2021},
  organization={IEEE}
}
```

```
@inproceedings{mahalingam2023human,
  title={Human-guided planning for complex manipulation tasks using the screw geometry of motion},
  author={Mahalingam, Dasharadhan and Chakraborty, Nilanjan},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={7851--7857},
  year={2023},
  organization={IEEE}
}
```

```
@inproceedings{patankar2023task,
  title={Task-oriented grasping with point cloud representation of objects},
  author={Patankar, Aditya and Phi, Khiem and Mahalingam, Dasharadhan and Chakraborty, Nilanjan and Ramakrishnan, IV},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={6853--6860},
  year={2023},
  organization={IEEE}
}
```

# Installation 

1. Clone the repository

```
git clone git@github.com:apat20/PyScLERP.git
```

2. Create a conda environment using provided .yml file

```
cd PyScLERP
conda env create -f environment.yml
```

# Usage

Current implementation contains to sample tasks, pivoting a cuboid about one of its edges and sliding a cuboid, both while maintaining continuous contact with the environment. Both the pivoting and the sliding motions are single constant screw motions. The initial and final poses along with the dimensions of the cuboid can be found in ``` config/ ``` 

1. Open a terminal and activate the conda environment

```
conda activate pysclerp
```

2. Type the following command to execute and visualize the results of pivoting the cuboid:
   
``` 
python main_pivoting.py 
```

3. Type the following command to execute and visualize the results of sliding the cuboid:
   
``` 
python main_sliding.py 
```

4. Type the following command to execute and visualize the results of the plan skeletion 1 - {pivot, pick-up, transfer, place}

``` 
python main_skeleton.py 
```

**NOTE:** This repository is under-development and we plan to add more functionality and examples where the task space path consists of multiple constant screw motions. For further inquiries contact:  Aditya Patankar (aditya.patankar@stonybrook.edu)

# References: 

1. Sarker, Anik, Anirban Sinha, and Nilanjan Chakraborty. "On screw linear interpolation for point-to-point path planning." 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020.

2. Fakhari, Amin, Aditya Patankar, and Nilanjan Chakraborty. "Motion and force planning for manipulating heavy objects by pivoting." 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2021.

3. Murray, Richard M., Zexiang Li, and S. Shankar Sastry. A mathematical introduction to robotic manipulation. CRC press, 2017.

4. Jia, Yan-Bin. "Dual quaternions." Iowa State University: Ames, IA, USA (2013): 1-15.

