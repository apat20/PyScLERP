# PyScLERP
Python code to compute a path in SE(3), given an initial and a final pose, using Screw Linear Interpolation (ScLERP). 


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

**NOTE:** This repository is under-development and we plan to add more functionality and examples where the task space path consists of multiple constant screw motions. For further inquiries contact:  Aditya Patankar (aditya.patankar@stonybrook.edu)

If you find this work useful please site our papers: 

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
@inproceedings{fakhari2021motion,
  title={Motion and force planning for manipulating heavy objects by pivoting},
  author={Fakhari, Amin and Patankar, Aditya and Chakraborty, Nilanjan},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={9393--9400},
  year={2021},
  organization={IEEE}
}
```

# References: 

```
1. Sarker, Anik, Anirban Sinha, and Nilanjan Chakraborty. "On screw linear interpolation for point-to-point path planning." 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2020.

2. Fakhari, Amin, Aditya Patankar, and Nilanjan Chakraborty. "Motion and force planning for manipulating heavy objects by pivoting." 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2021.

3. Murray, Richard M., Zexiang Li, and S. Shankar Sastry. A mathematical introduction to robotic manipulation. CRC press, 2017.

4. Jia, Yan-Bin. "Dual quaternions." Iowa State University: Ames, IA, USA (2013): 1-15.
```
