# Motion and Structure from Event-based Normal Flow

This repository delivers **EvLinearSolver**, a linear solution for a series of geometric model fitting problems on event data.

### **Video**

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/2RzSa9pC-7A/mqdefault.jpg)](https://www.youtube.com/watch?v=2RzSa9pC-7A) &nbsp;&nbsp; 

### **Related Publications**

[1] **[Motion and Structure from Event-based Normal Flow](http://arxiv.org/abs/2407.12239)**, *Zhongyang Ren, Bangyan Liao, Delei Kong, Jinghang Li, Peidong Liu, Laurent Kneip, Guillermo Gallego, Yi Zhou*, ECCV 2024. [PDF](http://arxiv.org/abs/2407.12239), [Video](https://www.youtube.com/watch?v=2RzSa9pC-7A).

### Citation
If you find this code useful in your research, please cite this work by:

```
@InProceedings{Ren2024eccv,
      title     = {Motion and Structure from Event-based Normal Flow}, 
      author    = {Zhongyang Ren and Bangyan Liao and Delei Kong and Jinghang Li and Peidong Liu 
                  and Laurent Kneip and Guillermo Gallego and Yi Zhou},
      booktitle = {European Conference on Computer Vision (ECCV)},
      doi       = {},
      year      = {2024}
      }
```


# 1. Installation

We have tested EvLinearSolver on machines with the following configurations

* Windows 10/11 + MATLAB R2022a



## 1.1 Dependencies
You may need add path to 'yamlmatlab' for hyperparameter settings
```
addpath(genpath('D:\Code\Mat\Utils')); % path to repo yamlmatlab
```
If you don't have this repo, please download it from https://github.com/ewiger/yamlmatlab

## 1.2 Dataset
We have released calculated normal flow on serveral sequence which could be downloaded from .
- Rotational motion estimation: sequence shapes_rotation from ECD Dataset (IJRR, 2017).
- 6 DoF tracking estimation：sequence corner_slow1 from VECtor Dataset (RAL, 2021).

You can also calculate normal flow from your event data

# 2. Usage
Given input sparse normal flow, our linear solver could give

To run this code, you need **Clone this repository** into your matlab workspace.
```
git clone https://github.com/NAIL-HNU/EvLinearSolver.git
```

You need to revised path in corresponding yaml file to run your code. 
- Rotation Motion Estimation: You can run file **rotation_test** to check the result, this function output estimated angular velocity and groundtruth from IMU measurement.

- 6 DoF Tracking Estimation: You can run file **tracking_test** to check the result, this function output estimated angular velocity and linear velocity as well as groundtruth from IMU and Mocap System measurement.

- Differential Homography Estimation: will be updated soon.

# 3. Contact

If you have any questions or inquiries, please open an issue or email **Zhongyang Ren** at 
- Primary: zhongyangren@hnu.edu.cn
- Alternate: cs.zhongyang@gmail.com (in case your email is filtered out by the primary mailbox).



