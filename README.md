# Motion and Structure from Event-based Normal Flow

This repository delivers **EvLinearSolver**, an event-based stereo visual-inertial odometry system built on top of our previous work ESVO [3]. It is a direct method that solves the tracking and mapping problems in parallel by leveraging the spatio-temporal coherence in the stereo event data. It alleviates ESVO's high computational complexity in mapping and address its degeneracy in camera pose tracking. To the best of our knowledge, the system is the first published work that achieves real-time performance using a standard CPU on event cameras of VGA pixel resolution. 

### **Video**

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/2RzSa9pC-7A/mqdefault.jpg)](https://www.youtube.com/watch?v=2RzSa9pC-7A) &nbsp;&nbsp; 

### **Related Publications**

[1] **[Motion and Structure from Event-based Normal Flow](http://arxiv.org/abs/2407.12239)**, *Zhongyang Ren, Bangyan Liao, Delei Kong, Jinghang Li, Peidong Liu, Laurent Kneip, Guillermo Gallego, Yi Zhou*, ECCV 2024. [PDF](http://arxiv.org/abs/2407.12239), [Video](https://www.youtube.com/watch?v=2RzSa9pC-7A).




# 1. Installation

We have tested EvLinearSolver on machines with the following configurations

* Windows 10/11 + MATLAB R2022a

**Clone this repository** into your matlab workspace.


## 1.1 Dependencies

You may need add path to 'yamlmatlab' for hyperparameter settings

```
addpath(genpath('D:\Code\Mat\Utils')); % path to repo yamlmatlab
```

If you don't have this repo, please download it from https://github.com/ewiger/yamlmatlab


# 2. Usage

- 
- 

# 3. Notes for Good Results

- Real-time performance is witnessed on a desktop with an Intel Core i7-14700k CPU. 

* To get real-time performance, you need a powerful PC with modern CPUs which supports at least 6 threads. 
  Remember to keep you computer cool!

* 
