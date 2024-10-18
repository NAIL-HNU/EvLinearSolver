# A Fast Linear Solver for Geometry and Motion Estimation on Event Data

<div align='right'>Neuromorphic Automation & Intelligence Laboratory</div>

### 1. Installation


We have tested linear solver on machines with the following configurations
* Win 11 + MATLAB R2022a 

### 2. Dependencies

You may need to load hyper parameters by 'yamlmatlab', which could be download from

[ewiger/yamlmatlab: Java-based implementation of YAML IO support in MATLAB. (github.com)](https://github.com/ewiger/yamlmatlab)

After this, you need to add the path of yamlmatlab folder to your MATLAB path by run the following code in MATLAB

``` matlab
addpath(genpath('C:\Code\Mat\yamlmatlab'))
```

### 3. Data preprocessing

For convenience, we provide the preprocessed data at:



You may need to revise the yaml file which is in the config filefolder to set the correct path on your computer



### 4. Test algorithm

Run main_realdata.m file for evaluation.

