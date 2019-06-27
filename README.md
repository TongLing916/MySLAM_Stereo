# MySLAM_Stereo
## Note
This project was originally designed by Gao Xiang. See [slambook2](https://github.com/gaoxiang12/slambook2) for more information.

## Dependencies
1. OpenCV
2. Pangolin
3. Sophus
4. Eigen
5. gtest
6. g2o
7. glog
8. gflags

`g2o`, `glog`, `gflags` are already contained in the folder `thirparty`. In the project folder, use the following command to build the project and the contained third parties

```
./build.sh
```

## Run
Create your own `yaml` files and execute the command (replace the `config_file` with your own file)
```
./bin/run_kitti_stereo -config_file="./config/KITTI00_02.yaml"
```