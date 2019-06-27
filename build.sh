cd thirdparty/g2o/
mkdir build
cd build
cmake ..
make -j4
cd ..
cd ..

cd gflags
mkdir build 
cd build
export CXXFLAGS="-fPIC" && cmake .. && make VERBOSE=1
make -j4
cd ..
cd ..

cd glog
mkdir build 
cd build
cmake ..
make -j4
cd ..
cd ..
cd ..

mkdir build
cd build
cmake ..
make -j4
cd ..


