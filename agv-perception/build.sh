ROOT=$(pwd)
cd ../../
rm -rf build/ install/ log/
colcon build
cd ${ROOT}
