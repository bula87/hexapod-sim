cd bullet3
mkdir build_dir
cd build_dir
cmake ..
make
cd ../../
ln -s bullet3/build_dir/Demos/Hexapod .
cp bullet3/Demos/Hexapod/config.txt ./Hexapod/.
