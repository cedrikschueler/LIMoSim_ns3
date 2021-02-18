rm -rf build_release
mkdir build_release
cd build_release
qmake ../ui/LIMoSim.pro -spec linux-g++ exts=ns3
make qmake_all
make -j8
