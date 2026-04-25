
git clone https://github.com/ceres-solver/ceres-solver.git

sudo cp /home/orangepi/drone/docs/ceres-solver/ceres-solver-2.0.0rc1.zip /home/orangepi/ceres-solver-2.0.0rc1.zip
unzip ceres-solver-2.0.0rc1.zip
sudo mv ceres-solver-2.0.0rc1 ceres-solver

cd ceres-solver
mkdir build && cd build
cmake ..
sudo make -j8
sudo make install
cd ../..