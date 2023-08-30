OV_PATH=`pwd`

clear && clear

echo -e "\n\nbuild ************************************************************"
cd ${OV_PATH}/ov_msckf/

if [ ! -d "build" ]; then
    mkdir build && cd build
    cmake -DENABLE_ROS=OFF ..
else
    cd build
fi

make -j`nproc`
# sudo make install

cd ${OV_PATH}

echo -e "\n\nrun **************************************************************"

# gdb --args \
# ${OV_PATH}/ov_msckf/build/run_stereo_inertial_euroc \
#     ${OV_PATH}/config/euroc_mav/estimator_config.yaml \
#     ~/Documents/EuRoC/MH_01_easy \
#     ~/Documents/EuRoC/MH_01_easy/MH01.txt

${OV_PATH}/ov_msckf/build/run_stereo_inertial_euroc \
    ${OV_PATH}/config/euroc_mav/estimator_config.yaml \
    ~/Documents/EuRoC/V1_01_easy \
    ~/Documents/EuRoC/V1_01_easy/V101.txt