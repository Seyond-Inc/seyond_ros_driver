#!/bin/bash

if [ -z "$ROS_VERSION" ] || [ -z "$ROS_DISTRO" ]; then
  echo "ROS_VERSION or ROS_DISTRO is not set. Please source the ROS environment."
  exit 1
fi

CURRENT_DIR=$(cd $(dirname $0); pwd)
PROJECT_DIR=${CURRENT_DIR}
DEB_ROS_ROOT=opt/ros/${ROS_DISTRO}

echo -e "\n\033[1;32m-- (1). Check the build result...\033[0m"
# check the driver has been built 
if [[ -e "${PROJECT_DIR}/install/setup.sh" ]]; then
  echo "The driver has been built"
  source ${PROJECT_DIR}/install/setup.sh
  echo "ROS_VERSION: $ROS_VERSION"
  echo "ROS_DISTRO: $ROS_DISTRO"
  # update the DISTRO
  DEB_ROS_ROOT=opt/ros/${ROS_DISTRO}
else
  # build the driver
  echo "The driver has not been built, start to build the driver"
  echo "ROS_VERSION: $ROS_VERSION"
  echo "ROS_DISTRO: $ROS_DISTRO"
  ${PROJECT_DIR}/build.bash
fi

echo -e "\n\033[1;32m-- (2). Clean the output directory...\033[0m"
echo "The output directory is ${CURRENT_DIR}/output"
# result output
OUTPUT_DIR=${CURRENT_DIR}/output
rm -rf ${OUTPUT_DIR}
mkdir -p ${OUTPUT_DIR}
cd ${OUTPUT_DIR}

echo -e "\n\033[1;32m-- (3). Build the deb package...\033[0m"

# build in local
DEB_VERSION=1.0.0
SOFT_VER=${DEB_VERSION}

# build deb
pushd ${OUTPUT_DIR}
rm -fr deb_root
mkdir -p deb_root/DEBIAN
mkdir -p deb_root/${DEB_ROS_ROOT}

if [ "$ROS_VERSION" == "1" ]; then
  cp -rp ${PROJECT_DIR}/install/share deb_root/$DEB_ROS_ROOT/
  cp -rp ${PROJECT_DIR}/install/lib deb_root/$DEB_ROS_ROOT/
elif [ "$ROS_VERSION" == "2" ]; then
  cp -rp ${PROJECT_DIR}/install/seyond/share deb_root/$DEB_ROS_ROOT/
  cp -rp ${PROJECT_DIR}/install/seyond/lib deb_root/$DEB_ROS_ROOT/
  if [ -d "${PROJECT_DIR}/install/seyond/include" ]; then
    cp -rp ${PROJECT_DIR}/install/seyond/include deb_root/$DEB_ROS_ROOT/
  fi
  if [ -d "${PROJECT_DIR}/install/seyond/local" ]; then
    cp -rp ${PROJECT_DIR}/install/seyond/local deb_root/$DEB_ROS_ROOT/
  fi
else
  echo "the ros version has not support yet!"
  exit -1
fi

echo Package: seyond-${ROS_DISTRO}-driver > deb_root/DEBIAN/control
echo Version: ${SOFT_VER} >> deb_root/DEBIAN/control

# For better clarity and differentiation, use 'amd64' or 'arm64' instead of 'all'.
echo Architecture: all >> deb_root/DEBIAN/control

echo Maintainer: Seyond >> deb_root/DEBIAN/control
echo Description: Seyond ROS Driver >> deb_root/DEBIAN/control
echo "Build-Time: $(date -R)" >> deb_root/DEBIAN/control

chmod -R 755 deb_root
DEB_RESULT_FILE_PREFIX_PARTIAL=seyond-ros${ROS_VERSION}-${ROS_DISTRO}-${SOFT_VER}
dpkg-deb -z 0 --build deb_root ${DEB_RESULT_FILE_PREFIX_PARTIAL}.deb
rm -rf deb_root
rm -rf ${CURRENT_DIR}/deb_control
ls -lrt
popd

echo -e "\n\033[1;32m-- (4). Done!\033[0m"
