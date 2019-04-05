#!/bin/bash -xe
WANT_ENV="docker-build docker-run"
. $(dirname $0)/env.sh

###########################
# Update system & install general dependencies
###########################

${SUDO} apt-get update
${SUDO} apt-get upgrade -y

${SUDO} apt-get install -y \
    dirmngr \
    python-pip \
    python-setuptools \
    python-yaml \
    python-distribute \
    python-docutils \
    python-dateutil \
    python-six \
    python-rosdep \
    python-rosinstall \
    python-wstool \
    python-catkin-tools \
    python-sh \
    python-mock \
    python-redis \
    python-future \
    python-wheel \
    qtchooser \
    build-essential \
    doxygen \
    ssh-client \
    cgroup-tools \
    redis-server
${SUDO:+${SUDO} -H} pip install --upgrade \
    parso \
    catkin_lint \
    fysom \
    pytest \
    pytest-qt \
    pytest-dependency \
    pytest-mock \
    attrs

cat | ${SUDO} tee -a /etc/apt/preferences <<EOF

Package: python-fysom
Pin: release *
Pin-Priority: -1
EOF

###########################
# Clean up
###########################
if $CLEANUP; then
    ${SUDO} apt-get clean
fi
