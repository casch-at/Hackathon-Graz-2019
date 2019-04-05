#!/bin/bash -xe
WANT_ENV="docker-build docker-run"
. $(dirname $0)/env.sh

${SUDO} apt-get update

###########################
# Tools
###########################

# install black
PY3_BUILD_DIR=${PY3_BUILD_DIR:-${WS_DIR}/python3}
mkdir -p "${PY3_BUILD_DIR}" && cd "${PY3_BUILD_DIR}"

if ! type "python3.6" >/dev/null; then
    PY_VERSION=3.6.7
    wget https://www.python.org/ftp/python/${PY_VERSION}/Python-${PY_VERSION}.tgz
    tar xvf Python-${PY_VERSION}.tgz
    cd Python-${PY_VERSION}
    ./configure --without-tests --disable-tests --enable-unicode
    make -j$(nproc)
    ${SUDO} make altinstall
else
    sudo apt-get install -y \
         python3-pip \
         python3-setuptools \
         python3-wheel
fi

# Install Python QA tools
${SUDO:+${SUDO} -H} python3.6 -m pip install \
    black \
    pre-commit
# - Monkey-patch install module to recognize .launch files
#${SUDO} sed -i -e "/'kt'/ a \    \'launch\': {\'text\', \'xml\'}," \
#    /usr/local/lib/python3.6/site-packages/identify/extensions.py
${SUDO:+${SUDO} -H} pip install \
    flake8 \
    pep8-naming

# Install QML QA tools
# don't need to install any dependencies since Qt is installed with ROS
if 1; then
QMLFMT_BUILD_DIR=${QMLFMT_BUILD_DIR:-${WS_DIR}/qmlfmt}
git clone https://github.com/machinekoder/qmlfmt.git "${QMLFMT_BUILD_DIR}"
cd "${QMLFMT_BUILD_DIR}"
source ${DOCKER_SCRIPTS_DIR}/qt-env-build.sh
cmake .
make -j$(nproc)
${SUDO} make install
fi

# Install Shell QA tools
${SUDO} apt-get install -y golang
${SUDO} go get mvdan.cc/sh/cmd/shfmt
${SUDO} ln -s ${GOPATH}/bin/shfmt /usr/local/bin/shfmt

# Install QA tool packages
${SUDO} apt-get install -y \
    clang-format

# install nano
${SUDO} apt-get install -y \
    nano

# Install Sphinx
${SUDO:+${SUDO} -H} pip install kitchen sphinx

###########################
# Clean up
###########################
if $CLEANUP; then
    ${SUDO} apt-get clean
    rm -rf ${PY3_BUILD_DIR}
    #rm -rf ${QMLFMT_BUILD_DIR}
    if test_environment docker-build; then
        ${SUDO} rm -rf /root/.ccache
    fi
fi
