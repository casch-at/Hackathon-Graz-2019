ARG DEBIAN_SUITE
FROM osrf/ros:melodic-desktop-full

ARG HTTP_PROXY

# Configure & update apt
# - Silence some Apt warnings
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";' > \
        /etc/apt/apt.conf.d/01norecommend
# - Update local Debian mirrors
RUN apt-get update
RUN apt-get upgrade -y && \
    apt-get clean
# - Silence more Apt warnings
RUN apt-get install -y \
    libfile-fcntllock-perl \
    apt-utils \
    && apt-get clean

# Install and configure sudo, passwordless for everyone
RUN apt-get install -y \
    sudo && \
    apt-get clean
RUN echo "ALL	ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers

# Cookie variable for container build environment
ENV ENV_COOKIE docker-build

###########################################
# Install packages
#
# Customize the following for building/running targeted software

# Install misc. packages
ARG EXTRA_PACKAGES
RUN apt-get install -y \
    astyle \
    ccache \
    ssh \
    gdb \
    wget \
    lsb-release \
    gnupg2 \
    mesa-utils \
    libgl1-mesa-dri \
    ${EXTRA_PACKAGES} \
    && apt-get clean

# Copy install scripts; only copy modular parts on demand so they can
# be fixed without dirtying the Docker cache
COPY env.sh /tmp/install/

# Update system & install general dependencies
COPY 1-system.sh /tmp/install/
RUN /tmp/install/1-system.sh

ENV GOPATH=/opt/go
COPY 8-tools.sh /tmp/install/
RUN /tmp/install/8-tools.sh

# Build and install nvidia graphics driver
COPY 9-glx.sh /tmp/install/
RUN /tmp/install/9-glx.sh

# Final cleanup
RUN rm -rf /tmp/install

# Image version for adding to the environment
ARG IMAGE_VERSION

ENV \
    # Speed builds with ccache
    PATH=/usr/lib/ccache:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin \
    # Qt configuration; use Qt v. 5 in /qt5 by default
    # Set locale for tools such as "black" to work correctly
    LC_ALL=C.UTF-8 \
    LANG=C.UTF-8 \
    # Set image version for internal sanity checks
    IMAGE_VERSION=${IMAGE_VERSION} \
    # disable Pythons .local import path
    PYTHONNOUSERSITE=True

# Running 'rosdep update' as root is not recommended
RUN rosdep fix-permissions

# Cookie variable for container run environment
ENV ENV_COOKIE docker-run

###########################################
# Set up user
#

# This shell script adds passwd and group entries for the user
COPY entrypoint.sh /usr/bin/entrypoint
ENTRYPOINT ["/usr/bin/entrypoint"]
# If no args to `docker run`, start an interactive shell
CMD ["/bin/bash", "--login", "-i"]
