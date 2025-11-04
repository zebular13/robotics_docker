#!/bin/bash

set -e

WORKDIR=~/edge-impulse-tools
NODEDIR=$WORKDIR/node
SOXDIR=$WORKDIR/sox
TMPDIR=$WORKDIR/tmp

echo "Installing Edge Impulse for Linux (and its dependencies) to $WORKDIR..."

# Install Node.js
install_nodejs_portable() {
    rm -rf $TMPDIR
    mkdir -p $TMPDIR
    mkdir -p $NODEDIR

    NODE_VERSION="v20.18.2"

    echo "Downloading Node.js $NODE_VERSION..."

    # Download Node.js (note to EI, you'll need to mirror new versions here to CDN manually)
    wget https://cdn.edgeimpulse.com/build-system/node-$NODE_VERSION-linux-arm64.tar.xz -O $TMPDIR/node-$NODE_VERSION-linux-arm64.tar.xz

    echo "Extracting..."

    # Extract Node.js
    tar xf $TMPDIR/node-$NODE_VERSION-linux-arm64.tar.xz -C $TMPDIR/ --no-same-owner

    # Move Node.js to /usr/local
    mv $TMPDIR/node-$NODE_VERSION-linux-arm64/* $NODEDIR

    # Cleanup
    rm -rf $TMPDIR
}

# source os-release to get the version
. /etc/os-release

# check if VERSION different than 1.3-ver.1.1
if [ "$VERSION" != "1.3-ver.1.1" ]
then
    echo ""
    echo "This script was tested on Qualcomm Linux version 1.3-ver.1.1, running on other versions might not work (found '$VERSION')"
    echo "If you have any issues with running this script on your Qualcomm Linux version, please report an issue at https://forum.edgeimpulse.com"
    echo ""
fi

# Check if NodeJS is available
if [ ! -f $NODEDIR/bin/node ]; then
    echo "Node.js was not found ($WORKDIR/bin/node), installing..."
    install_nodejs_portable
    if [ ! -f $NODEDIR/bin/node ]; then
        echo "Node.js installation failed! Check logs above."
        exit 1
    fi
    echo "Node.js installation OK ($WORKDIR/bin/node)"
    echo ""
else
    echo "Found Node.js ($WORKDIR/bin/node)"
    echo ""
fi

# we check if node is in the PATH _before_ changing PATH, if so, we don't need to update ~/.profile
SHOULD_UPDATE_PROFILE_NODE=1
if command -v node >/dev/null 2>&1; then
    SHOULD_UPDATE_PROFILE_NODE=0
fi

PATH="$PATH":$NODEDIR/bin/

# Check if Edge Impulse Linux CLI is available
if [ ! -f $NODEDIR/bin/edge-impulse-linux ]; then
    echo "Edge Impulse for Linux was not found ($WORKDIR/bin/edge-impulse-linux), installing..."

    # otherwise we'll get
    # node: error while loading shared libraries: cannot apply additional memory protection after relocation: Permission denied
    setenforce 0 || true

    npm install -g edge-impulse-linux
    if [ ! -f $NODEDIR/bin/edge-impulse-linux ]; then
        echo "Edge Impulse Linux CLI installation failed! Check logs above."
        exit 2
    fi
    echo "Edge Impulse for Linux installation OK ($WORKDIR/bin/edge-impulse-linux)"
    echo ""
else
    echo "Found Edge Impulse for Linux ($WORKDIR/bin/edge-impulse-linux)"
    echo ""
fi

# we check if sox is in the PATH _before_ changing PATH, if so, we don't need to update ~/.profile
SHOULD_UPDATE_PROFILE_SOX=1
if command -v sox >/dev/null 2>&1; then
    SHOULD_UPDATE_PROFILE_SOX=0
fi

PATH="$PATH":$SOXDIR/bin/

# Check if sox is available
if [ ! -f $SOXDIR/bin/sox ]; then
    echo "sox was not found ($SOXDIR/bin/sox), installing..."

    rm -rf $SOXDIR
    mkdir -p $SOXDIR
    cd $SOXDIR
    wget https://cdn.edgeimpulse.com/build-system/qclinux/sox-qclinux-14.4.2-v1.tar.gz
    tar xf sox-qclinux-14.4.2-v1.tar.gz

    if [ ! -f $SOXDIR/bin/sox ]; then
        echo "sox installation failed! Check logs above."
        exit 2
    fi
    echo "sox installation OK ($SOXDIR/bin/sox)"
    echo ""
else
    echo "Found sox ($SOXDIR/bin/sox)"
    echo ""
fi

# Update .profile
if [ $SHOULD_UPDATE_PROFILE_NODE -eq 1 ] || [ $SHOULD_UPDATE_PROFILE_SOX -eq 1 ]; then
    echo "Updating ~/profile..."

    if [ $SHOULD_UPDATE_PROFILE_NODE -eq 1 ]; then
        echo "" >> ~/.profile
        echo "# Begin Edge Impulse Linux (Node.js part)" >> ~/.profile
        echo "setenforce 0      # Disable enforcing SELinux (required to run Node)" >> ~/.profile
        echo "export PATH=\"\$PATH\":$NODEDIR/bin" >> ~/.profile
        echo "# End Edge Impulse Linux (Node.js part)" >> ~/.profile
        echo "" >> ~/.profile
    fi

    if [ $SHOULD_UPDATE_PROFILE_SOX -eq 1 ]; then
        echo "" >> ~/.profile
        echo "# Begin Edge Impulse Linux (sox part)" >> ~/.profile
        echo "export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH\":$SOXDIR/lib" >> ~/.profile
        echo "export PATH=\"\$PATH\":$SOXDIR/bin" >> ~/.profile
        echo "# End Edge Impulse Linux (sox part)" >> ~/.profile
        echo "" >> ~/.profile
    fi

    echo "Updating ~/profile OK"
    echo ""
    echo "Done. Run:"
    echo "    $ source ~/.profile"
    echo "    $ edge-impulse-linux"
    echo "to get started!"
else
    echo "Done. Run 'edge-impulse-linux' to get started!"
fi
