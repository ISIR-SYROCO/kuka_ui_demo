kuka ui demo
============

Dependencies:
-------------

 * lua5.1
 * lqt: https://code.google.com/p/lqt/

Example of installation:
------------------------

    sudo apt-get install lua5.1
    cd ~/src
    git clone https://github.com/mkottman/lqt
    mkdir _build
    cd _build
    cmake -DCMAKE_INSTALL_PREFIX=/some/where ..
    make

Edit .bashrc to add some environment variables:

    export LUA_PATH="~/src/ros_workspace/orocos_toolchain/install/share/lua/5.1/?.lua"
    export LUA_CPATH="~/src/lqt/_build/lib/?.so"

If `orocos_toolchain` is installed in `~/src/ros_workspace/orocos_toolchain/`


