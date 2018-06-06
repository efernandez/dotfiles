#!/bin/bash

GDB_PRINTERS=~/.gdb_printers

if [[ ! -d $GDB_PRINTERS ]];
then
    mkdir $GDB_PRINTERS
    cd $GDB_PRINTERS
    svn co svn://gcc.gnu.org/svn/gcc/trunk/libstdc++-v3/python
    cd -
fi

cp .gdbinit ~/.gdbinit
