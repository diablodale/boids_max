# Jitter Makefile by Wesley Smith 1/8/2005 wesley.hoke@gmail.com
# This file uses gcc to make Jitter objects under Cygwin on the Windows platform.
# To use this file to make your own objects will require a little bit of 
# modification although I have tried to make the modifications as simple
# as possible.
#
# How to use this Makefile:
# example command line: $>make xray.jit.quicksort
# 
# The Makefile will read xray.jit.quicksort and look for the following 2 files:
# xray.jit.quicksort.c and max.xray.jit.quicksort.c.
#
# The 2 files will be compiled into .o's.  Then, the linker will take those .o's
# and build a .mxe from them and a file called max.xray.jit.def.  The .def will have the 
# following form:
#------------------------------- 
# ;xray.jit.quicksort.def
#
# LIBRARY	jit.quicksort
#
# EXPORTS
# 	main
#-------------------------------------
#
# For some reason, you can't put a name for the LIBRARY that has more than one period in it
# so I use an intermediate name that lacks the namespace.  Once the object is compiled, I
# rename the object to include the name space so that the output of the make process will be:
# xray.jit.quicksort.mxe
#
# Variables that you will most likely have to modify:
# JITTER - the diretory where BOTH the jitter includes and jitlib.lib can be found
# MAX - the directory where BOTH the max includes and MaxAPI.lib can be found

JITTER = /home/whsmith/JitterWinSDK/JitterWindowsSDK/c74support/jit-includes
MAX = /home/whsmith/maxmspsdk_win/maxmspsdk_win/4.5headers/c74support/max-includes

CC = gcc
CFLAGS = -c -mno-cygwin -DWIN_VERSION -DWIN_EXT_VERSION -I$(JITTER) -I$(MAX)

JITLIB = -ljitlib
MAXLIB = -lMaxAPI
LDFLAGS = -shared -mno-cygwin -L$(JITTER) $(JITLIB) -L$(MAX) $(MAXLIB)

JITSRC = $(STEM:%=%.c)
MAXSRC = $(STEM:%=max.%.c)

JITOBJ = $(STEM:%=%.o)
MAXOBJ = $(STEM:%=max.%.o)

JITDEF = $(STEM:%=max.%.def)

JITOUTPUT = $(STEM:%=%.mxe)

#stem is the command line argument that gets set when make is called
STEM = default



all:
#	echo $(CFLAGS)
#	echo $(LDFLAGS)

%: STEM = $@
%:
	$(CC) $(CFLAGS) $(JITSRC)
	$(CC) $(CFLAGS) $(MAXSRC)
	$(CC) -o $(JITOUTPUT) $(JITOBJ) $(MAXOBJ) $(JITDEF) $(LDFLAGS)
	
clean:
	rm -f *.o
	