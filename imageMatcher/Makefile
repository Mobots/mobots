#  http://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html
# 'make depend' uses makedepend to automatically generate dependencies 
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#

# define the C compiler to use
CC = g++

# define any compile-time flags
CFLAGS = -Wall

# define any directories containing header files other than /usr/include
#
INCLUDES = -I/usr/local/include/opencv -I/usr/include/opencv

# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
#LFLAGS = -L/home/newhall/lib  -L../lib

# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname 
#   option, something like (this will link in libmylib.so and libm.so:
OPENCV_LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video \
-lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy \
-lopencv_flann -lopencv_gpu
# define the C source files
SRCS = testFeatures.cpp FeaturesFinder.cpp FeaturesMatcher.cpp ror.c

# define the C object files 
#
# This uses Suffix Replacement within a macro:
#   $(name:string1=string2)
#         For each word in 'name' replace 'string1' with 'string2'
# Below we are replacing the suffix .c of all words in the macro SRCS
# with the .o suffix
#
OBJS = $(SRCS:.cpp=.o)

# define the executable file 
MAIN = testFeatures

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all:    $(MAIN)

$(MAIN): $(OBJS) 
	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) $(OPENCV_LIBS)

# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	$(RM) *.o *~ $(MAIN)

depend: $(SRCS)
	makedepend $(INCLUDES) $^ 2>/dev/null #fuck this shit

# DO NOT DELETE THIS LINE -- make depend needs it

testFeatures.o: /usr/include/opencv2/highgui/highgui.hpp
testFeatures.o: /usr/include/opencv2/core/core.hpp
testFeatures.o: /usr/include/opencv2/core/types_c.h /usr/include/assert.h
testFeatures.o: /usr/include/features.h /usr/include/bits/predefs.h
testFeatures.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
testFeatures.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
testFeatures.o: /usr/include/stdlib.h /usr/include/bits/waitflags.h
testFeatures.o: /usr/include/bits/waitstatus.h /usr/include/endian.h
testFeatures.o: /usr/include/bits/endian.h /usr/include/bits/byteswap.h
testFeatures.o: /usr/include/sys/types.h /usr/include/bits/types.h
testFeatures.o: /usr/include/bits/typesizes.h /usr/include/time.h
testFeatures.o: /usr/include/sys/select.h /usr/include/bits/select.h
testFeatures.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
testFeatures.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
testFeatures.o: /usr/include/alloca.h /usr/include/string.h
testFeatures.o: /usr/include/xlocale.h /usr/include/stdint.h
testFeatures.o: /usr/include/bits/wchar.h /usr/include/math.h
testFeatures.o: /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h
testFeatures.o: /usr/include/bits/huge_vall.h /usr/include/bits/inf.h
testFeatures.o: /usr/include/bits/nan.h /usr/include/bits/mathdef.h
testFeatures.o: /usr/include/bits/mathcalls.h
testFeatures.o: /usr/include/opencv2/core/version.hpp
testFeatures.o: /usr/include/opencv2/core/operations.hpp
testFeatures.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
testFeatures.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
testFeatures.o: /usr/include/bits/posix2_lim.h
testFeatures.o: /usr/include/opencv2/core/mat.hpp
testFeatures.o: /usr/include/opencv2/highgui/highgui_c.h
testFeatures.o: /usr/include/opencv2/core/core_c.h FeaturesFinder.h
testFeatures.o: /usr/include/opencv2/features2d/features2d.hpp
testFeatures.o: /usr/include/opencv2/flann/miniflann.hpp
testFeatures.o: /usr/include/opencv2/gpu/gpu.hpp
testFeatures.o: /usr/include/opencv2/imgproc/imgproc.hpp
testFeatures.o: /usr/include/opencv2/imgproc/types_c.h
testFeatures.o: /usr/include/opencv2/objdetect/objdetect.hpp
testFeatures.o: /usr/include/opencv2/gpu/gpumat.hpp
testFeatures.o: /usr/include/opencv2/gpu/devmem2d.hpp
testFeatures.o: /usr/include/opencv2/gpu/matrix_operations.hpp
testFeatures.o: FeaturesMatcher.h
FeaturesFinder.o: /usr/include/opencv2/gpu/gpumat.hpp
FeaturesFinder.o: /usr/include/opencv2/core/core.hpp
FeaturesFinder.o: /usr/include/opencv2/core/types_c.h /usr/include/assert.h
FeaturesFinder.o: /usr/include/features.h /usr/include/bits/predefs.h
FeaturesFinder.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
FeaturesFinder.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
FeaturesFinder.o: /usr/include/stdlib.h /usr/include/bits/waitflags.h
FeaturesFinder.o: /usr/include/bits/waitstatus.h /usr/include/endian.h
FeaturesFinder.o: /usr/include/bits/endian.h /usr/include/bits/byteswap.h
FeaturesFinder.o: /usr/include/sys/types.h /usr/include/bits/types.h
FeaturesFinder.o: /usr/include/bits/typesizes.h /usr/include/time.h
FeaturesFinder.o: /usr/include/sys/select.h /usr/include/bits/select.h
FeaturesFinder.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
FeaturesFinder.o: /usr/include/sys/sysmacros.h
FeaturesFinder.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
FeaturesFinder.o: /usr/include/string.h /usr/include/xlocale.h
FeaturesFinder.o: /usr/include/stdint.h /usr/include/bits/wchar.h
FeaturesFinder.o: /usr/include/math.h /usr/include/bits/huge_val.h
FeaturesFinder.o: /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h
FeaturesFinder.o: /usr/include/bits/inf.h /usr/include/bits/nan.h
FeaturesFinder.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
FeaturesFinder.o: /usr/include/opencv2/core/version.hpp
FeaturesFinder.o: /usr/include/opencv2/core/operations.hpp
FeaturesFinder.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
FeaturesFinder.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
FeaturesFinder.o: /usr/include/bits/posix2_lim.h
FeaturesFinder.o: /usr/include/opencv2/core/mat.hpp
FeaturesFinder.o: /usr/include/opencv2/gpu/devmem2d.hpp FeaturesFinder.h
FeaturesFinder.o: /usr/include/opencv2/features2d/features2d.hpp
FeaturesFinder.o: /usr/include/opencv2/flann/miniflann.hpp
FeaturesFinder.o: /usr/include/opencv2/gpu/gpu.hpp
FeaturesFinder.o: /usr/include/opencv2/imgproc/imgproc.hpp
FeaturesFinder.o: /usr/include/opencv2/imgproc/types_c.h
FeaturesFinder.o: /usr/include/opencv2/core/core_c.h
FeaturesFinder.o: /usr/include/opencv2/objdetect/objdetect.hpp
FeaturesFinder.o: /usr/include/opencv2/gpu/matrix_operations.hpp profile.h
FeaturesMatcher.o: /usr/include/opencv2/highgui/highgui.hpp
FeaturesMatcher.o: /usr/include/opencv2/core/core.hpp
FeaturesMatcher.o: /usr/include/opencv2/core/types_c.h /usr/include/assert.h
FeaturesMatcher.o: /usr/include/features.h /usr/include/bits/predefs.h
FeaturesMatcher.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
FeaturesMatcher.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
FeaturesMatcher.o: /usr/include/stdlib.h /usr/include/bits/waitflags.h
FeaturesMatcher.o: /usr/include/bits/waitstatus.h /usr/include/endian.h
FeaturesMatcher.o: /usr/include/bits/endian.h /usr/include/bits/byteswap.h
FeaturesMatcher.o: /usr/include/sys/types.h /usr/include/bits/types.h
FeaturesMatcher.o: /usr/include/bits/typesizes.h /usr/include/time.h
FeaturesMatcher.o: /usr/include/sys/select.h /usr/include/bits/select.h
FeaturesMatcher.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
FeaturesMatcher.o: /usr/include/sys/sysmacros.h
FeaturesMatcher.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
FeaturesMatcher.o: /usr/include/string.h /usr/include/xlocale.h
FeaturesMatcher.o: /usr/include/stdint.h /usr/include/bits/wchar.h
FeaturesMatcher.o: /usr/include/math.h /usr/include/bits/huge_val.h
FeaturesMatcher.o: /usr/include/bits/huge_valf.h
FeaturesMatcher.o: /usr/include/bits/huge_vall.h /usr/include/bits/inf.h
FeaturesMatcher.o: /usr/include/bits/nan.h /usr/include/bits/mathdef.h
FeaturesMatcher.o: /usr/include/bits/mathcalls.h
FeaturesMatcher.o: /usr/include/opencv2/core/version.hpp
FeaturesMatcher.o: /usr/include/opencv2/core/operations.hpp
FeaturesMatcher.o: /usr/include/limits.h /usr/include/bits/posix1_lim.h
FeaturesMatcher.o: /usr/include/bits/local_lim.h /usr/include/linux/limits.h
FeaturesMatcher.o: /usr/include/bits/posix2_lim.h
FeaturesMatcher.o: /usr/include/opencv2/core/mat.hpp
FeaturesMatcher.o: /usr/include/opencv2/highgui/highgui_c.h
FeaturesMatcher.o: /usr/include/opencv2/core/core_c.h
FeaturesMatcher.o: /usr/include/opencv2/features2d/features2d.hpp
FeaturesMatcher.o: /usr/include/opencv2/flann/miniflann.hpp
FeaturesMatcher.o: /usr/include/opencv2/calib3d/calib3d.hpp FeaturesMatcher.h
FeaturesMatcher.o: FeaturesFinder.h /usr/include/opencv2/gpu/gpu.hpp
FeaturesMatcher.o: /usr/include/opencv2/imgproc/imgproc.hpp
FeaturesMatcher.o: /usr/include/opencv2/imgproc/types_c.h
FeaturesMatcher.o: /usr/include/opencv2/objdetect/objdetect.hpp
FeaturesMatcher.o: /usr/include/opencv2/gpu/gpumat.hpp
FeaturesMatcher.o: /usr/include/opencv2/gpu/devmem2d.hpp
FeaturesMatcher.o: /usr/include/opencv2/gpu/matrix_operations.hpp ror.h
FeaturesMatcher.o: profile.h
