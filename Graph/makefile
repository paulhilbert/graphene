MATLAB_INC_DIR = /home/slim/matlab/extern/include
COMMONS_INC_DIR = /usr/local/include/commons
COMMONS_LIB_DIR = /usr/local/lib/commons
MATLAB_LIB_DIR1 = /home/slim/matlab/bin/glnx86
SYS_LIB_DIR = /home/slim/matlab/bin/glnx86
MATLAB_LIB_DIR2 = /home/slim/matlab/sys/os/glnx86


CXX = g++
OBJ_EXT = .o
SOBJ_EXT = .so
OBJ_OPT = -c

MEX        = /home/slim/matlab/bin/mex
MEXSUFFIX  = .mexglx
MEXFLAGS   = -cxx CC='$(CXX)' CXX='$(CXX)' LD='$(CXX)'

#---------------------------------------------------------------------#
#                    compiler flags
#---------------------------------------------------------------------#
MATLAB_CXXFLAGS = \
					-I$(MATLAB_INC_DIR) \
					-I$(COMMONS_INC_DIR) \
					-D USE_LAPACKPP \
					-D USE_BOOST_RANDOM

CXXFLAGS = \
		-fPIC  \
		$(MATLAB_CXXFLAGS)	\
	   -O2

#---------------------------------------------------------------------#
#                    linker flags
#---------------------------------------------------------------------#
LDFLAGS_SO = \

LDFLAGS_MAT = \
			  -L$(MATLAB_LIB_DIR1) -L$(SYS_LIB_DIR) -L$(COMMONS_LIB_DIR)\
			  -lmat -leng -lut -lmx \
			  -lboost_system-mt -lboost_filesystem-mt -lboost_date_time \
			  -lexpat -llapackpp -lvf -lpthread \
			  -lUndirectedGraph

#--------------------------------------------------------------------#
#                    target entries
#---------------------------------------------------------------------#


all:  \
		matchGraphs$(MEXSUFFIX) 

#OBJ	=  \
   			UndirectedGraph$(OBJ_EXT) 

matchGraphsOBJ   =  \
            matchGraphs$(OBJ_EXT)

matchGraphs$(MEXSUFFIX): $(matchGraphsOBJ)
	/home/slim/matlab/bin/mex $(LDFLAGS_MAT) $(MEXFLAGS) $(matchGraphsOBJ)

clean:	
		rm -f  $(OBJ) $(matchGraphsOBJ)

#---------------------------------------------------------------------#
#                    suffix rules
#---------------------------------------------------------------------#


.C$(OBJ_EXT):
	$(CXX) $(CXXFLAGS) $(OBJ_OPT) $<

