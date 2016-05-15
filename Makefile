PROJECTNAME = pcl_test
CPPFLAGS = -g -std=c++11 -Wall
LDFLAGS = -lboost_system -lboost_thread -L /usr/lib/x86_64-linux-gnu/ -lpcl_common -lpcl_io -lpcl_visualization -lpcl_search -lpcl_features -lpcl_kdtree -lvtkCommonCore-6.2 -lvtkCommonDataModel-6.2 -lvtkRenderingLOD-6.2 -lvtkRenderingCore-6.2 -lvtkCommonMath-6.2 -lvtkFiltersSources-6.2 -lvtkCommonExecutionModel-6.2
OBJDIR = obj/
SRCDIR = src/
INCDIRS = include/ /usr/include/pcl-1.7/ /usr/include/eigen3/ /usr/include/vtk-6.2/
INC = $(foreach d, $(INCDIRS), -I$d)
COMPILER = g++

RM = rm -rf

SRCS = $(shell find $(SRCDIR) -name "*.cpp")
OBJS = $(addprefix $(OBJDIR),$(subst $(SRCDIR),,$(SRCS:.cpp=.o)))

all: $(PROJECTNAME)

$(PROJECTNAME): $(OBJS)
		$(COMPILER) $(OBJS) $(LDFLAGS) -o $(PROJECTNAME)

$(OBJDIR)%.o: $(SRCDIR)%.cpp
		@mkdir -p $(@D)
			$(COMPILER) $(CPPFLAGS) $(INC) -c $< -o $@

clean:
		$(RM) $(OBJDIR)* $(PROJECTNAME)

clear: clean $(PCA_ICP)
