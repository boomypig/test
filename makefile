# Object files common to all targets
ALL_THE_OS = Arena.o RobotBase.o
THE_DOT_HS = Arena.h RobotBase.h

# The default target to build all executables
all: test_robot

# Compile all .cpp files into .o files
%.o: %.cpp
	g++ -g -std=c++20 -Wall -Wpedantic -Wextra -Werror -c -fPIC $<

# Link object files to create test_robot executable

test_robot: test_robot.o $(ALL_THE_OS)
	g++ -g -I/usr/include -o test_robot test_robot.o $(ALL_THE_OS) -ldl

RobotBase.o: RobotBase.cpp
	g++ -g  -std=c++20 -Wall -Wpedantic -Wextra -Werror -c -fPIC $<
Arena.o: Arena.cpp
	g++ -g  -std=c++20 -Wall -Wpedantic -Wextra -Werror -c -fPIC $<

# Clean up all object files and executables
clean:
	rm -f *.o  test_robot 