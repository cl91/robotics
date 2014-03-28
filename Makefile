# Makefile for player c++ clients

#Specify your target and your cc source file name below
TARGET = client
SRC = client.cc leg_detection.cc

# Pick up the necessary options, directories and options for compiling and linking with player 
# Add a warning on all errors, and -g3 for debugging information 
CPPFLAGS = `pkg-config --cflags playerc++` -Wall -g3 
LDFLAGS = `pkg-config --libs playerc++`

#Target and command below builds the target output executable file
$(TARGET): $(SRC)
	g++ -std=c++0x $(CPPFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)
