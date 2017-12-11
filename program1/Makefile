CC = g++
CC_FLAGS = -Wall
LIB = -L/usr/local/lib/
LD_FLAGS = -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc
INPUT_FILE = program1.cpp
OUTPUT_FILE = program1

all: $(OUTPUT_FILE)

$(OUTPUT_FILE): $(INPUT_FILE)
	$(CC) $(CC_FLAGS) $(LIB) $(INPUT_FILE) $(LD_FLAGS) -o $(OUTPUT_FILE)

clean:
	rm $(OUTPUT_FILE)
