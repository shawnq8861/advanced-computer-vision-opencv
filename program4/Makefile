CC = g++
CC_FLAGS = -Wall
LIB = -L/usr/local/lib/
LD_FLAGS = -lueye_api -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d -lopencv_features2d -lopencv_xfeatures2d
INPUT_FILE = program4.cpp
OUTPUT_FILE = program4

all: $(OUTPUT_FILE)

$(OUTPUT_FILE): $(INPUT_FILE)
	$(CC) $(CC_FLAGS) $(LIB) $(INPUT_FILE) $(LD_FLAGS) -o $(OUTPUT_FILE)

clean:
	rm $(OUTPUT_FILE)
