all: jpegheader.h decoder.cpp
	g++ ./decoder.cpp -fpermissive -o ./decoder
clean:
	rm ./decoder
help:
	@ echo "make - build file into decoder"
	@ echo "clean - clean executable decoder file"
