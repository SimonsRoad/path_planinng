all:
	mex surfpoints.cpp -I/usr/local/include -L/usr/local/lib -lopencv_core -lopencv_features2d
	mex surfmatch.cpp
	mex surfpoints_gpu.cpp -Icommon/gpusurf -I/opt/local/include -I/opt/local/include/opencv -L/opt/local/lib -I/usr/local/cuda/include -lopencv_core -lopencv_features2d

