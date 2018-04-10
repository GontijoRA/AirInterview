Airsquire task - Cylinder Detection - Round 2

Author: Roberto Almeida Gontijo
Place: Belo Horizonte - Minas Gerais - Brazil
Email: robertoalmeidagontijo@hotmail.com

-------------------------------------------------------------------------------------------------------------
What was done?
	- C/C++ programming of:
		- Cylinder detection 

-------------------------------------------------------------------------------------------------------------
Files
	main.cpp - Contain the main function	
	functions.cpp - Contain functions to print clouds, filter, estimate normals, segmentation of planes, join point planes in one cloud, remove points, clusterize, detect cylinder
	header.h - Contain prototypes of functions and references of libraries

-------------------------------------------------------------------------------------------------------------	
How to compile the program?
	- Open linux terminal	
	- Go to ./Airsquire/build/
	- Type: ./compile.sh

		or

		cmake ..
		make

------------------------------------------------------------------------------------------------------------		
How to execute the program?
	- Open linux terminal	
	- Go to ./Airsquire/build/
	- Type: 

		./airsquire.sh filename        ( where filename is the name of one file inside the input_files folder ) 
		or
		./airsquire ./input_files/ filename ( where filename is the name of one file inside the input_files folder )

	e.g.,
		./airsquire.sh table_scene_mug_stereo_textured.pcd
		./airsquire.sh test33.pcd
		./airsquire.sh test34.pcd

		or
	
		./airsquire ./input_files/ test33.pcd

-------------------------------------------------------------------------------------------------------------
Input files
The input_file table_scene_mug_stereo_textured.pcd was obtained at:
	 http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php

The input_files: test{33, 34, 35, 46, 47, 48, 49, 50, 51, 52, 53}.pcd were obtained at:
	https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD/test
	
------------------------------------------------------------------------------------------------------------
Algorithm implemented to cylinder detection by pointcloud analysis:
	1) Read the input cloud
	2) Removing outliers and computing normals (Same beginning steps of PCL Cylinder model segmentation tutorial)
		a) Build a passthrough filter to remove spurious NaNs	
		b) Estimate point normals
	3) Segmentation of planes based on depth information to an organized cloud (organizedMultiPlaneSegmentation)
	4) Removing planes because walls, desks and many other objects built to support things are planes
	5) Euclidean Clustering to create point groups based on euclidean distance
	6) For each cluster
		a) Evaluate if a cluster has a cylinder by sample_consensus cylinder model	
		b) If the cluster has a cylinder
			b.1 Write an output file with this cluster

Note: The code was inspired in three PCL tutorials (organizedMultiPlaneSegmentation, Cylinder Detection and Euclidean Clustering).

-------------------------------------------------------------------------------------------------------------	
Video:
	- Go to ./Airsquire/video

------------------------------------------------------------------------------------------------------------
Rendering in web
	- Website with the PCL input_clouds and respective output rendering:
	- Go using windows or linux to the website:
	gontijora.000webhostapp.com/

	Note: My account is free at this domain and 1h per day my website sleep (this is a condition of this host) 
