#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS
#include "myclass.h"
#include "mySetPara.h"
#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>

int main()
{
	////////////////////////////
	createDirectoryRecursively();
	// Delete directory contents, the output directory must be created!!!
	deleteDirectoryContents();
	////////////////////////////

	// Whether to perform quantitative experiments
	if (isQuantitativeExperiment) {
		// Current data type, can be "sphere" or "torus"
		std::string dataType = "torus";
		// Data path
		std::string dataPath = "./data/" + dataType;
		// Output file path
		std::ofstream outfile("./output/quantitative_results.txt");
		// Iterate through the dataset
		for (int i = 0; i <= 80; i++) {
			// Current iteration
			number_of_Candidates = 0;
			// Point cloud file path
			std::string pointfilename = dataPath + "/" + std::to_string(i) + "/points.obj";
			std::string linefilename = dataPath + "/" + std::to_string(i) + "/lines.obj";

			// Point cloud and line cloud data
			PointCloud pc; // Point cloud
			PointCloud Line2pc; // Converted line cloud
			LineCloud Lc; // Line cloud

			// Initialize RANSAC parameters	
			RansacShapeDetector::Options ransacOptions;
			setRansacOptions(ransacOptions, dataType);

			// Get data
			switch (INPUT_TYPE) {
			case 0:
				// Point cloud only
				readPoints(pointfilename, pc);
				break;
			case 1:
				// Line cloud only
				readLines(linefilename, Lc, ransacOptions.m_bitmapEpsilon);
				break;
			case 2:
				// Both point and line cloud
				readPoints(pointfilename, pc);
				readLines(linefilename, Lc, ransacOptions.m_bitmapEpsilon);
				break;
			default:
				std::cerr << "ERROR: Unknown INPUT_TYPE = " << INPUT_TYPE << std::endl;
				break;
			}

			// Perform preprocessing to convert line segments to point clouds and then merge them with the original point cloud
			lc2pc(Lc, ransacOptions.m_bitmapEpsilon, Line2pc);
			pc += Line2pc;
			pc.calcNormals(30);
			// Estimate line normals
			estimateLineNormals(pc, Lc);


			// Create RANSAC shape detector
			RansacShapeDetector detector(ransacOptions);
			// Add plane primitive shape constructor
			detector.Add(new PlanePrimitiveShapeConstructor());

			// Store plane primitive shapes
			MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes;

			// Initialize trial count
			int trials = 30;
			// Record successful plane detections
			int num_plane = 0;
			// Record average successful detection time
			double timeSum = 0;
			for (int j = 0; j < trials; j++) {
				// Display current trial
				std::cout << dataType << " " << i << " " << j << std::endl;
				detector.Detect(pc, Lc, 0, pc.size(), &shapes);

				num_plane += shapes.size();
				std::string outPutPath = "./output/" + std::to_string(i) + "/" + std::to_string(j);
				//writePointPlane(shapes, pc, Lc, ransacOptions.m_epsilon, ransacOptions.m_bitmapEpsilon, outPutPath);
				for (int k = 0; k < Lc.size(); k++) {
					Lc[k].tag = false;
				}
				shapes.clear();
			}
		}
	}
	else {

		// Initialize data type
		std::string dataType = "test";
		// Data path
		std::string pointfilename = "./data/" + dataType + "/points.obj";
		std::string linefilename = "./data/" + dataType + "/lines.obj";
		// Output path
		PointCloud pc; // Point cloud
		PointCloud line2pc; // Converted line cloud
		// Merged point cloud
		LineCloud Lc;
		// RANSAC options
		RansacShapeDetector::Options ransacOptions;
		// Set RANSAC options
		setRansacOptions(ransacOptions, dataType);
		// Get data
		if (INPUT_TYPE == 0) {
			readPoints(pointfilename, pc);
		}
		else if (INPUT_TYPE == 1) {
			readLines(linefilename, Lc, ransacOptions.m_bitmapEpsilon);
		}
		else {
			readPoints(pointfilename, pc);
			readLines(linefilename, Lc, ransacOptions.m_bitmapEpsilon);
		}
		lc2pc(Lc, ransacOptions.m_bitmapEpsilon, line2pc);
		pc += line2pc;
		pc.calcNormals(30);
		estimateLineNormals(pc, Lc);
		// Output the number of added points and lines
		std::cout << "added " << pc.size() << " points" << std::endl;
		std::cout << "added " << Lc.size() - 1 << " lines" << std::endl;
		RansacShapeDetector detector(ransacOptions);
		detector.Add(new PlanePrimitiveShapeConstructor());
		MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes;
		int remainpoint = 0;
		remainpoint = detector.Detect(pc, Lc, 0, pc.size(), &shapes); // Perform detection
		writePointPlane(shapes, pc, Lc, ransacOptions.m_epsilon, ransacOptions.m_bitmapEpsilon);
	}
	return 0;
}