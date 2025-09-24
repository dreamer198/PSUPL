#include "myclass.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <PointCloud.h>
#include <Windows.h>
#include <random>
#include <string>
#include <direct.h> 
#include <io.h> 
#include <cmath>

int ImproveBoundsNum = 0;
int FindBestCandidateNum = 0;
std::chrono::microseconds ImproveBoundsTime(0);
std::chrono::microseconds FindBestCandidateTime(0);



size_t number_of_Candidates = 0;


void readPoints(const std::string& filename, PointCloud& point2pc) {

	std::ifstream infile(filename);
	if (!infile.is_open()) {

		std::cerr << "Failed to open file!" << std::endl;
		std::exit(EXIT_FAILURE);
	}
	// Prepare to read data
	std::string dataline;
	Point point;
	// Start reading the file
	while (std::getline(infile, dataline)) {
		std::istringstream iss(dataline);
		// Read vertex data from obj file
		if (dataline[0] == 'v') {
			iss.ignore(1);
		}
		if (!(iss >> point.pos[0] >> point.pos[1] >> point.pos[2])) {
			//std::cerr << "Error reading line: " << dataline << std::endl;
			continue;
		}
		point2pc.push_back(point);
	}
	// Close the file
	infile.close();
}

// Read lines from a file
void readLines(const std::string& filename, LineCloud& lc, double m_bitmapEpsilon) {
	// Open the file
	std::ifstream infile(filename);
	// Check if the file is open
	if (!infile.is_open()) {
		// Failed to open file
		std::cerr << "Failed to open file!" << std::endl;
		std::exit(EXIT_FAILURE);
	}
	// Prepare to read data
	std::string dataline;
	PointCloud pc;

	// Initialize point cloud and add a dummy point at index 0
	Point point;
	pc.push_back(point);
	// For the line cloud, add a dummy line at index 0
	lc.push_back(Line());

	Vec3f startPoint;
	Vec3f endPoint;
	size_t startPointIdx = 0;
	size_t endPointIdx = 0;
	// Line segment direction
	Vec3f direction;
	// Line segment length
	float segmentLength = 0;
	// Line segment midpoint
	double x = 0.0, y = 0.0, z = 0.0;
	char v = 'v';
	char l = 'l';

	// Prepare to read line segments
	std::vector<double> values;
	double value = 0;

	// Start reading the file
	while (std::getline(infile, dataline)) {
		std::istringstream iss(dataline);
		// Read vertex data from obj file
		// If the line starts with 'v', it's a vertex
		if (dataline[0] == 'v') {
			// Read vertex identifier
			iss >> v;
			while (iss >> value) {
				values.push_back(value);
			}
			point.pos = Vec3f(values[0], values[1], values[2]);
			if (values.size() == 6) {
				point.color = Vec3f(values[3], values[4], values[5]);
			}
			pc.push_back(point);
			values.clear();
		}
		// Read line segment data from obj file
		// If the line starts with 'l', it's a line segment
		if (dataline[0] == 'l') {
			if (!(iss >> l >> startPointIdx >> endPointIdx)) {
				std::cerr << "Error reading line: " << dataline << std::endl;
				continue;
			}
			// Start point
			startPoint = pc[startPointIdx].pos;
			// End point
			endPoint = pc[endPointIdx].pos;
			// Line segment direction
			direction = endPoint - startPoint;
			// Line segment length
			segmentLength = direction.length();
			// Normalize direction
			direction /= segmentLength;
			// Calculate effective number of samples
			int numSamples = int(segmentLength / m_bitmapEpsilon) + 2;
			Line line(startPoint, endPoint, segmentLength, direction, numSamples);
			line.color = pc[startPointIdx].color;
			lc.push_back(line);
		}
	}
	// Close the file
	infile.close();
}

// Convert line cloud to point cloud
void lc2pc(LineCloud& lc, double m_bitmapEpsilon, PointCloud& line2pc) {
	// Sample points along each line segment
	for (size_t i = 1; i < lc.size(); i++) {
		// Line segment length
		float segmentLength = lc[i].segmentLength;
		// Line segment direction
		Vec3f direction = lc[i].direction;

		for (int j = 0; j <= lc[i].num - 2; j++) {
			Vec3f samplePoint = lc[i].startPoint + direction * (j * m_bitmapEpsilon);
			line2pc.push_back(Point(samplePoint, i));
		}
		line2pc.push_back(Point(lc[i].endPoint, i));
	}
}

// Delete all contents of a directory, including subdirectories
void deleteDirectoryContents(const std::string& dirPath) {
	WIN32_FIND_DATA findFileData;
	HANDLE hFind = FindFirstFile((dirPath + "\\*").c_str(), &findFileData);
	if (hFind == INVALID_HANDLE_VALUE) {
		std::cerr << "Failed to open directory: " << dirPath << std::endl;
		return;
	}
	do {
		const std::string fileOrDirName = findFileData.cFileName;
		if (fileOrDirName == "." || fileOrDirName == "..") {
			continue;
		}
		const std::string fullPath = dirPath + "\\" + fileOrDirName;
		if (findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			// Delete contents of subdirectory
			deleteDirectoryContents(fullPath);
			if (!RemoveDirectory(fullPath.c_str())) { // Remove directory
				std::cerr << "Failed to remove directory: " << fullPath << std::endl;
			}
		}
		else {
			// Delete file
			if (!DeleteFile(fullPath.c_str())) {
				std::cerr << "Failed to delete file: " << fullPath << std::endl;
			}
		}
	} while (FindNextFile(hFind, &findFileData) != 0);
	FindClose(hFind);
}

// Split a file path into its components
std::vector<std::string> splitPath(const std::string& path) {
	std::vector<std::string> parts;
	std::string part;
	for (char c : path) {
		if (c == '\\' || c == '/') {
			if (!part.empty()) {
				parts.push_back(part);
				part.clear();
			}
		}
		else {
			part += c;
		}
	}
	if (!part.empty()) {
		parts.push_back(part);
	}
	return parts;
}

// Create a directory and all parent directories if they don't exist
void createDirectoryRecursively(const std::string& path) {
	std::vector<std::string> parts = splitPath(path);
	std::string currentPath;

	for (const auto& part : parts) {
		if (!currentPath.empty()) {
			currentPath += "\\";
		}
		currentPath += part;

		// Check if the directory exists
		if (_access(currentPath.c_str(), 0) != 0) {
			// Directory does not exist, create it
			if (_mkdir(currentPath.c_str()) != 0) {
				std::cerr << "Failed to create directory: " << currentPath << std::endl;
				std::cerr << "Error code: " << errno << std::endl; // Print error code
				return;
			}
		}
	}
}

// Write point plane, shapes store the corresponding plane, point2pc store the actual point cloud, Lc store the actual line cloud, transacOptions for plane segmentation, filePath for the output directory (default is output)
void writePointPlane(MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>>& shapes,
	PointCloud& point2pc, LineCloud& Lc, float m_epsilon, float m_bitmapEpsilon, const std::string& filePath) {

	// Create output directory
	createDirectoryRecursively(filePath);
	std::string pointPlanesDir = filePath + "/pointPlanes";
	std::string linePlanesDir = filePath + "/linePlanes";
	createDirectoryRecursively(pointPlanesDir);
	createDirectoryRecursively(linePlanesDir);

	// Calculate the total number of points
	size_t pointNumSum = 0;

	// Create a random number generator
	std::random_device rd;  // Obtain a random number from hardware
	std::mt19937 gen(rd()); // Seed the generator
	// Create a uniform distribution in the range [0, 1)
	std::uniform_real_distribution<> dis(0.0, 1.0);

	// Create the all point file
	std::string allPointFileName = filePath + "/pointPlanes.obj";
	std::ofstream allPointFile(allPointFileName);

	// Check if the file was created successfully
	if (!allPointFile.is_open()) {
		std::cerr << "Failed to create point plane file: " << allPointFileName << std::endl;
		std::exit(EXIT_FAILURE);
	}

	// Create the all point normal file
	std::string allPointFileName_normal = filePath + "/pointPlanes_normal.obj";
	std::ofstream allPointFile_normal(allPointFileName_normal);

	// Create the all line to point normal file
	std::string allLine2PointFileName_normal = filePath + "/line2PointPlanes_normal.obj";
	std::ofstream allLine2PointFile_normal(allLine2PointFileName_normal);

	// Create the all line file
	std::string allLineFileName = filePath + "/linePlanes.obj";
	std::ofstream allLineFile(allLineFileName);
	// Check if the file was created successfully
	if (!allLineFile.is_open()) {
		std::cerr << "Failed to create line plane file: " << allLineFileName << std::endl;
		std::exit(EXIT_FAILURE);
	}
	// Create the all point normal file
	std::string normalFileName = filePath + "/normal.txt";
	std::ofstream normalFile(normalFileName);
	// Check if the file was created successfully
	if (!normalFile.is_open()) {
		std::cerr << "Failed to create normal file: " << normalFileName << std::endl;
		std::exit(EXIT_FAILURE);
	}
	// Create the plane tag file
	std::string tagFileName = filePath + "/planeTag.txt";
	std::ofstream tagFile(tagFileName);
	// Check if the file was created successfully
	if (!tagFile.is_open()) {
		std::cerr << "Failed to create plane tag file: " << tagFileName << std::endl;
		std::exit(EXIT_FAILURE);
	}

	// Create the all line file
	size_t allLineNum = 0;

	// Create the line planes
	for (int i = 0; i < shapes.size(); i++) {

		// Get the normal of the current plane
		Vec3f normal(0, 0, 0);
		shapes[i].first->Normal(Vec3f(0, 0, 0), &normal);

		// Create random colors
		double r = dis(gen);
		double g = dis(gen);
		double b = dis(gen);

		// Create the point plane file
		std::string pointFileName = pointPlanesDir + "/pointPlane" + std::to_string(i + 1) + ".obj";
		std::ofstream pointFile(pointFileName);
		// Check if the file was created successfully
		if (!pointFile.is_open()) {
			std::cerr << "Failed to create point plane file: " << pointFileName << std::endl;
			std::exit(EXIT_FAILURE);
		}

		// Create the line plane file
		std::string lineFileName = linePlanesDir + "/linePlane" + std::to_string(i + 1) + ".obj";
		std::ofstream lineFile(lineFileName);
		// Check if the file was created successfully
		if (!lineFile.is_open()) {
			std::cerr << "Failed to create line plane file: " << lineFileName << std::endl;
			std::exit(EXIT_FAILURE);
		}

		// Get the number of points in the current plane
		size_t pointNum = shapes[i].second;
		// Get the number of lines in the current plane
		size_t lineNum = 0;

		auto it = point2pc.end() - pointNumSum - 1;

		// Iterate through the points in the current plane
		for (int j = 0; j < pointNum; j++) {
			// Get the current point
			if (it->tag == 0) {

				// Create the point normal file
				allPointFile_normal << "v" << " " << it->pos[0] << " " << it->pos[1] << " " << it->pos[2] << std::endl;
				allPointFile_normal << "vn" << " " << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;


				// Create the point plane file
				pointFile << "v " << " " << it->pos[0] << " " << it->pos[1] << " " << it->pos[2] << " " << r << " " << g << " " << b << std::endl;
				// Create the all point file
				allPointFile << "v " << " " << it->pos[0] << " " << it->pos[1] << " " << it->pos[2] << " " << r << " " << g << " " << b << std::endl;
				// Create the normal file
				normalFile << it->normal[0] << " " << it->normal[1] << " " << it->normal[2] << std::endl;
				// Create the plane ID file
				tagFile << i + 1 << std::endl;
			}
			// Create the line file
			else {
				// Create the line normal file
				allLine2PointFile_normal << "v" << " " << it->pos[0] << " " << it->pos[1] << " " << it->pos[2] << std::endl;
				allLine2PointFile_normal << "vn" << " " << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;

				// Get the current line's tag
				size_t tag = it->tag;
				// Check if the line has already been processed
				if (!Lc[tag].tag) {
					Vec3f startPoint = Lc[tag].startPoint;
					Vec3f endPoint = Lc[tag].endPoint;

					// Check if the line is close to the current plane
					if (shapes[i].first->Distance(startPoint) < 3 * m_epsilon && shapes[i].first->Distance(endPoint) < 3 * m_epsilon) {
						// Create the line file
						lineFile << "v " << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << " " << r << " " << g << " " << b << std::endl;
						lineFile << "v " << " " << endPoint[0] << " " << endPoint[1] << " " << endPoint[2] << " " << r << " " << g << " " << b << std::endl;
						// Create the all line file
						allLineFile << "v " << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << " " << r << " " << g << " " << b << std::endl;
						allLineFile << "v " << " " << endPoint[0] << " " << endPoint[1] << " " << endPoint[2] << " " << r << " " << g << " " << b << std::endl;
						// Create the line number
						lineNum++;
						allLineNum++;
						// Mark the line as processed
						Lc[tag].tag = true;
					}
				}
			}
			// Create the line connection
			--it;
		}
		// Create the line connection file
		for (size_t t = 1; t <= lineNum; t++) {
			lineFile << "l" << " " << 2 * t - 1 << " " << 2 * t << std::endl;
		}
		// Update the point number sum
		pointNumSum += pointNum;
		// Close the files
		pointFile.close();
		lineFile.close();
	}
	// Create the line connection file
	for (size_t t = 1; t <= allLineNum; t++) {
		allLineFile << "l" << " " << 2 * t - 1 << " " << 2 * t << std::endl;
	}
	// Close the files
	allPointFile.close();
	allLineFile.close();
	normalFile.close();
	tagFile.close();
	allPointFile_normal.close();
	allLine2PointFile_normal.close();
	// Create the line connection file
	//visualizationLineCloud(m_bitmapEpsilon, filePath);
	// Create the line visualization file
	visualizationLineCloud(0.0005, filePath);
}

// Create the line visualization file
void visualizationLineCloud(float m_bitmapEpsilon, const std::string& filePath) {
	// Get the original line cloud	
	LineCloud Lc;
	readLines(filePath + "/linePlanes.obj", Lc, m_bitmapEpsilon);

	// Create the line cloud file
	std::ofstream lineCloudFile(filePath + "/linePlanesWithColor.obj");
	// Check if the file was opened successfully
	if (!lineCloudFile.is_open()) {
		std::cerr << "Failed to open line cloud file: " << filePath + "/linePlanesWithColor.obj" << std::endl;
		std::exit(EXIT_FAILURE);
	}
	// Create the line segments
	for (size_t i = 1; i < Lc.size(); i++) {
		Vec3f startPoint = Lc[i].startPoint;
		Vec3f endPoint = Lc[i].endPoint;
		Vec3f color = Lc[i].color;
		lineCloudFile << "v" << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
		lineCloudFile << "v" << " " << endPoint[0] << " " << endPoint[1] << " " << endPoint[2] << " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
	}
	// Create the line samples
	for (size_t i = 1; i < Lc.size(); i++) {
		Vec3f startPoint = Lc[i].startPoint;
		Vec3f endPoint = Lc[i].endPoint;
		Vec3f color = Lc[i].color;
		for (int j = 1; j < Lc[i].num - 1; j++) {
			Vec3f samplePoint = startPoint + Lc[i].direction * (j * m_bitmapEpsilon);
			lineCloudFile << "v" << " " << samplePoint[0] << " " << samplePoint[1] << " " << samplePoint[2] << " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
		}
	}
	// Create the line connection file
	for (size_t i = 1; i < Lc.size(); i++) {
		lineCloudFile << "l" << " " << 2 * i - 1 << " " << 2 * i << std::endl;
	}
	lineCloudFile.close();
}

// Create the line normals
void estimateLineNormals(PointCloud& pc, LineCloud& lc) {
	// Iterate through each point in the point cloud
	for (size_t i = 0; i < pc.size(); i++) {
		// Get the tag of the current point
		size_t tag = pc[i].tag;
		// Check if the tag is valid
		if (tag) {
			// Accumulate the normal vectors for the corresponding line
			lc[tag].normals += pc[i].normal;
			// Store the normal vectors for the corresponding line
			lc[tag].normalsVec.push_back(pc[i].normal);
		}
	}
	// Normalize the normal vectors for each line
	for (size_t i = 1; i < lc.size(); i++) {
		lc[i].normals.normalize();
	}
}