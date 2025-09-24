#pragma once
#include <basic.h>
#include <PrimitiveShape.h>
#include <vector>
#include <MiscLib/RefCountPtr.h>
#include <MiscLib/RefCounted.h>
#include <chrono>
// Input type: 0 for point cloud, 1 for line cloud, 2 for both  
// 0 indicates point cloud, 1 indicates line cloud, 2 indicates both point and line cloud
constexpr int INPUT_TYPE = 2;

// Whether to perform quantitative experiments
constexpr bool isQuantitativeExperiment = false;

extern int ImproveBoundsNum;
extern int FindBestCandidateNum;
extern std::chrono::microseconds ImproveBoundsTime;
extern std::chrono::microseconds FindBestCandidateTime;
extern size_t number_of_Candidates;

// Line segment structure
struct DLL_LINKAGE Line
{
    // Start point
    Vec3f startPoint;
    // End point
    Vec3f endPoint;
    // Tag indicating whether the line segment has been processed (default is false)
    bool tag = false;
    // Effective length of the line segment (default is 2)
    size_t num = 2;
    // Segment length
    float segmentLength = 1;
    // Direction vector
    Vec3f direction;
    // Line color
    Vec3f color;
    // Midpoint of the line segment
    Vec3f midPoint;
    // Normal vector of the line segment
    Vec3f normals = Vec3f(0.0, 0.0, 0.0);

    // Line fitting algorithm parameters
    std::vector<Vec3f> normalsVec;

    // Constructor
    Line() {}
    Line(const Vec3f& StartPoint, const Vec3f& EndPoint, const float& SegmentLength, const Vec3f& Direction, size_t Num) {
        startPoint = StartPoint;
        endPoint = EndPoint;
        segmentLength = SegmentLength;
        direction = Direction;
        num = Num;
        midPoint = (StartPoint + EndPoint) / 2;
    }
    Line(const Vec3f& StartPoint, const Vec3f& EndPoint, size_t Num) {
        startPoint = StartPoint;
        endPoint = EndPoint;
        midPoint = (StartPoint + EndPoint) / 2;
        num = Num;
    }
};

using LineCloud = MiscLib::Vector< Line >;

// Get points from file, filename is the file path, point2pc is the point cloud data
void readPoints(const std::string& filename, PointCloud& point2pc);

// Get lines from file, filename is the file path, lc is the line cloud data
void readLines(const std::string& filename, LineCloud& lc, double m_bitmapEpsilon);

// Convert line cloud to point cloud
void lc2pc(LineCloud& lc, double m_epsilon, PointCloud& line2pc);

// Delete directory contents
void deleteDirectoryContents(const std::string& dirPath = "./output");

// Split path into components
std::vector<std::string> splitPath(const std::string& path);

// Create directory recursively
void createDirectoryRecursively(const std::string& path = "./output");

// Write point-plane correspondences, shapes store the corresponding primitive shapes, point2pc stores the actual point cloud, Lc stores the actual line cloud, ransacOptions are the RANSAC parameters, filePath is the output file path (default is output)
void writePointPlane(MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>>& shapes,
    PointCloud& point2pc, LineCloud& Lc, float m_epsilon, float m_bitmapEpsilon, const std::string& filePath = "./output");

// Create line visualization file
void visualizationLineCloud(float m_bitmapEpsilon, const std::string& filePath = "./output");

// Estimate line normals
void estimateLineNormals(PointCloud& pc, LineCloud& lc);