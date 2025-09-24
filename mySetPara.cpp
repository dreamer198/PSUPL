#include"mySetPara.h"
void setRansacOptions(RansacShapeDetector::Options& options, const std::string& type) {
	if (type == "sphere") {
		options.m_epsilon = 0.01;
		options.m_bitmapEpsilon = 0.05;
		options.m_normalThresh = .95f;
		options.m_minSupport = 75;
		options.m_probability = .001f;
	}
	else {
		std::cout << "Unknown type" << std::endl;
	}
}