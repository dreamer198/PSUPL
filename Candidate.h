#ifndef CANDIDATE_HEADER
#define CANDIDATE_HEADER
#include "ScoreComputer.h"
#include <MiscLib/RefCountPtr.h>
#include "PrimitiveShape.h"
#include <MiscLib/Vector.h>
#include <limits>
#include <MiscLib/RefCounted.h>
#include <MiscLib/Random.h>
#include <MiscLib/NoShrinkVector.h>
#include "Octree.h"
#include <algorithm>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE Candidate
{
public:
	Candidate();
	Candidate(PrimitiveShape *shape, size_t level);
	Candidate(PrimitiveShape* shape, size_t level, size_t plan);
	PrimitiveShape *Shape() { return m_shape; }
	void Shape(PrimitiveShape *shape) { m_shape = shape; }
	float LowerBound() const { return m_lowerBound; }
	float UpperBound() const { return m_upperBound; }
	MiscLib::RefCounted< MiscLib::Vector< size_t > > *Indices() { return m_indices; }
	void Indices(MiscLib::RefCounted< MiscLib::Vector< size_t > > *indices) { m_indices = indices; }
	size_t ComputedSubsets() const { return m_subset; }
	float ExpectedValue() const { return (m_lowerBound + m_upperBound) / 2.f; }
	void SetSubset(size_t subset) { m_subset = subset; }
	size_t Level() const { return m_level; }
	//返回所属构建方案
	size_t Plan() const { return m_plan; }
	void Reset();
	template< class ScoreVisitorT >
	bool ImproveBounds(const MiscLib::Vector< ImmediateOctreeType * > &octrees,
		const PointCloud &pc, ScoreVisitorT &scoreVisitor,
		size_t currentSize, float bitmapEpsilon,
		size_t maxSubset, size_t minPoints = 500);
	template< class ScoreVisitorT >
	void RecomputeBounds(const MiscLib::Vector< ImmediateOctreeType * > &octrees,
		const PointCloud &pc, ScoreVisitorT &scoreVisitor, size_t currentSize,
		float epsilon, float normalThresh, float bitmapEpsilon);
	void Reindex(const MiscLib::Vector< int > &newIndices, int minInvalidIndex,
		size_t mergedSubsets, const MiscLib::Vector< size_t > &subsetSizes,
		const PointCloud &pc, size_t currentSize, float epsilon,
		float normalThresh, float bitmapEpsilon);
	void Reindex(const MiscLib::Vector< size_t > &reindex);
	template< class ScoreVisitorT >
	void GlobalScore(ScoreVisitorT &scoreVisitor,
		const IndexedOctreeType &oct);
	float WeightedScore(const PointCloud &pc, float epsilon,
		float normalThresh) const;
	void ConnectedComponent(const PointCloud &pc, float bitmapEpsilon, float* borderRatio = 0 );
	template< class ScoreVisitorT >
	float GlobalWeightedScore( ScoreVisitorT &scoreVisitor, const IndexedOctreeType &oct,
		const PointCloud &pc, float epsilon, float normalThresh,
		float bitmapEpsilon );
	inline bool operator<(const Candidate &c) const;
	inline bool operator>(const Candidate &c) const;
	inline bool operator<=(const Candidate &c) const;
	inline bool operator>=(const Candidate &c) const;
	inline void Clone(Candidate *c) const;
	bool IsEquivalent(const Candidate &c, const PointCloud &pc,
		float epsilon, float normalThresh) const;
	size_t Size() const { return m_indices->size(); }

private:
	inline void GetBounds(size_t sampledPoints, size_t totalPoints);
	inline void Induct(double totalSize, double sampleSize,
		double totalCorrectSize, float *lower, float *upper) const;
	inline void Deduct(double totalSize, double sampleSize,
		double totalCorrectSize, float *lower, float *upper) const;

	void GetScore( const PointCloud& pc, float bitmapEpsilon, bool doFiltering );
	void GetScoreMaxCCSize( const PointCloud& pc, float bitmapEpsilon, bool doFiltering );
	void GetScoreMaxCCMinBorder( const PointCloud& pc, float bitmapEpsilon, bool doFiltering );

	float GetVariance( const PointCloud &pc );
	float GetPseudoVariance( const PointCloud &pc );

private:
	MiscLib::RefCountPtr< PrimitiveShape > m_shape;
	size_t m_subset;
	float m_lowerBound;
	float m_upperBound;
	MiscLib::RefCountPtr< MiscLib::RefCounted< MiscLib::Vector< size_t > > > m_indices;
	size_t m_level;
	bool m_hasConnectedComponent;
	size_t m_score;
	//所属构建方案
	size_t m_plan;
};

bool Candidate::operator<(const Candidate &c) const
{
	return ExpectedValue() < c.ExpectedValue();
}

bool Candidate::operator>(const Candidate &c) const
{
	return ExpectedValue() > c.ExpectedValue();
}

bool Candidate::operator<=(const Candidate &c) const
{
	return ExpectedValue() <= c.ExpectedValue();
}

bool Candidate::operator>=(const Candidate &c) const
{
	return ExpectedValue() >= c.ExpectedValue();
}

void Candidate::Clone(Candidate *c) const
{
	c->m_shape = m_shape->Clone();
	c->m_shape->Release();
	c->m_subset = m_subset;
	c->m_lowerBound = m_lowerBound;
	c->m_upperBound = m_upperBound;
	c->m_indices = new MiscLib::RefCounted< MiscLib::Vector< size_t > >(*m_indices);
	c->m_indices->Release();
	c->m_level = m_level;;
	c->m_hasConnectedComponent = m_hasConnectedComponent;
	c->m_score = m_score;
}

void Candidate::GetBounds(size_t sampledPoints, size_t totalPoints)
{
	Induct((double)totalPoints, (double)sampledPoints, (double)m_score, &m_lowerBound,
		&m_upperBound);
	m_lowerBound = std::max(0.f, m_lowerBound);
}

void Candidate::Induct(double totalSize, double sampleSize,
		double totalCorrectSize, float *lower, float *upper) const
{
	Deduct(-2.0 - sampleSize, -2.0 - totalSize, -1.0 - totalCorrectSize,
		lower, upper);
	*lower = -1.f - *lower;
	*upper = -1.f - *upper;
}

void Candidate::Deduct(double totalSize, double sampleSize,
	double totalCorrectSize, float *lower, float *upper) const
{
	double nI = sampleSize * totalCorrectSize;
	double denom = nI * (totalSize - sampleSize) *
		(totalSize - totalCorrectSize);
	double frac = denom / (totalSize - 1);
	double dev = std::sqrt(frac);
	*lower = float((nI - dev) / totalSize); // 95% quantile
	*upper = float((nI + dev) / totalSize); // 95% quantile
}
//采用随机子集对候选形状进行评估，更新置信区间
//octrees为点云八叉树子集的容器，pc为点云，scoreVisitor为子集分数访问器，currentSize为当前点云的大小，bitmapEpsilon为位图分辨率，
// maxSubset为子集最大数目（设置为1，表示只用最小的子集先评估一次，如果后续"期望点数"一样，再通过下一个子集来评估，不断进行细分），minPoints为最小点数（默认为500）
template< class ScoreVisitorT >
bool Candidate::ImproveBounds(const MiscLib::Vector< ImmediateOctreeType * > &octrees,
	const PointCloud &pc, ScoreVisitorT &scoreVisitor,
	size_t currentSize, float bitmapEpsilon,
	size_t maxSubset, size_t minPoints)
{
#ifdef TEST
	if (m_plan == 0) {
		Plan0ImproveBoundsNum++;
}
	else if (m_plan == 1) {
		Plan1ImproveBoundsNum++;
	}
	else {
		Plan2ImproveBoundsNum++;
	}
	//记录函数开始时间
	auto start = std::chrono::high_resolution_clock::now();
	ImproveBoundsNum++;
#endif // TEST

	
	//m_subset表示该候选形状被多少个子集评估过
	//若当前候选形状的子集数目大于设定的最大子集数，或者大于整个八叉树的子集数，则退出
	if (m_subset >= maxSubset) {
#ifdef TEST
		// 记录函数结束时间
		auto end = std::chrono::high_resolution_clock::now();
		// 计算运行时间
		ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
		return false;
	}
		
	if (m_subset >= octrees.size()) {
#ifdef TEST
		// 记录函数结束时间
		auto end = std::chrono::high_resolution_clock::now();
		// 计算运行时间
		ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
		return false;
	}

	//累计已经用于评估候选形状的样本点数目（从最小的子集开始）
	size_t sampledPoints = 0;
	for(size_t i = 0; i < m_subset; ++i)
		sampledPoints += octrees[i]->size();
	//统计新加入的用于评估候选形状的样本点数目
	size_t newlySampledPoints = 0;
	//m_indices是候选形状中的点索引，对于建立的初始形状而言，m_indices还是空指针
	scoreVisitor.SetIndices(m_indices);
	do
	{
		//设置当前得分访问器的八叉树，从最底层的八叉树开始
		scoreVisitor.SetOctree(*octrees[m_subset]);
		//通过子集八叉树对候选形状评分
		m_shape->Visit(&scoreVisitor);
		//累计样本点
		newlySampledPoints += octrees[m_subset]->size();
		sampledPoints += octrees[m_subset]->size();
		//修改已经对候选形状进行评分的子集数目
		++m_subset;
	}
	//如果已用于评估的子集数目小于子集总数，同时新加入评估的点数小于预设的最少点数，则不断纳入新的更大的子集用于评估平面
	while(m_subset < octrees.size() && newlySampledPoints < minPoints);
	//这里的得分应该是在当前评估子集中得分之和（表示有多少正确点）
	m_score = m_indices->size();
	//得到置信区间上下界
	GetBounds(sampledPoints, currentSize);

	// check if connected component is worthwhile
	//如果只用了最小的子集来评估平面，则直接退出
    if( m_subset == 1)
	{
#ifdef TEST
		// 记录函数结束时间
		auto end = std::chrono::high_resolution_clock::now();
		// 计算运行时间
		ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
		return true;
	}
	if( m_hasConnectedComponent ||
		(2.f * ( m_upperBound - ( m_lowerBound / .7f)) /
		(m_upperBound + (m_lowerBound / .7f))) < .3f)
	{
		if(!m_hasConnectedComponent && m_indices->size() < 2)
		{
#ifdef TEST
			// 记录函数结束时间
			auto end = std::chrono::high_resolution_clock::now();
			// 计算运行时间
			ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
			return true;
		}
		//标志该候选平面已进行连通性检查
		m_hasConnectedComponent = true;
		/*oldSize为连通性检查前，候选平面中点的数目*/
		size_t oldSize = m_indices->size();
		//返回最大连通分量中点的数目，更新候选平面得分
		m_score = m_shape->ConnectedComponent(pc,
			(4 << ((octrees.size() - m_subset) / 2)) * bitmapEpsilon,
			m_indices, false);
		//得到进行了连通性检查的属于这个平面的所有点的索引
		m_indices->resize(m_score);
		//如果所有的子集都用于评估该候选平面了，那么相当于直接用整个点云评估该平面，此时置信区间的上下界和候选平面得分都相等
		if(m_subset >= octrees.size())
		{
			GetScore( pc, bitmapEpsilon, true);
			m_upperBound = m_lowerBound = m_score;
#ifdef TEST
			// 记录函数结束时间
			auto end = std::chrono::high_resolution_clock::now();
			// 计算运行时间
			ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
			return true;
		}
		//子集规模不同，位图分辨率也不一样，因为子集越小，点的距离约远，因此为了评分的公平性，需要更大的位图距离
		GetScore( pc, (2 << ((octrees.size() - m_subset) / 2)) * bitmapEpsilon, false);
		//更新上下界
		GetBounds( sampledPoints, currentSize);
	}
#ifdef TEST
	// 记录函数结束时间
	auto end = std::chrono::high_resolution_clock::now();
	// 计算运行时间
	ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
	return true;
}

template< class ScoreVisitorT >
void Candidate::RecomputeBounds(const MiscLib::Vector< ImmediateOctreeType * > &octrees,
	const PointCloud &pc, ScoreVisitorT &scoreVisitor, size_t currentSize,
	float epsilon, float normalThresh, float bitmapEpsilon)
{
	// run over indices and check if still unassigned
	const MiscLib::Vector< int > &shapeIndex = scoreVisitor.GetShapeIndex();
	size_t indicesSize = m_indices->size();
	for(size_t i = 0; i < indicesSize;)
	{
		if(shapeIndex[(*m_indices)[i]] != -1)
		{
			--indicesSize;
			std::swap((*m_indices)[i], (*m_indices)[indicesSize]);
		}
		else
			++i;
	}

	m_score = indicesSize;

	if(m_hasConnectedComponent)
	{
		if(m_indices->size() != indicesSize) // dirty!
		{
			if((float)indicesSize / m_indices->size() > 0.95)
			{
				m_indices->resize(indicesSize);

				GetScore( pc, m_subset >= octrees.size()? bitmapEpsilon :
						(4 << ((octrees.size() - m_subset) / 2)) * bitmapEpsilon,
						m_subset >= octrees.size());

				if(m_subset >= octrees.size())
				{
					m_upperBound = m_lowerBound = (float)m_indices->size();
					return;
				}
				// reevaluate bounds
			}
			else
			{
				m_subset = 0;
				m_hasConnectedComponent = false;
				m_indices->clear();
				m_score = 0;
				ImproveBounds(octrees, pc, scoreVisitor,
					currentSize, bitmapEpsilon, 1);
				return;
			}
		}
		else if(m_subset > octrees.size())
			return;
		// reevaluate bounds
	}
	else
	{
		m_indices->resize(indicesSize);
		m_score = m_indices->size();
	}
	size_t sampledPoints = 0,
	endi = std::min(m_subset, octrees.size());
	for(size_t i = 0; i < endi; ++i)
		sampledPoints += octrees[i]->size();

	GetBounds(sampledPoints, currentSize);
}

template< class ScoreVisitorT >
void Candidate::GlobalScore(ScoreVisitorT &scoreVisitor,
	const IndexedOctreeType &oct)
{
	m_indices->clear();
	scoreVisitor.SetOctree(oct);
	scoreVisitor.SetIndices(m_indices);
	m_shape->Visit(&scoreVisitor);
	m_lowerBound = m_upperBound = (float)m_indices->size();
}

template< class ScoreVisitorT >
float Candidate::GlobalWeightedScore( ScoreVisitorT &scoreVisitor, const IndexedOctreeType &oct,
	const PointCloud &pc, float epsilon, float normalThresh,
	float bitmapEpsilon )
{
	GlobalScore( scoreVisitor, oct );
	ConnectedComponent( pc, bitmapEpsilon );
	return WeightedScore( pc, epsilon, normalThresh );
}

#endif
