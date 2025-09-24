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
	//����������������
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
	//������������
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
//��������Ӽ��Ժ�ѡ��״����������������������
//octreesΪ���ư˲����Ӽ���������pcΪ���ƣ�scoreVisitorΪ�Ӽ�������������currentSizeΪ��ǰ���ƵĴ�С��bitmapEpsilonΪλͼ�ֱ��ʣ�
// maxSubsetΪ�Ӽ������Ŀ������Ϊ1����ʾֻ����С���Ӽ�������һ�Σ��������"��������"һ������ͨ����һ���Ӽ������������Ͻ���ϸ�֣���minPointsΪ��С������Ĭ��Ϊ500��
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
	//��¼������ʼʱ��
	auto start = std::chrono::high_resolution_clock::now();
	ImproveBoundsNum++;
#endif // TEST

	
	//m_subset��ʾ�ú�ѡ��״�����ٸ��Ӽ�������
	//����ǰ��ѡ��״���Ӽ���Ŀ�����趨������Ӽ��������ߴ��������˲������Ӽ��������˳�
	if (m_subset >= maxSubset) {
#ifdef TEST
		// ��¼��������ʱ��
		auto end = std::chrono::high_resolution_clock::now();
		// ��������ʱ��
		ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
		return false;
	}
		
	if (m_subset >= octrees.size()) {
#ifdef TEST
		// ��¼��������ʱ��
		auto end = std::chrono::high_resolution_clock::now();
		// ��������ʱ��
		ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
		return false;
	}

	//�ۼ��Ѿ�����������ѡ��״����������Ŀ������С���Ӽ���ʼ��
	size_t sampledPoints = 0;
	for(size_t i = 0; i < m_subset; ++i)
		sampledPoints += octrees[i]->size();
	//ͳ���¼��������������ѡ��״����������Ŀ
	size_t newlySampledPoints = 0;
	//m_indices�Ǻ�ѡ��״�еĵ����������ڽ����ĳ�ʼ��״���ԣ�m_indices���ǿ�ָ��
	scoreVisitor.SetIndices(m_indices);
	do
	{
		//���õ�ǰ�÷ַ������İ˲���������ײ�İ˲�����ʼ
		scoreVisitor.SetOctree(*octrees[m_subset]);
		//ͨ���Ӽ��˲����Ժ�ѡ��״����
		m_shape->Visit(&scoreVisitor);
		//�ۼ�������
		newlySampledPoints += octrees[m_subset]->size();
		sampledPoints += octrees[m_subset]->size();
		//�޸��Ѿ��Ժ�ѡ��״�������ֵ��Ӽ���Ŀ
		++m_subset;
	}
	//����������������Ӽ���ĿС���Ӽ�������ͬʱ�¼��������ĵ���С��Ԥ������ٵ������򲻶������µĸ�����Ӽ���������ƽ��
	while(m_subset < octrees.size() && newlySampledPoints < minPoints);
	//����ĵ÷�Ӧ�����ڵ�ǰ�����Ӽ��е÷�֮�ͣ���ʾ�ж�����ȷ�㣩
	m_score = m_indices->size();
	//�õ������������½�
	GetBounds(sampledPoints, currentSize);

	// check if connected component is worthwhile
	//���ֻ������С���Ӽ�������ƽ�棬��ֱ���˳�
    if( m_subset == 1)
	{
#ifdef TEST
		// ��¼��������ʱ��
		auto end = std::chrono::high_resolution_clock::now();
		// ��������ʱ��
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
			// ��¼��������ʱ��
			auto end = std::chrono::high_resolution_clock::now();
			// ��������ʱ��
			ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
			return true;
		}
		//��־�ú�ѡƽ���ѽ�����ͨ�Լ��
		m_hasConnectedComponent = true;
		/*oldSizeΪ��ͨ�Լ��ǰ����ѡƽ���е����Ŀ*/
		size_t oldSize = m_indices->size();
		//���������ͨ�����е����Ŀ�����º�ѡƽ��÷�
		m_score = m_shape->ConnectedComponent(pc,
			(4 << ((octrees.size() - m_subset) / 2)) * bitmapEpsilon,
			m_indices, false);
		//�õ���������ͨ�Լ����������ƽ������е������
		m_indices->resize(m_score);
		//������е��Ӽ������������ú�ѡƽ���ˣ���ô�൱��ֱ������������������ƽ�棬��ʱ������������½�ͺ�ѡƽ��÷ֶ����
		if(m_subset >= octrees.size())
		{
			GetScore( pc, bitmapEpsilon, true);
			m_upperBound = m_lowerBound = m_score;
#ifdef TEST
			// ��¼��������ʱ��
			auto end = std::chrono::high_resolution_clock::now();
			// ��������ʱ��
			ImproveBoundsTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
#endif // TEST
			return true;
		}
		//�Ӽ���ģ��ͬ��λͼ�ֱ���Ҳ��һ������Ϊ�Ӽ�ԽС����ľ���ԼԶ�����Ϊ�����ֵĹ�ƽ�ԣ���Ҫ�����λͼ����
		GetScore( pc, (2 << ((octrees.size() - m_subset) / 2)) * bitmapEpsilon, false);
		//�������½�
		GetBounds( sampledPoints, currentSize);
	}
#ifdef TEST
	// ��¼��������ʱ��
	auto end = std::chrono::high_resolution_clock::now();
	// ��������ʱ��
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
