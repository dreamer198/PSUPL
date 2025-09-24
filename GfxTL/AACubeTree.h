#ifndef GfxTL__AACUBETREE_HEADER__
#define GfxTL__AACUBETREE_HEADER__
#include <deque>
#include <GfxTL/BaseTree.h>
#include <GfxTL/AACube.h>
#include <GfxTL/AABox.h>
#include <GfxTL/VectorKernel.h>
#include <GfxTL/NullClass.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <unordered_set>

namespace GfxTL
{
	template< unsigned int DimT, class BaseT >
	class AACubeTreeCell
		: public BaseT
	{
	public:
		enum { Dim = DimT, NChildren = 1 << DimT };
		typedef BaseT BaseType;
		typedef AACubeTreeCell< DimT, BaseT > ThisType;

		AACubeTreeCell()
		{
			//pointIdx = new std::vector<size_t>();
			//line2pointIdx = new std::vector<size_t>();
			for (size_t i = 0; i < 1 << DimT; ++i)
				m_children[i] = NULL;
		}

		~AACubeTreeCell()
		{
			for (size_t i = 0; i < 1 << DimT; ++i)
				if (m_children[i] > (ThisType*)1)
					delete m_children[i];
		}

		ThisType& operator[](unsigned int i)
		{
			return *m_children[i];
		}

		const ThisType& operator[](unsigned int i) const
		{
			return *m_children[i];
		}

		void Child(unsigned int i, ThisType* cell)
		{
			m_children[i] = cell;
		}

		ThisType** const Children()
		{
			return m_children;
		}

		const ThisType* const* const Children() const
		{
			return m_children;
		}

		//private:
		ThisType* m_children[1 << DimT];
		//�ýڵ��ڲ���ʵ������
		std::vector<size_t> pointIdx;
		//�ýڵ��ڲ���ת������
		std::vector<size_t> line2pointIdx;
		//�ýڵ��ڲ�������
		std::vector<size_t> lineIdx;
	};

	template< class DataStrategyT >
	struct BaseAACubeTreeStrategy
	{
		typedef typename DataStrategyT::value_type value_type;

		class CellData
		: public DataStrategyT::CellData
		{};

		template< class BaseT >
		class StrategyBase
		: public DataStrategyT::template StrategyBase< BaseT >
		{
		public:
			typedef typename DataStrategyT::template StrategyBase< BaseT > BaseType;
			typedef typename BaseType::CellType CellType;
			//����ά��
			enum { Dim = CellType::Dim };
			typedef typename ScalarTypeDeferer< value_type >::ScalarType ScalarType;
			typedef AACube< VectorXD< Dim, ScalarType > > CubeType;
			typedef CubeType CellCubeType;
			typedef GfxTL::VectorXD< Dim, ScalarType > CellCenterType;

			const CubeType &RootCube() const { return m_rootCube; }

		protected:
			template< class BuildInformationT >
			bool ShouldSubdivide(const CellType &cell, BuildInformationT &bi) const
			{
				return false;
			}

			template< class BuildInformationT >
			void InitRootBuildInformation(BuildInformationT *bi)
			{
				BaseType::InitRootBuildInformation(bi);
				m_rootCube = bi->Cube();
			}

			template< class BuildInformationT >
			void InitGlobalBuildInformation(const CellType &root,
				const BuildInformationT &bi)
			{}

			template< class BuildInformationT >
			void EnterGlobalBuildInformation(const CellType &cell,
				BuildInformationT *bi) const
			{}

			template< class BuildInformationT >
			void LeaveGlobalBuildInformation(const CellType &cell,
				const BuildInformationT &bi) const
			{}

			template< class BuildInformationT >
			void InitLeaf(CellType *cell, const BuildInformationT &bi)
			{}

			template< class BuildInformationT >
			void InitSubdivided(const BuildInformationT &bi, CellType *cell)
			{}

			template< class TraversalBaseT >
			class CellCubeTraversalInformation
			: public TraversalBaseT
			{
			public:
				typedef AACube< VectorXD< Dim, ScalarType > > CubeType;
				CubeType &Cube() { return m_cube; }
				const CubeType &Cube() const { return m_cube; }

			private:
				CubeType m_cube;
			};

			template< class TraversalBaseT >
			class CellCenterTraversalInformation
			: public TraversalBaseT
			{
			public:
				typedef typename TraversalBaseT::CubeType CubeType;
				CubeType &Cube() { return m_cube; }
				const CubeType &Cube() const { return m_cube; }

			private:
				CubeType m_cube;
			};

			template< class TraversalBaseT >
			class TraversalInformationBase
			: public DataStrategyT::template StrategyBase< BaseT >
				::template TraversalInformation< TraversalBaseT >
			{};

			template< class TraversalInformationT >
			void InitRootTraversalInformation(const CellType &root,
				TraversalInformationT *ti) const
			{
				BaseType::InitRootTraversalInformation(root, ti);
				InitCubeRootTraversalInformation(root, ti);
				InitCenterRootTraversalInformation(root, ti);
			}

			template< class TraversalInformationT >
			void InitTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi, unsigned int childIdx,
				TraversalInformationT *ti) const
			{
				BaseType::InitTraversalInformation(parent, pTi, childIdx, ti);
				InitCubeTraversalInformation(parent, pTi, childIdx, ti);
				InitCenterTraversalInformation(parent, pTi, childIdx, ti);
			}

			template< class TraversalBaseT >
			void CellCube(const CellType &cell,
				const CellCubeTraversalInformation< TraversalBaseT > &ti,
				CellCubeType *cube) const
			{
				*cube = ti.Cube();
			}

			template< class TraversalBaseT >
			void CellCenter(const CellType &cell,
				const CellCenterTraversalInformation< TraversalBaseT > &ti,
				CellCenterType *center) const
			{
				ti.Cube().Center(center);
			}

		private:
			template< class TraversalInformationT >
			void InitCubeRootTraversalInformation(const CellType &root,
				TraversalInformationT *ti) const
			{}

			template< class TraversalBaseT >
			void InitCubeRootTraversalInformation(const CellType &root,
				CellCubeTraversalInformation< TraversalBaseT > *ti) const
			{
				ti->Cube() = m_rootCube;
			}

			template< class TraversalInformationT >
			void InitCubeTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi,
				unsigned int childIdx, TraversalInformationT *ti) const
			{}

			template< class TraversalBaseT >
			void InitCubeTraversalInformation(const CellType &parent,
				const CellCubeTraversalInformation< TraversalBaseT > &pTi,
				unsigned int childIdx, CellCubeTraversalInformation< TraversalBaseT > *ti) const
			{
				ti->Cube().Width(pTi.Cube().Width() / ScalarType(2));
				for(unsigned int i = 0; i < Dim; ++i)
					if(childIdx & (1 << (Dim - 1 - i)))
						ti->Cube().Min()[i] =
							pTi.Cube().Min()[i] + ti->Cube().Width();
					else
						ti->Cube().Min()[i] = pTi.Cube().Min()[i];
			}

			template< class TraversalInformationT >
			void InitCenterRootTraversalInformation(const CellType &root,
				TraversalInformationT *ti) const
			{}

			template< class TraversalBaseT >
			void InitCenterRootTraversalInformation(const CellType &root,
				CellCenterTraversalInformation< TraversalBaseT > *ti) const
			{
				ti->Cube() = m_rootCube;
			}

			template< class TraversalInformationT >
			void InitCenterTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi,
				unsigned int childIdx, TraversalInformationT *ti) const
			{}

			template< class TraversalBaseT >
			void InitCenterTraversalInformation(const CellType &parent,
				const CellCenterTraversalInformation< TraversalBaseT > &pTi,
				unsigned int childIdx, CellCenterTraversalInformation< TraversalBaseT > *ti) const
			{
				ti->Cube().Width(pTi.Cube().Width() / ScalarType(2));
				for(unsigned int i = 0; i < Dim; ++i)
					if(childIdx & (1 << (Dim - 1 - i)))
						ti->Cube().Min()[i] =
							pTi.Cube().Min()[i] + ti->Cube().Width();
					else
						ti->Cube().Min()[i] = pTi.Cube().Min()[i];
			}

		private:
			CubeType m_rootCube;
		};
	};

	template< unsigned int DimT, class StrategiesT,
		template< unsigned int > class VectorKernelT = VectorKernelD >
	class AACubeTree
	: public StrategiesT::template StrategyBase
		<
			typename VectorKernelT< DimT >::template VectorKernelType
			<
				BaseTree
				<
					AACubeTreeCell< DimT, typename StrategiesT::CellData >
				>
			>
		>
	{
	public:
		typedef StrategiesT StrategiesType;
		typedef AACubeTreeCell< DimT, typename StrategiesT::CellData > CellType;
		typedef typename StrategiesT::template StrategyBase
		<
			typename VectorKernelT< DimT >::template VectorKernelType
			<
				BaseTree
				<
					AACubeTreeCell< DimT, typename StrategiesT::CellData >
				>
			>
		> BaseType;
		typedef AACubeTree< DimT, StrategiesT, VectorKernelT > ThisType;
		typedef typename BaseType::value_type value_type;
		typedef typename ScalarTypeDeferer< value_type >::ScalarType ScalarType;
		typedef typename BaseType::CellRange CellRange;

		void Build()
		{
			AABox< VectorXD< DimT, ScalarType > > bbox;
			bbox.Bound(BaseType::begin(), BaseType::end());
			ScalarType width = bbox.Max()[0] - bbox.Min()[0];
			for(unsigned int i = 1; i < DimT; ++i)
				width = std::max(width, bbox.Max()[i] - bbox.Min()[i]);
			AACube< VectorXD< DimT, ScalarType > > bcube(bbox.Min(), width);
			Build(bcube);
		}

		void Build(const AACube< VectorXD< DimT, ScalarType > > &bcube)
		{
			typedef std::pair< CellType *, BuildInformation > Pair;
			BaseType::Clear();
			BaseType::Root() = new CellType;
			std::deque< Pair > stack(1);
			// init build information directly on stack to avoid
			// copying.
			stack.back().first = BaseType::Root();
			InitRootBuildInformation(bcube, &stack.back().second);
			InitRoot(stack.back().second, BaseType::Root());
			BaseType::InitGlobalBuildInformation(*BaseType::Root(), stack.back().second);
			while(stack.size())
			{
				Pair &p = stack.back();
				if(p.second.CreateChild() == 1 << DimT)
				{
					BaseType::LeaveGlobalBuildInformation(*p.first, p.second);
					stack.pop_back();
					continue;
				}
				if(IsLeaf(*p.first))
				{
					if(!ShouldSubdivide(*p.first, p.second))
					{
						BaseType::InitLeaf(p.first, p.second);
						stack.pop_back();
						continue;
					}
					Subdivide(p.second, p.first);
					if(IsLeaf(*p.first)) // couldn't subdivide?
					{
						BaseType::InitLeaf(p.first, p.second);
						stack.pop_back();
						continue;
					}
					BaseType::InitSubdivided(p.second, p.first);
				}
				else
					BaseType::LeaveGlobalBuildInformation(*p.first, p.second);
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()))
					++p.second.CreateChild();
				if(p.second.CreateChild() == (1 << DimT))
				{
					stack.pop_back();
					continue;
				}
				BaseType::EnterGlobalBuildInformation(*p.first, &p.second);
				stack.resize(stack.size() + 1); // create new entry
				stack.back().first = &(*p.first)[p.second.CreateChild()];
				InitBuildInformation(*p.first, p.second,
					p.second.CreateChild(), &stack.back().second);
				InitCell(*p.first, p.second, p.second.CreateChild(),
					stack.back().second, &(*p.first)[p.second.CreateChild()]);
				do
				{
					++p.second.CreateChild();
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()));
			}
		}
		//maxLevelΪ�˲��������㼶�����ǶԽ��Ĳ㼶Ҫ��minSizeΪ������������������Ҳ�ǶԽ������ٵ���Ҫ��
		template< class PointT >
		const CellType *NodeContainingPoint(const PointT &point,
			size_t maxLevel, CellRange *range) const
		{

			//������ݽṹ�ĸ��ڵ��Ƿ���ڡ�������ڵ㲻���ڣ��� range ��ʼ��Ϊ(0, 0)�������� NULL
			if(!BaseType::Root())
			{
				range->first = 0;
				range->second = 0;
				return NULL;
			}
			//BaseType::Root();
			// ���� TraversalInfoType����ʼ�����ڵ�ı�����Ϣ
			typedef typename BaseType::template CellLevelTraversalInformation
				<
					typename BaseType::template CellCenterTraversalInformation
					<
						typename BaseType::template TraversalInformationBase< NullClass >
					>
				> TraversalInfoType;
			TraversalInfoType rti;
			// ��ȡ���ڵ�ķ�Χ
			BaseType::InitRootTraversalInformation(*BaseType::Root(), &rti);
			BaseType::GetCellRange(*BaseType::Root(), rti, range);
			// �������ص� NodeContainingPoint ��������ʵ�ʵĲ���
			return NodeContainingPoint(point, maxLevel, *BaseType::Root(), rti, range);
		}
		
		bool TraverseOctree()
		{
			// ������ݽṹ�ĸ��ڵ��Ƿ���ڡ�������ڵ㲻���ڣ��� range ��ʼ��Ϊ(0, 0)��ֱ���˳�
			if (!BaseType::Root())
			{
				return false;
			}
			// ���� TraversalInfoType����ʼ�����ڵ�ı�����Ϣ
			typedef typename BaseType::template CellLevelTraversalInformation
				<
				typename BaseType::template CellCenterTraversalInformation
				<
				typename BaseType::template TraversalInformationBase< NullClass >
				>
				> TraversalInfoType;
			TraversalInfoType rti;
			CellRange range;
			// ��ȡ���ڵ�ķ�Χ
			BaseType::InitRootTraversalInformation(*BaseType::Root(), &rti);
			BaseType::GetCellRange(*BaseType::Root(), rti, &range);
			// �������ص� TraverseOctree ��������ʵ�ʹ���
			TraverseOctree(*BaseType::Root(), rti, &range);
			return true;
		}

		// �ݹ�����˲����ĸ����ڵ�
		template< class TraversalInformationT >
		void TraverseOctree(CellType& cell, const TraversalInformationT& ti, CellRange* range) {
			//������յ�ǰ�ڵ�ĵ㡢��ת�㡢������
			cell.pointIdx.clear();
			cell.line2pointIdx.clear();
			cell.lineIdx.clear();
			// �����ǰ�ڵ�ΪҶ�ڵ㣬��ֱ�ӻ�ȡ��ǰ�㷶Χ���������е㣬ȷ����𣬹����㡢��ת����������
			if (IsLeaf(cell))
			{
				//�����ϣ��
				std::unordered_set<size_t> uniqueSet;
				// ��ȡ��ǰҶ�ڵ��������Χ
				GetCellRange(cell, ti, range);
				// ������ǰҶ�ڵ��ڵ����е�
				for (unsigned int i = 0; i < cell.Size(); i++) {
					//��ȡ��ǰ������ͱ�ǩ
					size_t tag = BaseType::at(BaseType::Dereference(i + range->first)).tag;
					//�������ʵ��
					if (tag == 0) {
						//���������
						cell.pointIdx.push_back(BaseType::Dereference(i + range->first));
					}
					// ����������ת��
					else {
						//������ת������
						cell.line2pointIdx.push_back(BaseType::Dereference(i + range->first));
						// ��鲢����
						if (uniqueSet.insert(tag).second) {
							cell.lineIdx.push_back(tag);
						}
					}
				}
			}
			// ����ǰ�ڵ�Ϊ��Ҷ�ӽڵ�
			else {
				// �����ϣ�����ںϲ���ȥ��
				std::unordered_set<size_t> uniqueLineIdx;
				for (unsigned int childIdx = 0; childIdx < 8; ++childIdx) { // ��8���ӽڵ�
					// ���ӽڵ������ݹ�
					if (ExistChild(cell, childIdx)) {
						TraversalInformationT cti;
						BaseType::InitTraversalInformation(cell, ti, childIdx, &cti);
						// ������ȵݹ�
						TraverseOctree(cell[childIdx], cti, range);
						// �ϲ��ӽڵ�ĵ���������ǰ�ڵ�
						cell.pointIdx.insert(cell.pointIdx.end(), cell[childIdx].pointIdx.begin(), cell[childIdx].pointIdx.end());
						cell.line2pointIdx.insert(cell.line2pointIdx.end(), cell[childIdx].line2pointIdx.begin(), cell[childIdx].line2pointIdx.end());
						// ���ӽڵ��lineIdx���뵽��ϣ����
						uniqueLineIdx.insert(cell[childIdx].lineIdx.begin(), cell[childIdx].lineIdx.end());
					}
				}
				// ��ȥ�غ��Ԫ����ӵ���ǰ�ڵ��lineIdx��
				cell.lineIdx.insert(cell.lineIdx.end(), uniqueLineIdx.begin(), uniqueLineIdx.end());
			}
		}
		
		//���ӻ��˲����������ڵ��ڵĵ���߶β��ò�ͬ����ɫ����ֻ����㣩
		void visualization(CellType& cell,int depth,int maxdepth) {
			// �����ǰ�ڵ�ΪҶ�ڵ㣬�����ѡ��һ����ɫ�����
			if (IsLeaf(cell)|| depth== maxdepth)
			{
				// ���������ļ�
				std::string pointFileName = "./output/pointoct.obj";
				std::string lineFileName = "./output/lineoct.obj";
				std::ofstream pointOutputFile(pointFileName, std::ios::app);
				std::ofstream lineOutputFile(lineFileName, std::ios::app);
				// ����һ�����������
				std::random_device rd;  // ��ȡ���������������
				std::mt19937 gen(rd()); // ʹ�� Mersenne Twister ����
				// ����һ�����ȷֲ��� 0 �� 1 ֮��ĸ������ֲ�
				std::uniform_real_distribution<> dis(0.0, 1.0);
				// ���������ɫ
				double r = dis(gen);
				double g = dis(gen);
				double b = dis(gen);
				// �����ڵ��ڵ����е�
				for (int i = 0; i < cell.pointIdx.size(); i++) {
					pointOutputFile << "v" << " " << BaseType::at(cell.pointIdx[i]).pos[0] << " " << BaseType::at(cell.pointIdx[i]).pos[1] << " "
						<< BaseType::at(cell.pointIdx[i]).pos[2] << " " << r << " " << g << " " << b << std::endl;
				}
				// ���������߶�
				for (int i = 0; i < cell.line2pointIdx.size(); i++) {
					lineOutputFile << "v" << " " << BaseType::at(cell.line2pointIdx[i]).pos[0] << " " << BaseType::at(cell.line2pointIdx[i]).pos[1] << " "
						<< BaseType::at(cell.line2pointIdx[i]).pos[2] << " " << r << " " << g << " " << b << std::endl;
				}
				//�ر��ļ�
				pointOutputFile.close();
				lineOutputFile.close();
			}
			// ����ǰ�ڵ�Ϊ��Ҷ�ӽڵ�
			else {
				for (unsigned int childIdx = 0; childIdx < 8; ++childIdx) { // ��8���ӽڵ�
					// ���ӽڵ������ݹ�
					if (ExistChild(cell, childIdx)) {
						// ������ȵݹ�
						visualization(cell[childIdx], depth + 1, maxdepth);
					}
				}
			}
		}

		//���˲����ṹд�������
		void Serialize(std::ostream *out) const
		{
			typedef std::pair< const CellType *, SerializeInformation > Pair;
			std::deque< Pair > stack(1);
			// init build information directly on stack to avoid
			// copying.
			stack.back().first = BaseType::Root();
			InitRootSerializeInformation(&stack.back().second);
			while(stack.size())
			{
				Pair &p = stack.back();
				if(p.second.CreateChild() == 1 << DimT)
				{
					stack.pop_back();
					continue;
				}
				if(!ShouldSubdivide(*p.first, p.second))
				{
					stack.pop_back();
					continue;
				}
				if(!p.second.Written())
				{
					const size_t byteCount =
						((1 << BaseType::m_dim) / 8) + (((1 << BaseType::m_dim) % 8)? 1 : 0);
					char b[byteCount];
					for(unsigned int i = 0; i < (1 << DimT); ++i)
						if(ExistChild(*p.first, i))
							b[i >> 3] |= 1 << (i - ((i >> 3) << 3));
						else
							b[i >> 3] &= ~(1 << (i - ((i >> 3) << 3)));
					out->write(b, byteCount);
					p.second.Written(true);
				}
				else
					LeaveGlobalBuildInformation(*p.first, p.second);
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()))
					++p.second.CreateChild();
				if(p.second.CreateChild() == (1 << DimT))
					continue;
				EnterGlobalBuildInformation(*p.first, &p.second);
				stack.resize(stack.size() + 1); // create new entry
				stack.back().first = &(*p.first)[p.second.CreateChild()];
				InitSerializeInformation(*p.first, p.second,
					p.second.CreateChild(), &stack.back().second);
				do
				{
					++p.second.CreateChild();
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()));
			}
		}

		void SerializeBreadthFirst(std::ostream *out)
		{
			typedef std::pair< const CellType *, SerializeInformation > Pair;
			std::deque< Pair > queue(1);
			// init build information directly on stack to avoid
			// copying.
			queue.back().first = BaseType::Root();
			InitRootSerializeInformation(&queue.back().second);
			while(queue.size())
			{
				Pair &p = queue.front();
				if(p.second.CreateChild() == 1 << DimT)
				{
					queue.pop_front();
					continue;
				}
				if(!ShouldSubdivide(*p.first, p.second))
				{
					queue.pop_front();
					continue;
				}
				if(!p.second.Written())
				{
					const size_t byteCount =
						((1 << BaseType::m_dim) / 8) + (((1 << BaseType::m_dim) % 8)? 1 : 0);
					char b[byteCount];
					for(unsigned int i = 0; i < (1 << DimT); ++i)
						if(ExistChild(*p.first, i))
							b[i >> 3] |= 1 << (i - ((i >> 3) << 3));
						else
							b[i >> 3] &= ~(1 << (i - ((i >> 3) << 3)));
					out->write(b, byteCount);
					p.second.Written(true);
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()))
					++p.second.CreateChild();
				if(p.second.CreateChild() == (1 << DimT))
					continue;
				queue.resize(queue.size() + 1); // create new entry
				queue.back().first = &(*p.first)[p.second.CreateChild()];
				InitSerializeInformation(*p.first, p.second,
					p.second.CreateChild(), &queue.back().second);
				do
				{
					++p.second.CreateChild();
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()));
			}
		}
		//����������ȡ���ݣ�������һ���˲����ṹ
		void Serialize(std::istream *in)
		{
			VectorXD< DimT, ScalarType > min;
			min.Zero();
			AACube< VectorXD< DimT, ScalarType > > bcube(min, 1);
			Serialize(bcube, in);
		}

		void Serialize(const AACube< VectorXD< DimT, ScalarType > >
			&bcube, std::istream *in)
		{
			typedef std::pair< CellType *, BuildInformation > Pair;
			BaseType::Clear();
			BaseType::Root() = new CellType();
			std::deque< Pair > stack(1);
			// init build information directly on stack to avoid
			// copying.
			stack.back().first = BaseType::Root();
			InitRootBuildInformation(bcube, &stack.back().second);
			InitRoot(stack.back().second, BaseType::Root());
			while(stack.size())
			{
				Pair &p = stack.back();
				if(p.second.CreateChild() == 1 << DimT)
				{
					LeaveGlobalBuildInformation(*p.first, p.second);
					stack.pop_back();
					continue;
				}
				if(IsLeaf(*p.first))
				{
					if(!ShouldSubdivide(*p.first, p.second))
					{
						stack.pop_back();
						continue;
					}
					// create the children
					const size_t byteCount =
						((1 << DimT) / 8) + (((1 << DimT) % 8)? 1 : 0);
					char b[byteCount];
					in->read(b, byteCount);
					for(size_t i = 0; i < (1 << DimT); ++i)
					{
						const size_t cmpB = 1 << (i - ((i >> 3) << 3));
						if(b[i >> 3] & cmpB)
							p.first->Children()[i] = new CellType();
						else
							p.first->Children()[i] = (CellType *)1;
					}
					Distribute(p.second, p.first);
					if(IsLeaf(*p.first)) // couldn't subdivide?
					{
						stack.pop_back();
						continue;
					}
				}
				else
					LeaveGlobalBuildInformation(*p.first, p.second);
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()))
					++p.second.CreateChild();
				if(p.second.CreateChild() == (1 << DimT))
					continue;
				EnterGlobalBuildInformation(*p.first, &p.second);
				stack.resize(stack.size() + 1); // create new entry
				stack.back().first = &(*p.first)[p.second.CreateChild()];
				InitBuildInformation(*p.first, p.second,
					p.second.CreateChild(), &stack.back().second);
				InitCell(*p.first, p.second, p.second.CreateChild(),
					stack.back().second, &(*p.first)[p.second.CreateChild()]);
				do
				{
					++p.second.CreateChild();
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()));
			}
		}

		void SerializeBreadthFirst(std::istream *in)
		{
			VectorXD< DimT, ScalarType > min;
			min.Zero();
			AACube< VectorXD< DimT, ScalarType > > bcube(min, 1);
			SerializeBreadthFirst(bcube, in);
		}

		void SerializeBreadthFirst(const AACube< VectorXD< DimT, ScalarType > >
			&bcube, std::istream *in)
		{
			typedef std::pair< CellType *, BuildInformation > Pair;
			BaseType::Clear();
			BaseType::Root() = new CellType();
			std::deque< Pair > queue(1);
			// init build information directly on stack to avoid
			// copying.
			queue.back().first = BaseType::Root();
			InitRootBuildInformation(bcube, &queue.back().second);
			InitRoot(queue.back().second, BaseType::Root());
			while(queue.size())
			{
				Pair &p = queue.front();
				if(p.second.CreateChild() == 1 << DimT)
				{
					queue.pop_front();
					continue;
				}
				if(IsLeaf(*p.first))
				{
					if(!ShouldSubdivide(*p.first, p.second))
					{
						queue.pop_front();
						continue;
					}
					// create the children
					const size_t byteCount =
						((1 << DimT) / 8) + (((1 << DimT) % 8)? 1 : 0);
					char b[byteCount];
					in->read(b, byteCount);
					for(size_t i = 0; i < (1 << DimT); ++i)
					{
						const size_t cmpB = 1 << (i - ((i >> 3) << 3));
						if(b[i >> 3] & cmpB)
							p.first->Children()[i] = new CellType();
						else
							p.first->Children()[i] = (CellType *)1;
					}
					Distribute(p.second, p.first);
					if(IsLeaf(*p.first)) // couldn't subdivide?
					{
						queue.pop_front();
						continue;
					}
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()))
					++p.second.CreateChild();
				if(p.second.CreateChild() == (1 << DimT))
					continue;
				queue.resize(queue.size() + 1); // create new entry
				queue.back().first = &(*p.first)[p.second.CreateChild()];
				InitBuildInformation(*p.first, p.second,
					p.second.CreateChild(), &queue.back().second);
				InitCell(*p.first, p.second, p.second.CreateChild(),
					queue.back().second, &(*p.first)[p.second.CreateChild()]);
				do
				{
					++p.second.CreateChild();
				}
				while(p.second.CreateChild() < (1 << DimT) &&
					!ExistChild(*p.first, p.second.CreateChild()));
			}
		}

	protected:
		class BuildInformation
		: public BaseType::BuildInformation
		{
			public:
				unsigned int &CreateChild()
				{ return m_createChild; }
				const unsigned int CreateChild() const
				{ return m_createChild; }
				const AACube< VectorXD< DimT, ScalarType > > &Cube() const
				{ return m_cube; }
				AACube< VectorXD< DimT, ScalarType > > &Cube()
				{ return m_cube; }
				
			private:
				unsigned int m_createChild;
				AACube< VectorXD< DimT, ScalarType > > m_cube;
		};

		class SerializeInformation
		: public BaseType::BuildInformation
		{
			public:
				unsigned int &CreateChild()
				{ return m_createChild; }
				const unsigned int CreateChild() const
				{ return m_createChild; }
				bool Written() const
				{ return m_written; }
				void Written(bool written)
				{ m_written = written; }
				
			private:
				unsigned int m_createChild;
				bool m_written;
		};

		class AxisSplitter
		{
		public:
			AxisSplitter() {}
			AxisSplitter(unsigned int axis, ScalarType value)
			: m_axis(axis)
			, m_value(value)
			{}
			unsigned int &Axis() { return m_axis; }
			ScalarType &Value() { return m_value; }
			template< class VectorT >
			bool operator()(const VectorT &v) const
			{
				return v[m_axis] <= m_value;
			}

		private:
			unsigned int m_axis;
			ScalarType m_value;
		};

		void InitRootBuildInformation(
			const AACube< VectorXD< DimT, ScalarType > > &bcube,
			BuildInformation *bi)
		{
			bi->CreateChild() = 0;
			bi->Cube() = bcube;
			BaseType::InitRootBuildInformation(bi);
		}

		void InitBuildInformation(const CellType &parent,
			const BuildInformation &parentInfo, unsigned int childIdx,
			BuildInformation *bi)
		{
			bi->CreateChild() = 0;
			bi->Cube().Width(parentInfo.Cube().Width() / ScalarType(2));
			for(unsigned int i = 0; i < DimT; ++i)
				if(childIdx & (1 << (DimT - 1 - i)))
					bi->Cube().Min()[i] =
						parentInfo.Cube().Min()[i] + bi->Cube().Width();
				else
					bi->Cube().Min()[i] = parentInfo.Cube().Min()[i];
			BaseType::InitBuildInformation(parent, parentInfo, childIdx,
				bi);
		}

		void InitRootSerializeInformation(SerializeInformation *bi) const
		{
			bi->CreateChild() = 0;
			bi->Written(false);
			BaseType::InitRootBuildInformation(bi);
		}

		void InitSerializeInformation(const CellType &parent,
			const SerializeInformation &parentInfo, unsigned int childIdx,
			SerializeInformation *bi) const
		{
			bi->CreateChild() = 0;
			bi->Written(false);
			BaseType::InitBuildInformation(parent, parentInfo, childIdx,
				bi);
		}

		void Subdivide(BuildInformation &bi, CellType *cell)
		{
			// get the array of splitters
			VectorXD< DimT, ScalarType > center;
			bi.Cube().Center(&center);
			AxisSplitter splitters[DimT];
			for(unsigned int i = 0; i < DimT; ++i)
			{
				splitters[i].Axis() = i;
				splitters[i].Value() = center[i];
			}
			BaseType::SplitData(splitters, DimT, *cell, bi, cell->Children());
		}

		void Distribute(BuildInformation &bi, CellType *cell)
		{
			const size_t byteCount =
				((1 << BaseType::m_dim) / 8) + (((1 << DimT) % 8)? 1 : 0);
			char b[byteCount];
			// create missing cells
			for(unsigned int i = 0; i < (1 << DimT); ++i)
				if(!ExistChild(*cell, i))
				{
					cell->Child(i, new CellType);
					b[i >> 3] |= 1 << (i - ((i >> 3) << 3));
				}
				else
					b[i >> 3] &= ~(1 << (i - ((i >> 3) << 3)));
			// get the array of splitters
			VectorXD< DimT, ScalarType > center;
			bi.Cube().Center(&center);
			AxisSplitter splitters[DimT];
			for(unsigned int i = 0; i < DimT; ++i)
			{
				splitters[i].Axis() = i;
				splitters[i].Value() = center[i];
			}
			SplitData(splitters, DimT, bi, cell->Children());
			for(unsigned int i = 0; i < (1 << DimT); ++i)
			{
				if(cell->Children()[i]->Size() &&
					(b[i >> 3] & (1 << (i - ((i >> 3) << 3)))))
					std::cout << "Bug in distribute!" << std::endl;
				if(!cell->Children()[i]->Size() &&
					(b[i >> 3] & (1 << (i - ((i >> 3) << 3)))))
				{
					delete cell->Children()[i];
					cell->Children()[i] = (CellType *)1;
				}
			}
		}

		template< class PointT, class TraversalInformationT >
		const CellType* NodeContainingPoint(const PointT& point,
			size_t maxLevel, const CellType& cell,
			const TraversalInformationT& ti, CellRange* range) const
		{
			//�Ӱ˲����ĸ��ڵ���Ҷ�ڵ�����
			// �����ǰ�ڵ���Ҷ�ڵ㣬���ظýڵ�
			if (IsLeaf(cell))
			{
				GetCellRange(cell, ti, range);
				return &cell;
			}
			// �����ǰ�ڵ��Ѵﵽ����Σ����ظýڵ�
			if (CellLevel(cell, ti) == maxLevel)
			{
				GetCellRange(cell, ti, range);
				return &cell;
			}
			// find the child containing the point
			//typename BaseType::CellCenterType center;
			//CellCenter(cell, ti, &center);
			// �ҵ������õ���ӽڵ�(����Ϊ0��7)
			size_t childIdx = 0;
			for (unsigned int i = 0; i < DimT; ++i)
			{
				if (point[i] > cell.Center()[i])//center[i])
					childIdx |= 1 << (DimT - i - 1);
			}

			// ����ӽڵ���ڲ��Ҵ�С��С����С�ߴ磬�ݹ�����ӽڵ�
			if (ExistChild(cell, childIdx)
				&& cell[childIdx].Size() >= 3)
			{
				TraversalInformationT cti;
				BaseType::InitTraversalInformation(cell, ti, childIdx, &cti);
				return NodeContainingPoint(point, maxLevel,
					cell[childIdx], cti, range);
			}
			// ���ص�ǰ�ڵ�
			GetCellRange(cell, ti, range);
			return &cell;
		}
	};
};

#endif
