#ifndef PRIMITIVESHAPEVISITOR_HEADER
#define PRIMITIVESHAPEVISITOR_HEADER

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE PlanePrimitiveShape;

class DLL_LINKAGE PrimitiveShapeVisitor
{
public:
	virtual ~PrimitiveShapeVisitor() {}
	virtual void Visit(const PlanePrimitiveShape &plane) = 0;
};

template< class BaseT >
class PrimitiveShapeVisitorShell
: public BaseT
{
public:
	PrimitiveShapeVisitorShell() {}

	template< class T >
	PrimitiveShapeVisitorShell(const T &t)
	: BaseT(t)
	{}

	template< class A, class B >
	PrimitiveShapeVisitorShell(const A &a, const B &b)
	: BaseT(a, b)
	{}

	void Visit(const PlanePrimitiveShape &plane)
	{
		BaseT::Visit(plane);
	}
};

#endif
