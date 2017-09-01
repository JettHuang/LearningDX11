//***************************************************************************************
// MathHelper.cpp by Frank Luna (C) 2011 All Rights Reserved.
//***************************************************************************************

#include "MathHelper.h"
#include <float.h>
#include <cmath>

const float MathHelper::Infinity = FLT_MAX;
const float MathHelper::Pi       = 3.1415926535f;

float MathHelper::AngleFromXY(float x, float y)
{
	float theta = 0.0f;
 
	// Quadrant I or IV
	if(x >= 0.0f) 
	{
		// If x = 0, then atanf(y/x) = +pi/2 if y > 0
		//                atanf(y/x) = -pi/2 if y < 0
		theta = atanf(y / x); // in [-pi/2, +pi/2]

		if(theta < 0.0f)
			theta += 2.0f*Pi; // in [0, 2*pi).
	}

	// Quadrant II or III
	else      
		theta = atanf(y/x) + Pi; // in [0, 2*pi).

	return theta;
}

XMVECTOR MathHelper::RandUnitVec3()
{
	XMVECTOR One  = XMVectorSet(1.0f, 1.0f, 1.0f, 1.0f);
	XMVECTOR Zero = XMVectorZero();

	// Keep trying until we get a point on/in the hemisphere.
	while(true)
	{
		// Generate random point in the cube [-1,1]^3.
		XMVECTOR v = XMVectorSet(MathHelper::RandF(-1.0f, 1.0f), MathHelper::RandF(-1.0f, 1.0f), MathHelper::RandF(-1.0f, 1.0f), 0.0f);

		// Ignore points outside the unit sphere in order to get an even distribution 
		// over the unit sphere.  Otherwise points will clump more on the sphere near 
		// the corners of the cube.

		if( XMVector3Greater( XMVector3LengthSq(v), One) )
			continue;

		return XMVector3Normalize(v);
	}
}

XMVECTOR MathHelper::RandHemisphereUnitVec3(XMVECTOR n)
{
	XMVECTOR One  = XMVectorSet(1.0f, 1.0f, 1.0f, 1.0f);
	XMVECTOR Zero = XMVectorZero();

	// Keep trying until we get a point on/in the hemisphere.
	while(true)
	{
		// Generate random point in the cube [-1,1]^3.
		XMVECTOR v = XMVectorSet(MathHelper::RandF(-1.0f, 1.0f), MathHelper::RandF(-1.0f, 1.0f), MathHelper::RandF(-1.0f, 1.0f), 0.0f);

		// Ignore points outside the unit sphere in order to get an even distribution 
		// over the unit sphere.  Otherwise points will clump more on the sphere near 
		// the corners of the cube.
		
		if( XMVector3Greater( XMVector3LengthSq(v), One) )
			continue;

		// Ignore points in the bottom hemisphere.
		if( XMVector3Less( XMVector3Dot(n, v), Zero ) )
			continue;

		return XMVector3Normalize(v);
	}
}

//////////////////////////////////////////////////////////////////////////

static const XMVECTOR g_UnitQuaternionEpsilon =
{
	1.0e-4f, 1.0e-4f, 1.0e-4f, 1.0e-4f
};
static const XMVECTOR g_UnitVectorEpsilon =
{
	1.0e-4f, 1.0e-4f, 1.0e-4f, 1.0e-4f
};
static const XMVECTOR g_UnitPlaneEpsilon =
{
	1.0e-4f, 1.0e-4f, 1.0e-4f, 1.0e-4f
};


//-----------------------------------------------------------------------------
// Return TRUE if any of the elements of a 3 vector are equal to 0xffffffff.
// Slightly more efficient than using XMVector3EqualInt.
//-----------------------------------------------------------------------------
static inline BOOL XMVector3AnyTrue(FXMVECTOR V)
{
	XMVECTOR C;

	// Duplicate the fourth element from the first element.
	C = XMVectorSwizzle(V, 0, 1, 2, 0);

	return XMComparisonAnyTrue(XMVector4EqualIntR(C, XMVectorTrueInt()));
}



//-----------------------------------------------------------------------------
// Return TRUE if all of the elements of a 3 vector are equal to 0xffffffff.
// Slightly more efficient than using XMVector3EqualInt.
//-----------------------------------------------------------------------------
static inline BOOL XMVector3AllTrue(FXMVECTOR V)
{
	XMVECTOR C;

	// Duplicate the fourth element from the first element.
	C = XMVectorSwizzle(V, 0, 1, 2, 0);

	return XMComparisonAllTrue(XMVector4EqualIntR(C, XMVectorTrueInt()));
}



//-----------------------------------------------------------------------------
// Return TRUE if the vector is a unit vector (length == 1).
//-----------------------------------------------------------------------------
static inline BOOL XMVector3IsUnit(FXMVECTOR V)
{
	XMVECTOR Difference = XMVector3Length(V) - XMVectorSplatOne();

	return XMVector4Less(XMVectorAbs(Difference), g_UnitVectorEpsilon);
}

static inline XMVECTOR XM_CALLCONV XMVectorPermute
(
	FXMVECTOR V1,
	FXMVECTOR V2,
	XMVECTORI32 Permute
)
{
	return XMVectorPermute(V1, V2, Permute.i[0], Permute.i[1], Permute.i[2], Permute.i[3]);
}

//-----------------------------------------------------------------------------
BOOL MathHelper::IntersectTriangleAxisAlignedBox(FXMVECTOR V0, FXMVECTOR V1, FXMVECTOR V2, const BoundingBox* pVolume)
{
	assert(pVolume);

	static CONST XMVECTORI32 Permute0W1Z0Y0X =
	{
		XM_PERMUTE_0W, XM_PERMUTE_1Z, XM_PERMUTE_0Y, XM_PERMUTE_0X
	};
	static CONST XMVECTORI32 Permute0Z0W1X0Y =
	{
		XM_PERMUTE_0Z, XM_PERMUTE_0W, XM_PERMUTE_1X, XM_PERMUTE_0Y
	};
	static CONST XMVECTORI32 Permute1Y0X0W0Z =
	{
		XM_PERMUTE_1Y, XM_PERMUTE_0X, XM_PERMUTE_0W, XM_PERMUTE_0Z
	};

	XMVECTOR Zero = XMVectorZero();

	// Load the box.
	XMVECTOR Center = XMLoadFloat3(&pVolume->Center);
	XMVECTOR Extents = XMLoadFloat3(&pVolume->Extents);

	XMVECTOR BoxMin = Center - Extents;
	XMVECTOR BoxMax = Center + Extents;

	// Test the axes of the box (in effect test the AAB against the minimal AAB 
	// around the triangle).
	XMVECTOR TriMin = XMVectorMin(XMVectorMin(V0, V1), V2);
	XMVECTOR TriMax = XMVectorMax(XMVectorMax(V0, V1), V2);

	// for each i in (x, y, z) if a_min(i) > b_max(i) or b_min(i) > a_max(i) then disjoint
	XMVECTOR Disjoint = XMVectorOrInt(XMVectorGreater(TriMin, BoxMax), XMVectorGreater(BoxMin, TriMax));
	if (XMVector3AnyTrue(Disjoint))
		return FALSE;

	// Test the plane of the triangle.
	XMVECTOR Normal = XMVector3Cross(V1 - V0, V2 - V0);
	XMVECTOR Dist = XMVector3Dot(Normal, V0);

	// Assert that the triangle is not degenerate.
	assert(!XMVector3Equal(Normal, Zero));

	// for each i in (x, y, z) if n(i) >= 0 then v_min(i)=b_min(i), v_max(i)=b_max(i)
	// else v_min(i)=b_max(i), v_max(i)=b_min(i)
	XMVECTOR NormalSelect = XMVectorGreater(Normal, Zero);
	XMVECTOR V_Min = XMVectorSelect(BoxMax, BoxMin, NormalSelect);
	XMVECTOR V_Max = XMVectorSelect(BoxMin, BoxMax, NormalSelect);

	// if n dot v_min + d > 0 || n dot v_max + d < 0 then disjoint
	XMVECTOR MinDist = XMVector3Dot(V_Min, Normal);
	XMVECTOR MaxDist = XMVector3Dot(V_Max, Normal);

	XMVECTOR NoIntersection = XMVectorGreater(MinDist, Dist);
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(MaxDist, Dist));

	// Move the box center to zero to simplify the following tests.
	XMVECTOR TV0 = V0 - Center;
	XMVECTOR TV1 = V1 - Center;
	XMVECTOR TV2 = V2 - Center;

	// Test the edge/edge axes (3*3).
	XMVECTOR e0 = TV1 - TV0;
	XMVECTOR e1 = TV2 - TV1;
	XMVECTOR e2 = TV0 - TV2;

	// Make w zero.
	e0 = XMVectorInsert(e0, Zero, 0, 0, 0, 0, 1);
	e1 = XMVectorInsert(e1, Zero, 0, 0, 0, 0, 1);
	e2 = XMVectorInsert(e2, Zero, 0, 0, 0, 0, 1);

	XMVECTOR Axis;
	XMVECTOR p0, p1, p2;
	XMVECTOR Min, Max;
	XMVECTOR Radius;

	// Axis == (1,0,0) x e0 = (0, -e0.z, e0.y)
	Axis = XMVectorPermute(e0, -e0, Permute0W1Z0Y0X);
	p0 = XMVector3Dot(TV0, Axis);
	// p1 = XMVector3Dot( V1, Axis ); // p1 = p0;
	p2 = XMVector3Dot(TV2, Axis);
	Min = XMVectorMin(p0, p2);
	Max = XMVectorMax(p0, p2);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (1,0,0) x e1 = (0, -e1.z, e1.y)
	Axis = XMVectorPermute(e1, -e1, Permute0W1Z0Y0X);
	p0 = XMVector3Dot(TV0, Axis);
	p1 = XMVector3Dot(TV1, Axis);
	// p2 = XMVector3Dot( V2, Axis ); // p2 = p1;
	Min = XMVectorMin(p0, p1);
	Max = XMVectorMax(p0, p1);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (1,0,0) x e2 = (0, -e2.z, e2.y)
	Axis = XMVectorPermute(e2, -e2, Permute0W1Z0Y0X);
	p0 = XMVector3Dot(TV0, Axis);
	p1 = XMVector3Dot(TV1, Axis);
	// p2 = XMVector3Dot( V2, Axis ); // p2 = p0;
	Min = XMVectorMin(p0, p1);
	Max = XMVectorMax(p0, p1);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (0,1,0) x e0 = (e0.z, 0, -e0.x)
	Axis = XMVectorPermute(e0, -e0, Permute0Z0W1X0Y);
	p0 = XMVector3Dot(TV0, Axis);
	// p1 = XMVector3Dot( V1, Axis ); // p1 = p0;
	p2 = XMVector3Dot(TV2, Axis);
	Min = XMVectorMin(p0, p2);
	Max = XMVectorMax(p0, p2);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (0,1,0) x e1 = (e1.z, 0, -e1.x)
	Axis = XMVectorPermute(e1, -e1, Permute0Z0W1X0Y);
	p0 = XMVector3Dot(TV0, Axis);
	p1 = XMVector3Dot(TV1, Axis);
	// p2 = XMVector3Dot( V2, Axis ); // p2 = p1;
	Min = XMVectorMin(p0, p1);
	Max = XMVectorMax(p0, p1);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (0,0,1) x e2 = (e2.z, 0, -e2.x)
	Axis = XMVectorPermute(e2, -e2, Permute0Z0W1X0Y);
	p0 = XMVector3Dot(TV0, Axis);
	p1 = XMVector3Dot(TV1, Axis);
	// p2 = XMVector3Dot( V2, Axis ); // p2 = p0;
	Min = XMVectorMin(p0, p1);
	Max = XMVectorMax(p0, p1);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (0,0,1) x e0 = (-e0.y, e0.x, 0)
	Axis = XMVectorPermute(e0, -e0, Permute1Y0X0W0Z);
	p0 = XMVector3Dot(TV0, Axis);
	// p1 = XMVector3Dot( V1, Axis ); // p1 = p0;
	p2 = XMVector3Dot(TV2, Axis);
	Min = XMVectorMin(p0, p2);
	Max = XMVectorMax(p0, p2);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (0,0,1) x e1 = (-e1.y, e1.x, 0)
	Axis = XMVectorPermute(e1, -e1, Permute1Y0X0W0Z);
	p0 = XMVector3Dot(TV0, Axis);
	p1 = XMVector3Dot(TV1, Axis);
	// p2 = XMVector3Dot( V2, Axis ); // p2 = p1;
	Min = XMVectorMin(p0, p1);
	Max = XMVectorMax(p0, p1);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	// Axis == (0,0,1) x e2 = (-e2.y, e2.x, 0)
	Axis = XMVectorPermute(e2, -e2, Permute1Y0X0W0Z);
	p0 = XMVector3Dot(TV0, Axis);
	p1 = XMVector3Dot(TV1, Axis);
	// p2 = XMVector3Dot( V2, Axis ); // p2 = p0;
	Min = XMVectorMin(p0, p1);
	Max = XMVectorMax(p0, p1);
	Radius = XMVector3Dot(Extents, XMVectorAbs(Axis));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(Min, Radius));
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(Max, -Radius));

	return XMVector4NotEqualInt(NoIntersection, XMVectorTrueInt());
}

//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with an axis aligned 
// box using the slabs method.
//-----------------------------------------------------------------------------
BOOL MathHelper::IntersectRayAxisAlignedBox(FXMVECTOR Origin, FXMVECTOR Direction, const BoundingBox* pVolume, FLOAT* pDist)
{
	assert(pVolume);
	assert(pDist);
	assert(XMVector3IsUnit(Direction));

	static const XMVECTOR Epsilon =
	{
		1e-20f, 1e-20f, 1e-20f, 1e-20f
	};
	static const XMVECTOR FltMin =
	{
		-FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX
	};
	static const XMVECTOR FltMax =
	{
		FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX
	};

	// Load the box.
	XMVECTOR Center = XMLoadFloat3(&pVolume->Center);
	XMVECTOR Extents = XMLoadFloat3(&pVolume->Extents);

	// Adjust ray origin to be relative to center of the box.
	XMVECTOR TOrigin = Center - Origin;

	// Compute the dot product againt each axis of the box.
	// Since the axii are (1,0,0), (0,1,0), (0,0,1) no computation is necessary.
	XMVECTOR AxisDotOrigin = TOrigin;
	XMVECTOR AxisDotDirection = Direction;

	// if (fabs(AxisDotDirection) <= Epsilon) the ray is nearly parallel to the slab.
	XMVECTOR IsParallel = XMVectorLessOrEqual(XMVectorAbs(AxisDotDirection), Epsilon);

	// Test against all three axii simultaneously.
	XMVECTOR InverseAxisDotDirection = XMVectorReciprocal(AxisDotDirection);
	XMVECTOR t1 = (AxisDotOrigin - Extents) * InverseAxisDotDirection;
	XMVECTOR t2 = (AxisDotOrigin + Extents) * InverseAxisDotDirection;

	// Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
	// use the results from any directions parallel to the slab.
	XMVECTOR t_min = XMVectorSelect(XMVectorMin(t1, t2), FltMin, IsParallel);
	XMVECTOR t_max = XMVectorSelect(XMVectorMax(t1, t2), FltMax, IsParallel);

	// t_min.x = maximum( t_min.x, t_min.y, t_min.z );
	// t_max.x = minimum( t_max.x, t_max.y, t_max.z );
	t_min = XMVectorMax(t_min, XMVectorSplatY(t_min));  // x = max(x,y)
	t_min = XMVectorMax(t_min, XMVectorSplatZ(t_min));  // x = max(max(x,y),z)
	t_max = XMVectorMin(t_max, XMVectorSplatY(t_max));  // x = min(x,y)
	t_max = XMVectorMin(t_max, XMVectorSplatZ(t_max));  // x = min(min(x,y),z)

														// if ( t_min > t_max ) return FALSE;
	XMVECTOR NoIntersection = XMVectorGreater(XMVectorSplatX(t_min), XMVectorSplatX(t_max));

	// if ( t_max < 0.0f ) return FALSE;
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(XMVectorSplatX(t_max), XMVectorZero()));

	// if (IsParallel && (-Extents > AxisDotOrigin || Extents < AxisDotOrigin)) return FALSE;
	XMVECTOR ParallelOverlap = XMVectorInBounds(AxisDotOrigin, Extents);
	NoIntersection = XMVectorOrInt(NoIntersection, XMVectorAndCInt(IsParallel, ParallelOverlap));

	if (!XMVector3AnyTrue(NoIntersection))
	{
		// Store the x-component to *pDist
		XMStoreFloat(pDist, t_min);
		return TRUE;
	}

	return FALSE;
}

//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with a triangle 
// (V0, V1, V2).  Return TRUE if there is an intersection and also set *pDist 
// to the distance along the ray to the intersection.
// 
// The algorithm is based on Moller, Tomas and Trumbore, "Fast, Minimum Storage 
// Ray-Triangle Intersection", Journal of Graphics Tools, vol. 2, no. 1, 
// pp 21-28, 1997.
//-----------------------------------------------------------------------------
BOOL MathHelper::IntersectRayTriangle(FXMVECTOR Origin, FXMVECTOR Direction, FXMVECTOR V0, CXMVECTOR V1, CXMVECTOR V2, FLOAT* pDist)
{
	assert(pDist);
	assert(XMVector3IsUnit(Direction));

	static const XMVECTOR Epsilon =
	{
		1e-20f, 1e-20f, 1e-20f, 1e-20f
	};

	XMVECTOR Zero = XMVectorZero();

	XMVECTOR e1 = V1 - V0;
	XMVECTOR e2 = V2 - V0;

	// p = Direction ^ e2;
	XMVECTOR p = XMVector3Cross(Direction, e2);

	// det = e1 * p;
	XMVECTOR det = XMVector3Dot(e1, p);

	XMVECTOR u, v, t;

	if (XMVector3GreaterOrEqual(det, Epsilon))
	{
		// Determinate is positive (front side of the triangle).
		XMVECTOR s = Origin - V0;

		// u = s * p;
		u = XMVector3Dot(s, p);

		XMVECTOR NoIntersection = XMVectorLess(u, Zero);
		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(u, det));

		// q = s ^ e1;
		XMVECTOR q = XMVector3Cross(s, e1);

		// v = Direction * q;
		v = XMVector3Dot(Direction, q);

		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(v, Zero));
		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(u + v, det));

		// t = e2 * q;
		t = XMVector3Dot(e2, q);

		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(t, Zero));

		if (XMVector4EqualInt(NoIntersection, XMVectorTrueInt()))
			return FALSE;
	}
	else if (XMVector3LessOrEqual(det, -Epsilon))
	{
		// Determinate is negative (back side of the triangle).
		XMVECTOR s = Origin - V0;

		// u = s * p;
		u = XMVector3Dot(s, p);

		XMVECTOR NoIntersection = XMVectorGreater(u, Zero);
		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(u, det));

		// q = s ^ e1;
		XMVECTOR q = XMVector3Cross(s, e1);

		// v = Direction * q;
		v = XMVector3Dot(Direction, q);

		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(v, Zero));
		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorLess(u + v, det));

		// t = e2 * q;
		t = XMVector3Dot(e2, q);

		NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(t, Zero));

		if (XMVector4EqualInt(NoIntersection, XMVectorTrueInt()))
			return FALSE;
	}
	else
	{
		// Parallel ray.
		return FALSE;
	}

	XMVECTOR inv_det = XMVectorReciprocal(det);

	t *= inv_det;

	// u * inv_det and v * inv_det are the barycentric cooridinates of the intersection.

	// Store the x-component to *pDist
	XMStoreFloat(pDist, t);

	return TRUE;
}

