#pragma once

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MTypeId.h>
#include <maya/MQuaternion.h>



class SpinePointAt : public MPxNode {
public:
	SpinePointAt();
	virtual	~SpinePointAt();
	virtual MPxNode::SchedulingType schedulingType() const override { return MPxNode::kParallel; }
	static void *creator();
	static MStatus initialize();
	virtual MStatus	compute(const MPlug& plug, MDataBlock& data);

	double quatDot(const MQuaternion &quatA, const MQuaternion &quatB);
	void angleAndFactor(const double &inValue, double &angle, double &factor);

	void scaleAngleAndFactor(const double &dot, const double &blend, const double &angle, const double &inFactor,
							 double &outFactorA, double &outFactorB);

	MQuaternion scaleQuat(const MQuaternion &quatA, const MQuaternion &quatB, const double &factorA,
						  const double &factorB);
public:
	static MTypeId id;
	static MObject tfmA;
	static MObject tfmB;
	static MObject parentInverse;
	static MObject axis;
	static MObject blend;
	static MObject pointAtX;
	static MObject pointAtY;
	static MObject pointAtZ;
	static MObject pointAt;
};

