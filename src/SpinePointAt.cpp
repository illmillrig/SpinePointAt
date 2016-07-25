
#include "SpinePointAt.h"

#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MEulerRotation.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MGlobal.h>

#include <cmath>



inline double max(double a, double b) {
	return (a < b) ? b : a;
}


inline MQuaternion quatFromMatrix(const MMatrix &tfm) {
	double x, y, z, w;
	w = std::sqrt(max(0.0, 1.0 + tfm[0][0] + tfm[1][1] + tfm[2][2])) / 2.0;
	x = std::sqrt(max(0.0, 1.0 + tfm[0][0] - tfm[1][1] - tfm[2][2])) / 2.0;
	y = std::sqrt(max(0.0, 1.0 - tfm[0][0] + tfm[1][1] - tfm[2][2])) / 2.0;
	z = std::sqrt(max(0.0, 1.0 - tfm[0][0] - tfm[1][1] + tfm[2][2])) / 2.0;
	x = std::copysign(x, tfm[1][2] - tfm[2][1]);
	y = std::copysign(y, tfm[2][0] - tfm[0][2]);
	z = std::copysign(z, tfm[0][1] - tfm[1][0]);

	return MQuaternion(x,y,z,w);
}


MTypeId SpinePointAt::id(0x001226CF);
MObject SpinePointAt::tfmA;
MObject SpinePointAt::tfmB;
MObject SpinePointAt::parentInverse;
MObject SpinePointAt::axis;
MObject SpinePointAt::blend;
MObject SpinePointAt::pointAtX;
MObject SpinePointAt::pointAtY;
MObject SpinePointAt::pointAtZ;
MObject SpinePointAt::pointAt;

SpinePointAt::SpinePointAt() {}

SpinePointAt::~SpinePointAt() {}

void* SpinePointAt::creator() {
	return new SpinePointAt();
}


MStatus SpinePointAt::initialize() {
	// attributes are writable by default
	// attributes are storable by default
	// attributes are readable by default
	// attributes not keyable by default

	MStatus stat;
	MFnNumericAttribute fnNum;
	MFnMatrixAttribute fnMat;
	MFnUnitAttribute fnUnit;
	MFnEnumAttribute fnEnum;

	tfmA = fnMat.create("tfmA", "tfma", MFnMatrixAttribute::kDouble, &stat);
	CHECK_MSTATUS(stat);
	fnMat.setKeyable(true);
	stat = SpinePointAt::addAttribute(tfmA);
	CHECK_MSTATUS(stat);

	tfmB = fnMat.create("tfmB", "tfmb", MFnMatrixAttribute::kDouble, &stat);
	CHECK_MSTATUS(stat);
	fnMat.setKeyable(true);
	stat = SpinePointAt::addAttribute(tfmB);
	CHECK_MSTATUS(stat);

	axis = fnEnum.create("axis", "a", 2);
	fnEnum.addField("X", 0);
	fnEnum.addField("Y", 1);
	fnEnum.addField("Z", 2);
	fnEnum.addField("-X", 3);
	fnEnum.addField("-Y", 4);
	fnEnum.addField("-Z", 5);
	fnEnum.setKeyable(true);
	stat = SpinePointAt::addAttribute(axis);
	CHECK_MSTATUS(stat);

	blend = fnNum.create("blend", "b", MFnNumericData::kDouble, 0.5, &stat);
	CHECK_MSTATUS(stat);
	fnNum.setKeyable(true);
	stat = SpinePointAt::addAttribute(blend);
	CHECK_MSTATUS(stat);

	pointAtX = fnNum.create("pointAtX", "pax", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS(stat);
	fnNum.setWritable(false);
	fnNum.setStorable(false);

	pointAtY = fnNum.create("pointAtY", "pay", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS(stat);
	fnNum.setWritable(false);
	fnNum.setStorable(false);

	pointAtZ = fnNum.create("pointAtZ", "paz", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS(stat);
	fnNum.setWritable(false);
	fnNum.setStorable(false);

	pointAt = fnNum.create("pointAt", "pa", pointAtX, pointAtY, pointAtZ, &stat);
	CHECK_MSTATUS(stat);
	fnNum.setWritable(false);
	fnNum.setStorable(false);
	SpinePointAt::addAttribute(pointAt);

	SpinePointAt::attributeAffects(tfmA, pointAt);
	SpinePointAt::attributeAffects(tfmB, pointAt);
	SpinePointAt::attributeAffects(parentInverse, pointAt);
	SpinePointAt::attributeAffects(axis, pointAt);
	SpinePointAt::attributeAffects(blend, pointAt);

	return MS::kSuccess;
}


MStatus SpinePointAt::compute( const MPlug& plug, MDataBlock& data ) {

	if (plug != pointAt)
		return MS::kUnknownParameter;

	const short axis = data.inputValue(SpinePointAt::axis).asShort();
	const double blend = data.inputValue(SpinePointAt::blend).asDouble();

	MMatrix tfmA = data.inputValue(SpinePointAt::tfmA).asMatrix();
	MQuaternion quatA = quatFromMatrix(tfmA);

	MMatrix tfmB = data.inputValue(SpinePointAt::tfmB).asMatrix();
	MQuaternion quatB = quatFromMatrix(tfmB);

	double dot = this->quatDot(quatA, quatB);
	double angle, factor;
	this->angleAndFactor(dot, angle, factor);

	double factorA, factorB;
	this->scaleAngleAndFactor(dot, blend, angle, factor, factorA, factorB);
	MQuaternion quatC = this->scaleQuat(quatA, quatB, factorA, factorB);

	MVector vOut;
	if (axis == 0)
		vOut = MVector(1, 0, 0);
	else if (axis == 1)
		vOut = MVector(0, 1, 0);
	else if (axis == 2)
		vOut = MVector(0, 0, 1);
	else if (axis == 3)
		vOut = MVector(-1, 0, 0);
	else if (axis == 4)
		vOut = MVector(0, -1, 0);
	else if (axis == 5)
		vOut = MVector(0, 0, -1);

	vOut = vOut.rotateBy(quatC);

	data.outputValue(SpinePointAt::pointAt).setMVector(vOut);
	data.setClean(plug);

	return MS::kSuccess;
}


double SpinePointAt::quatDot(const MQuaternion &quatA, const MQuaternion &quatB) {
	return (quatA.x * quatB.x) + (quatA.y * quatB.y) + (quatA.z * quatB.z) + (quatA.w * quatB.w);
}


void SpinePointAt::angleAndFactor(const double &inValue, double &angle, double &factor) {
	double a = inValue;
	double ac = acos(a);
	double as = sin(ac);

	if ( (a * (-a)) + 1.0 == 0.0 )
		angle = (-1);
	else
		angle = ac;


	if (as != 0.0)
		factor = 1.0 / as;
	else
		factor = 0.0;
}


void SpinePointAt::scaleAngleAndFactor(const double &dot, const double &blend,
									   const double &angle, const double &inFactor,
									   double &outFactorA, double &outFactorB) {
	if (dot >= 1.0){
		outFactorA = (1.0 - blend);
		outFactorB = blend;
		return;
	}
	outFactorA = sin(angle * (1.0 - blend)) * inFactor;
	outFactorB = sin(angle * blend) * inFactor;
}


MQuaternion SpinePointAt::scaleQuat(const MQuaternion &quatA, const MQuaternion &quatB,
									const double &factorA, const double &factorB) {
	return (factorA * quatA) + (factorB * quatB);
}


