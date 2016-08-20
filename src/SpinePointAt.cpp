
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
	return (a > b) ? a : b;
}

inline double min(double a, double b) {
	return (a < b) ? a : b;
}


inline void F3DLimit(double &d, const double dMin, const double dMax) {
    d = max(d, dMin);
    d = min(d, dMax);
}



inline void quatFromMatrix(const MMatrix &tfm, MQuaternion &quat) {
	quat.w = std::sqrt(max(0.0, 1.0 + tfm[0][0] + tfm[1][1] + tfm[2][2])) / 2.0;
	quat.x = std::sqrt(max(0.0, 1.0 + tfm[0][0] - tfm[1][1] - tfm[2][2])) / 2.0;
	quat.y = std::sqrt(max(0.0, 1.0 - tfm[0][0] + tfm[1][1] - tfm[2][2])) / 2.0;
	quat.z = std::sqrt(max(0.0, 1.0 - tfm[0][0] - tfm[1][1] + tfm[2][2])) / 2.0;
	quat.x = std::copysign(quat.x, tfm[1][2] - tfm[2][1]);
	quat.y = std::copysign(quat.y, tfm[2][0] - tfm[0][2]);
	quat.z = std::copysign(quat.z, tfm[0][1] - tfm[1][0]);
}


inline void calculateRotation(const MMatrix &a, MQuaternion &q) {
  double trace = a[0][0] + a[1][1] + a[2][2];
  if( trace > 0 ) {
    double s = 0.5 / sqrt(trace + 1.0);
    q.w = 0.25f / s;
    q.x = ( a[2][1] - a[1][2] ) * s;
    q.y = ( a[0][2] - a[2][0] ) * s;
    q.z = ( a[1][0] - a[0][1] ) * s;
  } else {
    if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
      double s = 2.0 * sqrt( 1.0 + a[0][0] - a[1][1] - a[2][2]);
      q.w = (a[2][1] - a[1][2] ) / s;
      q.x = 0.25f * s;
      q.y = (a[0][1] + a[1][0] ) / s;
      q.z = (a[0][2] + a[2][0] ) / s;
    } else if (a[1][1] > a[2][2]) {
      double s = 2.0 * sqrt( 1.0 + a[1][1] - a[0][0] - a[2][2]);
      q.w = (a[0][2] - a[2][0] ) / s;
      q.x = (a[0][1] + a[1][0] ) / s;
      q.y = 0.25f * s;
      q.z = (a[1][2] + a[2][1] ) / s;
    } else {
      double s = 2.0f * sqrt( 1.0 + a[2][2] - a[0][0] - a[1][1] );
      q.w = (a[1][0] - a[0][1] ) / s;
      q.x = (a[0][2] + a[2][0] ) / s;
      q.y = (a[1][2] + a[2][1] ) / s;
      q.z = 0.25f * s;
    }
  }
}



MTypeId SpinePointAt::id(0x001226CF);
MObject SpinePointAt::inARotX;
MObject SpinePointAt::inARotY;
MObject SpinePointAt::inARotZ;
MObject SpinePointAt::inARot;
MObject SpinePointAt::inBRotX;
MObject SpinePointAt::inBRotY;
MObject SpinePointAt::inBRotZ;
MObject SpinePointAt::inBRot;
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

    inARotX = fnUnit.create("inARotX", "iarx", MFnUnitAttribute::kAngle, 0.0);
    inARotY = fnUnit.create("inARotY", "iary", MFnUnitAttribute::kAngle, 0.0);
    inARotZ = fnUnit.create("inARotZ", "iarz", MFnUnitAttribute::kAngle, 0.0);
    inARot = fnNum.create("inARot", "iar", SpinePointAt::inARotX, SpinePointAt::inARotY, SpinePointAt::inARotZ, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
    fnNum.setKeyable(true);
    fnNum.setStorable(true);
    stat = SpinePointAt::addAttribute(inARot);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    inBRotX = fnUnit.create("inBRotX", "ibrx", MFnUnitAttribute::kAngle, 0.0);
    inBRotY = fnUnit.create("inBRotY", "ibry", MFnUnitAttribute::kAngle, 0.0);
    inBRotZ = fnUnit.create("inBRotZ", "ibrz", MFnUnitAttribute::kAngle, 0.0);
    inBRot = fnNum.create("inBRot", "ibr", SpinePointAt::inBRotX, SpinePointAt::inBRotY, SpinePointAt::inBRotZ, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
    fnNum.setKeyable(true);
    fnNum.setStorable(true);
    stat = SpinePointAt::addAttribute(inBRot);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

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


    SpinePointAt::attributeAffects(inARot, pointAt);
	SpinePointAt::attributeAffects(inBRot, pointAt);
	SpinePointAt::attributeAffects(axis, pointAt);
	SpinePointAt::attributeAffects(blend, pointAt);

	return MS::kSuccess;
}


MStatus SpinePointAt::compute( const MPlug& plug, MDataBlock& data ) {

	if (plug != pointAt)
		return MS::kUnknownParameter;

	const short axis = data.inputValue(SpinePointAt::axis).asShort();
	const double blend = data.inputValue(SpinePointAt::blend).asDouble();

    const MEulerRotation rotA = data.inputValue(SpinePointAt::inARot).asVector();
    const MQuaternion quatA = rotA.asQuaternion();

    const MEulerRotation rotB = data.inputValue(SpinePointAt::inBRot).asVector();
    const MQuaternion quatB = rotB.asQuaternion();

	const double dot = this->quatDot(quatA, quatB);
    double angle, factor;
    this->angleAndFactor(dot, angle, factor);

	double factorA, factorB;
	this->scaleAngleAndFactor(dot, blend, angle, factor, factorA, factorB);
	MQuaternion quatC = this->scaleQuat(quatA, quatB, factorA, factorB);

	MVector vOut (0.0, 0.0, 0.0);
	if (axis == 0)
		vOut.x = 1.0;

	else if (axis == 1)
		vOut.y = 1.0;

	else if (axis == 2)
		vOut.z = 1.0;

	else if (axis == 3)
		vOut.x = -1.0;

	else if (axis == 4)
		vOut.y = -1.0;

	else if (axis == 5)
		vOut.z = -1.0;

	vOut *= quatC;

	data.outputValue(SpinePointAt::pointAt).setMVector(vOut);

	data.setClean(plug);
	return MS::kSuccess;
}


inline double SpinePointAt::quatDot(const MQuaternion &quatA, const MQuaternion &quatB) const {
	return (quatA.x * quatB.x) +
		   (quatA.y * quatB.y) +
		   (quatA.z * quatB.z) +
		   (quatA.w * quatB.w);
}


void SpinePointAt::angleAndFactor(const double &in, double &angle, double &factor) const {

	const double ac = acos(in);

	if ( (in * -in) + 1.0 == 0.0 )
		angle = (-1.0);
	else
		angle = ac;


	const double as = sin(ac);

	if (as != 0.0)
		factor = 1.0 / as;
	else
		factor = 0.0;
}


void SpinePointAt::scaleAngleAndFactor(const double &dot, const double &blend,
									   const double &angle, const double &inFactor,
									   double &outFactorA, double &outFactorB) const {
	if (dot >= (1 - 1.0e-12)){
		outFactorA = 1.0 - blend;
		outFactorB = blend;
		return;
	}

	outFactorA = sin(angle * (1.0 - blend)) * inFactor;
	outFactorB = sin(angle * blend) * inFactor;


}


inline MQuaternion SpinePointAt::scaleQuat(const MQuaternion &quatA, const MQuaternion &quatB,
										   const double &factorA, const double &factorB) const {
	return (factorA * quatA) + (factorB * quatB);
}




MQuaternion SpinePointAt::fullSlerp(const MQuaternion &quatA, const MQuaternion &quatB, const double &blend) {

    double dot = quatDot(quatA, quatB);

    double scaleA, scaleB;

    if (dot >= 1.0 - 1.0e-12) {
        scaleA = 1.0 - blend;
        scaleB = blend;
    }


    double angle;

    if (round((-dot * dot+1) / 0.00001) * 0.00001  == 0)
        return quatA;
    else
        angle = acos(dot);



    double factor;

    if (round(sin(angle) / 0.000001) * 0.000001 !=  0)
        factor = 1.0 / sin(angle);
    else
        return quatA;


    scaleA = sin( (1.0 - blend) * angle ) * factor;
    scaleB = sin( blend * angle ) * factor;

    return (scaleA * quatA) + (scaleB * quatB);
}
