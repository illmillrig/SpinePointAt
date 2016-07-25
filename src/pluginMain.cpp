
#include "SpinePointAt.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin( MObject obj ) {
	MStatus   status;
	MFnPlugin plugin( obj, "Travis Miller", "2016", "Any");

	status = plugin.registerNode( "SpinePointAt", SpinePointAt::id, SpinePointAt::creator,
								  SpinePointAt::initialize );
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	return status;
}

MStatus uninitializePlugin( MObject obj) {
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( SpinePointAt::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
