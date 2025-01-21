#ifndef __ENDPOINTS_H__
#define __ENDPOINTS_H__

#define CSG_UDS_PATH "/var/tmp/can-sock-gw/"

#define UDS_ORCHESTRATOR_ENDPOINT			(CSG_UDS_PATH "orchestrator")
#define UDS_ORCHESTRATOR_CLIENT_ENDPOINT	(CSG_UDS_PATH "orchestrator.client")
#define UDS_REGULAR_TELEMETRY_ENDPOINT		(CSG_UDS_PATH "can.telemetry.regular")
#define UDS_TARGET_PAYLOAD_ENDPOINT			(CSG_UDS_PATH "can.target_payload")
#define UDS_LOG_TELEMETRY_ENDPOINT			(CSG_UDS_PATH "can.telemetry.log")
#define UDS_FINE_ANGLES_ENDPOINT			(CSG_UDS_PATH "fine_guidance.angles")
#define UDS_MAIN_TELEMETRY_ENDPOINT			(CSG_UDS_PATH "can.telemetry.main")
#define UDS_EXTRA_TELEMETRY_ENDPOINT		(CSG_UDS_PATH "can.telemetry.extra")

#define UDS_FINE_ANGLES_CLIENT_ENDPOINT		(CSG_UDS_PATH "fine_guidance.angles.client")

#endif  // __ENDPOINTS_H__
