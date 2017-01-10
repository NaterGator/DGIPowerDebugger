
#include <windows.h>
#include <cstdio>
#include <cstdint>
// NOTE: Need to compile with /EHsc due to throw semantics
#include <vector>

#include <sys/timeb.h>
#include <ctime>

using namespace std;

typedef vector<float> FloatVector_t;
typedef vector<double> DoubleVector_t;

FloatVector_t powerData;
DoubleVector_t timeData;

// buffer allocation for auxiliary power, TODO rename this
#define AUTO 0 

#define POWER_CHANNEL_A 0
#define POWER_CHANNEL_B 1

#define POWER_DATA_TYPE_CURRENT 0
#define POWER_DATA_TYPE_VOLTAGE 1
#define POWER_DATA_TYPE_RANGE   2

#define POWER_MODE_CONTINUOUS 0
#define POWER_MODE_ONE_SHOT   1

#define POWER_CALIBRATION_TYPE_A 8
#define POWER_CALIBRATION_TYPE_B 9


// DGILIB Interface Types
#define IF_TIMESTAMP 	0x00
#define IF_SPI			0x20
#define IF_USART		0x21
#define IF_I2C			0x22
#define IF_GPIO			0x30
#define IF_POWER_DATA	0x40
#define IF_POWER_SYNC	0x41
#define IF_DEBUG		0x50
#define IF_PRINT		0x51
#define IF_RESERVED		0xff
char *
decodeInterfaceId(unsigned char id) {
	switch (id) {
		case IF_TIMESTAMP:		return "Timestamp";
		case IF_SPI:			return "SPI";
		case IF_USART:			return "USART";
		case IF_I2C:			return "I2C";
		case IF_GPIO:			return "GPIO";
		case IF_POWER_DATA:		return "Power Data";
		case IF_POWER_SYNC:		return "Power Sync";
		case IF_DEBUG:			return "Debug";
		case IF_PRINT:			return "Print";
		case IF_RESERVED:		return "Reserved";
		default:
			break;
	}
	return "Unknown";
}

// Auxiliary Power Interface Status
#define POWER_AUX_STATUS_IDLE				0x00
#define POWER_AUX_STATUS_RUNNING			0x01
#define POWER_AUX_STATUS_DONE				0x02
#define POWER_AUX_STATUS_CALIBRATING		0x03
#define POWER_AUX_STATUS_INIT_FAILED		0x10
#define POWER_AUX_STATUS_OVERFLOWED			0x11
#define POWER_AUX_STATUS_USB_DISCONNECTED	0x12
#define POWER_AUX_STATUS_CALIBRATION_FAILED	0x20
char *
decodePowerAuxStatus(unsigned status) {
	switch (status) {
		case POWER_AUX_STATUS_IDLE:					return "IDLE";
		case POWER_AUX_STATUS_RUNNING:				return "RUNNING";
		case POWER_AUX_STATUS_DONE:					return "DONE";
		case POWER_AUX_STATUS_CALIBRATING:			return "CALIBRATING";
		case POWER_AUX_STATUS_INIT_FAILED:			return "INIT_FAILED";
		case POWER_AUX_STATUS_OVERFLOWED:			return "OVERFLOWED";
		case POWER_AUX_STATUS_USB_DISCONNECTED:		return "USB_DISCONNECTED";
		case POWER_AUX_STATUS_CALIBRATION_FAILED:	return "CALIBRATION_FAILED";
		default:
			break;
	}
	return "Unknown";
}

// Status callback prototype
typedef void (*DeviceStatusChangedCallBack)(char *, char *, BOOL);
// Discovery
typedef void(*PAM_initialize_status_change_notification)(uint32_t *);
typedef void(*PAM_uninitialize_status_change_notification)(uint32_t);
typedef void(*PAM_register_for_device_status_change_notifications)(uint32_t, DeviceStatusChangedCallBack);
typedef void(*PAM_unregister_for_device_status_change_notifications)(uint32_t, DeviceStatusChangedCallBack);
typedef void(*PAM_discover)(void);
typedef int(*PAM_get_device_count)(void);
typedef int(*PAM_get_device_name)(int, char *);
typedef int(*PAM_get_device_serial)(int, char *);
typedef int(*PAM_is_msd_mode)(char *);
typedef int(*PAM_set_mode)(char *, int);
// Housekeeping
typedef int(*PAM_dgi_connect)(char *, uint32_t *);
typedef int(*PAM_disconnect)(uint32_t);
typedef int(*PAM_connection_status)(uint32_t); // DOCBUG: Guide says this is *
typedef int(*PAM_get_major_version)(void);
typedef int(*PAM_get_minor_version)(void);
typedef int(*PAM_get_build_number)(void);
typedef int(*PAM_get_fw_version)(uint32_t, unsigned char *, unsigned char *);
typedef int(*PAM_start_polling)(uint32_t);
typedef int(*PAM_stop_polling)(uint32_t);
typedef int(*PAM_target_reset)(uint32_t, bool);
// Interface Communication
typedef int(*PAM_interface_list)(uint32_t, unsigned char *, unsigned char *);
typedef int(*PAM_interface_enable)(uint32_t, int, bool);
typedef int(*PAM_interface_disable)(uint32_t, int);
typedef int(*PAM_interface_get_configuration)(uint32_t, int, unsigned *, unsigned *, unsigned *);
typedef int(*PAM_interface_set_configuration)(uint32_t, int, unsigned *, unsigned *, unsigned);
typedef int(*PAM_interface_clear_buffer)(uint32_t, int);
typedef int(*PAM_interface_read_data)(uint32_t, int, unsigned char *, unsigned long long*, int *, unsigned *, unsigned *, unsigned *);
typedef int(*PAM_interface_write_data)(uint32_t, int, unsigned char *, int *);
// Auxiliary
typedef int(*PAM_auxiliary_power_initialize)(uint32_t *, uint32_t);
typedef int(*PAM_auxiliary_power_uninitialize)(uint32_t);
typedef int(*PAM_auxiliary_power_register_buffer_pointers)(uint32_t, float *, double *, size_t *, size_t, int, int);
typedef int(*PAM_auxiliary_power_unregister_buffer_pointers)(uint32_t, int, int);
typedef bool(*PAM_auxiliary_power_calibration_is_valid)(uint32_t);
typedef int(*PAM_auxiliary_power_trigger_calibration)(uint32_t, int);
typedef int(*PAM_auxiliary_power_get_calibration)(uint32_t, uint8_t *, size_t);
typedef int(*PAM_auxiliary_power_get_circuit_type)(uint32_t, int *);
typedef int(*PAM_auxiliary_power_get_status)(uint32_t);
typedef int(*PAM_auxiliary_power_start)(uint32_t, int, int);
typedef int(*PAM_auxiliary_power_stop)(uint32_t);
typedef int(*PAM_auxiliary_power_lock_data_for_reading)(uint32_t);
typedef int(*PAM_auxiliary_power_copy_data)(uint32_t, float *, double *, size_t *, size_t, int, int);
typedef int(*PAM_auxiliary_power_free_data)(uint32_t);

#define PAMDECL(x) PAM_##x x;

// Discovery
PAMDECL(initialize_status_change_notification);
PAMDECL(uninitialize_status_change_notification);
PAMDECL(register_for_device_status_change_notifications);
PAMDECL(unregister_for_device_status_change_notifications);
PAMDECL(discover);
PAMDECL(get_device_count);
PAMDECL(get_device_name);
PAMDECL(get_device_serial);
PAMDECL(is_msd_mode);
PAMDECL(set_mode);
// Housekeeping
PAMDECL(dgi_connect); // DOCBUG conflicts with winsock.h connect()
PAMDECL(disconnect);
PAMDECL(connection_status);
PAMDECL(get_major_version);
PAMDECL(get_minor_version);
PAMDECL(get_build_number);
PAMDECL(get_fw_version);
PAMDECL(start_polling);
PAMDECL(stop_polling);
PAMDECL(target_reset);
// Interface Communication
PAMDECL(interface_list);
PAMDECL(interface_enable);
PAMDECL(interface_disable);
PAMDECL(interface_get_configuration);
PAMDECL(interface_set_configuration);
PAMDECL(interface_clear_buffer);
PAMDECL(interface_read_data);
PAMDECL(interface_write_data);
// Auxilliary
PAMDECL(auxiliary_power_initialize);
PAMDECL(auxiliary_power_uninitialize);
PAMDECL(auxiliary_power_register_buffer_pointers);
PAMDECL(auxiliary_power_unregister_buffer_pointers);
PAMDECL(auxiliary_power_calibration_is_valid);
PAMDECL(auxiliary_power_trigger_calibration);
PAMDECL(auxiliary_power_get_calibration);
PAMDECL(auxiliary_power_get_circuit_type);
PAMDECL(auxiliary_power_get_status);
PAMDECL(auxiliary_power_start);
PAMDECL(auxiliary_power_stop);
PAMDECL(auxiliary_power_lock_data_for_reading);
PAMDECL(auxiliary_power_copy_data);
PAMDECL(auxiliary_power_free_data);

void
deviceChangedCallback(char *deviceName, char *serialNumber, BOOL connected) {
	printf("m: deviceChangedCallback(%s, %s, %d)\n", deviceName, serialNumber, connected);
}

// A visually compact assist for handling DLL mapping & error checking
#define MAPLIB(x) { \
	x = (PAM_##x)GetProcAddress(handle, ""#x""); \
	if (x == NULL) { \
		printf("w: Unable to map DLL function " #x "\n"); \
	}\
}

HMODULE
loadDll(void) {
	HMODULE handle = LoadLibrary("dgilib.dll");
	printf("m: Loading dll\n");
	if (handle == NULL) {
		printf("e: Failed to load the DGI DLL: %d\n", GetLastError());
		return NULL;
	}
	// Discovery
	MAPLIB(initialize_status_change_notification);
	MAPLIB(uninitialize_status_change_notification);
	MAPLIB(register_for_device_status_change_notifications);
	MAPLIB(unregister_for_device_status_change_notifications);
	MAPLIB(discover);
	MAPLIB(get_device_count);
	MAPLIB(get_device_name);
	MAPLIB(get_device_serial);
	MAPLIB(is_msd_mode);
	MAPLIB(set_mode);
	// Housekeeping
	//MAPLIB(dgi_connect);
	dgi_connect = (PAM_dgi_connect)GetProcAddress(handle, "connect");
	MAPLIB(disconnect);
	MAPLIB(connection_status);
	MAPLIB(get_major_version);
	MAPLIB(get_minor_version);
	MAPLIB(get_build_number);
	MAPLIB(get_fw_version);
	MAPLIB(start_polling);
	MAPLIB(stop_polling);
	MAPLIB(target_reset);
	// Interface Communications
	MAPLIB(interface_list);
	MAPLIB(interface_enable);
	MAPLIB(interface_disable);
	MAPLIB(interface_get_configuration);
	MAPLIB(interface_set_configuration);
	MAPLIB(interface_clear_buffer);
	MAPLIB(interface_read_data);
	MAPLIB(interface_write_data);
	// Auxilliary
	MAPLIB(auxiliary_power_initialize);
	MAPLIB(auxiliary_power_uninitialize);
	MAPLIB(auxiliary_power_register_buffer_pointers);
	MAPLIB(auxiliary_power_unregister_buffer_pointers);
	MAPLIB(auxiliary_power_calibration_is_valid);
	MAPLIB(auxiliary_power_trigger_calibration);
	MAPLIB(auxiliary_power_get_calibration);
	MAPLIB(auxiliary_power_get_circuit_type);
	MAPLIB(auxiliary_power_get_status);
	MAPLIB(auxiliary_power_start);
	MAPLIB(auxiliary_power_stop);
	MAPLIB(auxiliary_power_lock_data_for_reading);
	MAPLIB(auxiliary_power_copy_data);
	MAPLIB(auxiliary_power_free_data);
	printf("m: DLL loaded\n");
	return handle;
}

uint32_t
initPam(void) {
	uint32_t hdgi;
	initialize_status_change_notification(&hdgi);
	printf("m: Initialized changed notifications\n");
	register_for_device_status_change_notifications(hdgi, deviceChangedCallback);
	printf("m: Registered device status changed callback\n");
	printf("m: DGI version %d.%d.%d\n",
		get_major_version(), get_minor_version(), get_build_number());
	return hdgi;
}

void
deInitPam(uint32_t hdgi) {
	unregister_for_device_status_change_notifications(hdgi, deviceChangedCallback);
	uninitialize_status_change_notification(hdgi);
	printf("m: Uninitialized PAM\n");
}


typedef void(*DataPollingCallback)(uint32_t);
#define MAX_BUFFER_SIZE (10 * 1024 * 1024) // according to the documents

typedef struct {
	DataPollingCallback cb;
	uint32_t hpower;
} CallbackParams_t;

// how we gracefully exit the polling thread
bool gPoll(false);

DWORD WINAPI
pollingThread(LPVOID lpParam) {
	CallbackParams_t *cbParams;
	cbParams = (CallbackParams_t *)lpParam;
	printf("m: Main client polling thread starting\n");
	while (gPoll) {
		Sleep(100);
		cbParams->cb(cbParams->hpower);
	}
	printf("m: Main client polling thread done\n");
	return 0;
}

// TODO: How do I know if I am overflowing?
void
mainDataCallback(uint32_t hpower) {
	double timestamps[512];
	float current[512];
	float voltage[512];
	size_t count;
	int i(0);
	size_t totalSamples(0);

	if (auxiliary_power_lock_data_for_reading(hpower)) {
		printf("w: Failed to lock power data\n");
	} else {
		do {
			auxiliary_power_copy_data(hpower, current, timestamps, &count, 512, POWER_CHANNEL_A, POWER_DATA_TYPE_CURRENT);
			auxiliary_power_copy_data(hpower, voltage, timestamps, &count, 512, POWER_CHANNEL_A, POWER_DATA_TYPE_VOLTAGE);
			for (int j = 0; j < count; ++j) {
				powerData.push_back(voltage[j] * current[j]);
				timeData.push_back(timestamps[j]);
			}
			totalSamples += count;
		} while (count == 512);
		auxiliary_power_free_data(hpower);
		//printf("m: Stored %d samples\n", totalSamples);
	}
}

int
work(uint32_t hdgi) {

	char buff[256]; // spec says at least 100, yikes
	int numDevs(0);
	unsigned char major(0), minor(0);
	unsigned char interfaces[10];
	unsigned char count(0);
	unsigned config_id[2000];
	unsigned config_value[2000];
	unsigned config_cnt(0);
	uint32_t hpower(0);
	int circuit_type(0);
	int status(0);
	struct _timeb t0, t;
	unsigned testVoltage(2345);
	HANDLE hthread(NULL);
	DWORD threadId(0);

	discover();
	numDevs = get_device_count();
	printf("m: Number of devices: %d\n", numDevs);
	for (int i = 0; i < numDevs; ++i) {
		printf("m: Device name: ");
		if (get_device_name(i, buff)) {
			printf("undefined");
		} else {
			printf("%s", buff);
		}
		printf(", Serial Number: ");
		if (get_device_serial(i, buff)) {
			printf("undefined");
		} else {
			printf("%s", buff);
			/* DOCBUG TODO segfault
			if (is_msd_mode(buff)) {
				printf(" (MSD mode)");
			} else {
				printf(" (non-MSD mode)");
			}
			*/
			/* BUGBUG segfault even though exists
			if (set_mode(buff, 1)) {
				printf(" (set to DGI mode)");
			} else {
				printf(" (error setting DGI mode)");
			}
			*/
		}
		printf("\n");

	}

	// TODO: Assume only ONE device for now, IDX = 1
	if (numDevs != 1) {
		if (numDevs == 0) {
			printf("e: No DGI devices found!\n");
		} else {
			printf("e: Can only support one DGI device, found %d\n", numDevs);
		}
		return 1;
	}

	printf("m: Connecting to device\n");
	// BUG: This sometimes fails for no apparent reason, but
	// BUG: I re-run the .exe and it works fine.
	if (dgi_connect(buff, &hdgi)) {
		printf("e: Error calling DGI connect on %s\n", buff);
		return 1; // from this point on goto work_exit for proper de-init
	} else {
		printf("m: Connected to device %s\n", buff);
	}

	// DOCBUG: Docs are confusing here: indicates open, but non-zero is error, so which is it?
	printf("m: Connection status %s\n", connection_status(hdgi) ? "CLOSED" : "OK");

	if (get_fw_version(hdgi, &major, &minor)) {
		printf("e: Failed to get device firmware version\n");
	} else {
		printf("m: Device firmware %d.%d\n", major, minor);
	}

	// Report the list of interfaces found
	if (interface_list(hdgi, interfaces, &count)) {
		printf("e: Failed to query interface list\n");
	} else {
		for (int i = 0; i < count; ++i) {
			printf("m: Found interface ID 0x%02x : %s\n", interfaces[i],
				decodeInterfaceId(interfaces[i]));
		}
	}

	/*

	// BUGBUG fails to enable with timestamping = true
	if (interface_enable(hdgi, IF_POWER_DATA, false)) {
		printf("e: Failed to enable the power data interface\n");
		goto work_exit;
	} else {
		printf("m: Power data interface enabled without timestamping\n");
	}
	// BUGBUG fails to enable with timestamping = false
	if (interface_enable(hdgi, IF_POWER_SYNC, true)) {
		printf("e: Failed to enable the power sync interface\n");
		goto work_exit;
	} else {
		printf("m: Power sync interface enabled WITH timestamping\n");
	}
	*/

#if 0
	// Power calibration has a lot of bytes! Generic + XAM + PAM
	if (interface_get_configuration(hdgi, IF_POWER_DATA, config_id, config_value, &config_cnt)) {
		printf("e: Failed to get configuration for power interface\n");
	} else {
		printf("m: Found %d configuration paramaters for power interface\n", config_cnt);
		/* TODO this is weird, DGI doc is confusing compared to the return values
		for (int i = 0; i < config_cnt; ++i) {
			printf("m: Configuration ID 0x%08x = 0x%08x\n", config_id[i], config_value[i]);
		}
		*/
	}
#endif

	if (auxiliary_power_initialize(&hpower, hdgi)) {
		printf("e: Failed to initialize aux power interface\n");
		goto work_exit;
	} else {
		printf("m: Auxiliary power interface initialized\n");
	}

	if (auxiliary_power_get_circuit_type(hpower, &circuit_type)) {
		printf("e: Failed to get circuit type\n");
	} else {
		printf("m: Circuit type: ");
		switch (circuit_type) {
			case 0x00: printf("OLD_XAM\n"); break;
			case 0x10: printf("XAM\n"); break;
			case 0x11: printf("PAM\n"); break;
			case 0xff: printf("UNKNOWN\n"); break;
			default:   printf("not found\n"); break;
		}
	}

	printf("m: Aux power calibraiton valid? %s\n",
		auxiliary_power_calibration_is_valid(hpower) ? "yes" : "no");

#if 0
	// DOCBUG: docs aren't clear on this, I assume table 3-16 in the DGI U/G, 3 = PAM-ChA
	// DOCBUG: Nowhere but the PYTHON Code does it say POWER_CALIBRATION_TYPE_A = 8. Probably
	// DOCBUG: because this is new firmware with new calibration like Bob said.
	_ftime(&t0);
	if (status = auxiliary_power_trigger_calibration(hpower, POWER_CALIBRATION_TYPE_A)) {
		printf("e: Failed to trigger calibration on Channel A, status: %s (0x%02x)\n", 
			decodePowerAuxStatus(status), status);
		goto work_exit;
	} else {
		printf("m: Calibrating Channel A...\n");
		status = 0;
		bool timeoutError(false);
		// TODO Need an O/S one-shot timeout after 60sec
		while ((status = auxiliary_power_get_status(hpower)) == POWER_AUX_STATUS_CALIBRATING) {
			_ftime(&t);
			if ((t.time - t0.time) >= 60) {
				timeoutError = true;
				break;
			}
		}
		if (timeoutError) {
			printf("e: Calibration took longer than 60 seconds\n");
			// BUGBUG: Device still has to finish otherwise cannot reconnect
			goto work_exit;
		} else {
			printf("m: Done calibrating, status: %s (0x%02x), time to calibrate: %ds\n", 
				decodePowerAuxStatus(status), status, t.time - t0.time);
		}
	}

	//? 
	Sleep(500);

	_ftime(&t0);
	if (status = auxiliary_power_trigger_calibration(hpower, POWER_CALIBRATION_TYPE_B)) {
		printf("e: Failed to trigger calibration on Channel B, status: %s (0x%02x)\n", 
			decodePowerAuxStatus(status), status);
		goto work_exit;
	} else {
		printf("m: Calibrating Channel B...\n");
		status = 0;
		bool timeoutError(false);
		// TODO Need an O/S one-shot timeout after 60sec
		while ((status = auxiliary_power_get_status(hpower)) == POWER_AUX_STATUS_CALIBRATING) {
			_ftime(&t);
			if ((t.time - t0.time) >= 60) {
				timeoutError = true;
				break;
			}
		}
		if (timeoutError) {
			printf("e: Calibration took longer than 60 seconds\n");
			// BUGBUG: Device still has to finish otherwise cannot reconnect
			goto work_exit;
		} else {
			printf("m: Done calibrating, status: %s (0x%02x), time to calibrate: %ds\n", 
				decodePowerAuxStatus(status), status, t.time - t0.time);
		}
	}

	if (auxiliary_power_calibration_is_valid(hpower)) {
		printf("m: Calibration is valid\n");
	} else {
		printf("e: Invalid calibration\n");
		goto work_exit;
	}

#endif

	config_id[0] = 4; // TODO create #define
	config_value[0] = testVoltage; // millivolts
	// TODO Q How does the hdgi handle know what interface I'm talking to? TODO what handle?
	if (interface_set_configuration(hdgi, IF_POWER_DATA, config_id, config_value, 1)) {
		printf("e: Failed to set voltage to %dmV\n", testVoltage);
	} else {
		printf("m: Channel A voltage set to %dmV\n", testVoltage);
	}

	Sleep(500); // According to sample python script, 500ms is needed for stabilization

	// Register all three buffers
	if (auxiliary_power_register_buffer_pointers(hpower, AUTO, AUTO, AUTO, MAX_BUFFER_SIZE, POWER_CHANNEL_A, POWER_DATA_TYPE_CURRENT)) {
		printf("e: Failed to register current buffer on channel A\n");
		goto work_exit;
	} else {
		printf("m: Current buffer registered on channel A\n");
	}
	if (auxiliary_power_register_buffer_pointers(hpower, AUTO, AUTO, AUTO, MAX_BUFFER_SIZE, POWER_CHANNEL_A, POWER_DATA_TYPE_VOLTAGE)) {
		printf("e: Failed to register voltage buffer on channel A\n");
		goto work_exit;
	} else {
		printf("m: Voltage buffer registered on channel A\n");
	}
	if (auxiliary_power_register_buffer_pointers(hpower, AUTO, AUTO, AUTO, MAX_BUFFER_SIZE, POWER_CHANNEL_A, POWER_DATA_TYPE_RANGE)) {
		printf("e: Failed to register range buffer on channel A\n");
		goto work_exit;
	} else {
		printf("m: Range buffer registered on channel A\n");
	}

	CallbackParams_t cbParams;
	cbParams.cb = mainDataCallback;
	cbParams.hpower = hpower;

	// This is how we tactfully stop the thread
	gPoll = true;

	hthread = CreateThread(
        NULL,                   // default security attributes
        0,                      // use default stack size  
        pollingThread,       // thread function name
        &cbParams,          // argument to thread function 
        0,                      // use default creation flags 
        &threadId);   // returns the thread identifier 

	if (! hthread) {
		printf("e: Failed to creating polling thread\n");
		goto work_exit;
	}

	if (start_polling(hdgi)) {
		printf("w: Failed to start DGI polling\n");
	} else {
		printf("m: DGI polling started\n");
	}
	if (auxiliary_power_start(hpower, POWER_MODE_CONTINUOUS, 0)) {
		printf("e: Failed to start power collection\n");
		if (TerminateThread(hthread, 0)) {
			printf("m: Thread stopped\n");
		} else {
			printf("e: Failed to terminate thread\n");
		}
		goto work_exit;
	} else {
		printf("m: Aux power started\n");
		printf("m: Sampling for 10s\n");
		Sleep(10000);
		// Let the thread finish so that we don't lose data
		gPoll = false;
		if (WaitForSingleObject(hthread, 1000)) {
			printf("w: Thread did not stop, killing\n");
			if (TerminateThread(hthread, 0)) {
				printf("m: Thread stopped\n");
			} else {
				printf("e: Failed to terminate thread\n");
			}
		} else {
			printf("m: Thread exited gracefully\n");
		}
		if (auxiliary_power_stop(hpower)) {
			printf("e: Failed to stop power collection\n");
		} else {
			printf("m: Aux power stopped\n");
		}
		if (stop_polling(hdgi)) {
			printf("w: Failed to stop DGI polling\n");
		} else {
			printf("m: DGI polling stopped\n");
		}
	}

	printf("m: Total samples: %d, sample rate %.0f Hz\n", powerData.size(), powerData.size() / 10.0);

work_exit:
	if (hpower) {
		auxiliary_power_unregister_buffer_pointers(hpower, POWER_CHANNEL_A, POWER_DATA_TYPE_CURRENT);
		auxiliary_power_unregister_buffer_pointers(hpower, POWER_CHANNEL_A, POWER_DATA_TYPE_VOLTAGE);
		auxiliary_power_unregister_buffer_pointers(hpower, POWER_CHANNEL_A, POWER_DATA_TYPE_RANGE);
		if (auxiliary_power_uninitialize(hpower)) {
			printf("e: Failed to unitialize aux power interface\n");
		} else {
			printf("m: Auxiliary power interface uninitialized\n");
		}
	}
	if (hdgi) {
		if (disconnect(hdgi)) {
			printf("e: Failed to disconnect from DGI device, serial # %s\n", buff);
			return 1;
		} else {
			printf("m: Disconnected from DGI device, serial # %s\n", buff);
		}
	}
	return 0;
}

int
main(int argc, char *argv[]) 
{
	HMODULE dll = loadDll();
	uint32_t handle;

	// Initialization
	if (dll) {
		handle = initPam();
		if (! handle) {
			printf("e: Failed to initialize PAM\n");
		} else {
			printf("m: Initialized PAM\n");
		}
	}
	// Do something...
	work(handle);

	FILE *out;
	if (out = fopen("results.txt", "w+")) {
		double dt;
		double J;
		double totalJ(0);
		printf("m: Writing log file\n");
		// terminating the thread can cause samples to truncate
		if (powerData.size() != timeData.size()) {
			printf("w: # power samples %d, # time samples\n", powerData.size(), timeData.size());
		}
		size_t min = (powerData.size() < timeData.size()) ? powerData.size() : timeData.size();
		for (int i = 1; i < min; ++i) {
			dt = timeData[i] - timeData[i - 1];
			J = powerData[i] * dt;
			totalJ += J;
			fprintf(out, "%f %f %f %f\n", timeData[i] * 1e6, powerData[i] * 1e6, J * 1e6, totalJ * 1e6);
		}
		fclose(out);
	} else {
		printf("e: Failed to write log file\n");
	}

	// Cleanup
	if (handle) {
		deInitPam(handle);
	}
	if (! FreeLibrary(dll)) {
		printf("e: failed to unload DLL\n");
	} else {
		printf("m: DLL unloaded\n");
	}
	printf("m: Done\n");
	return 0;
}