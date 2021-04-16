#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>

#include "common/getopt.h"
#include "lcmtypes/nmea_t.h"
#include "lcmtypes/ubx_t.h"
#include "lcm/lcm.h"

#include "ublox.h"

static lcm_t *lcm;
static uint8_t verbose = 0;
static int32_t nMsgs = 0;

// TODO: The packet processing code depends on the platform being little-
// endian, as it does no byte order conversion
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "This code expects a little-endian platform"
#endif

typedef struct {
    uint16_t pending[6];
    uint8_t usage[6];
    uint8_t peakUsage[6];
    uint8_t tUsage;
    uint8_t tPeakUsage;
    uint8_t errors;
    uint8_t reserved;
} ubx_mon_txbuf_t;

void ubx_callback(int64_t time, ublox_packet_t *packet)
{
    nMsgs += 1;

    // Handle MON-TXBUF status messages
    if (packet->class == 0x0A && packet->id == 0x08) {
	// XXX This depends on the platform being little-endian
        ubx_mon_txbuf_t *data = (ubx_mon_txbuf_t*)(packet->payload);
        // Port index 3 is USB
        printf("[%11lld.%06ld] ublox  msgs: %-7d ",
               (long long)(time/1000000), (long)(time%1000000), nMsgs);
        printf("txbuf: %-4d usage: %3d%%  errors: %x\n",
               data->pending[3], data->usage[3], data->errors);
    }
    // Print NAK messages
    else if (packet->class == 0x05 && packet->id == 0x00) {
        printf("Warning: NAK received for CFG message %02x %02x\n",
               packet->payload[0], packet->payload[1]);
    } else {
	// Print unknown messages
        if (verbose)
            ublox_packet_print(packet);

        // Publish everything
        ubx_t raw;
        raw.utime = time;
        raw.cls = packet->class;
        raw.id = packet->id;
        raw.len = packet->length;
        raw.data = packet->payload;
        ubx_t_publish(lcm, "UBX", &raw);
    }
}

void nmea_callback(int64_t time, const char *msg)
{
    nMsgs += 1;

    nmea_t nm;
    nm.utime = time;
    nm.nmea = (char*)msg;

    char d[1024];
    int dlen = nmea_t_encode(d, 0, 1000, &nm);
    lcm_publish(lcm, "NMEA", d, dlen);

    if (verbose)
        printf("%s\n", msg);
}

// Set measurement rate (ublox 6T supports up to 5Hz, or 200ms)
void set_measurement_rate(ublox_t *ub, uint16_t rate_ms)
{
    struct {
        uint16_t measRate;
        uint16_t navRate;
        uint16_t timeRef;
    } s;
    s.measRate = rate_ms;
    s.navRate = 1;
    s.timeRef = 1;
    ublox_command(ub, 0x06, 0x08, 6, (uint8_t*)&s);
}

// Rate is relative to the measurement rate;
// 1 = one message every measurement, 2 = one message every 2 measurements
void set_msg_rate(ublox_t *ub, uint8_t class, uint8_t id, uint8_t rate)
{
    uint8_t data[] = {class, id, rate};
    ublox_command(ub, 0x06, 0x01, 3, data);
}

void set_nav_engine(ublox_t *ub, uint8_t dynmodel)
{
  // These are the defaults except dynmodel (data[2])
  uint8_t data[] = {0xff, 0xff, dynmodel, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ublox_command(ub, 0x06, 0x24, 36, data);
}

// Enable or disable the UART ports (portID=1 or 2)
void configure_uart(ublox_t *ub, uint8_t portId, uint8_t enable)
{
    struct {
        uint8_t portId;
        uint8_t reserved0;
        uint16_t txReady;
        uint32_t mode;
        uint32_t baudRate;
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t reserved4;
        uint16_t reserved5;
    } data = {0};

    data.portId = portId;
    data.mode = 0x8D0; // 8 data bits, no parity, 1 stop bit
    data.baudRate = 115200;
    // enable/disable both NMEA and UBX protocols
    data.inProtoMask = data.outProtoMask = enable ? 0x3 : 0;
    ublox_command(ub, 0x06, 0x00, 20, (uint8_t*)&data);
}

int main (int argc, char *argv[])
{
    getopt_t *gopt;
    ublox_t *ub;

    setlinebuf(stdout);

    gopt = getopt_create();
    getopt_add_string(gopt, 'd', "device", "/dev/ttyACM0",
                      "GPS serial device");
    getopt_add_int(gopt, 'b', "baud", "115200", "Serial baud rate");
    getopt_add_int(gopt, 'h', "hz", "2", "Navigation update rate in Hz");
    getopt_add_bool(gopt, 'v', "verbose", 0, "Enable verbose output");
    getopt_add_bool(gopt, 'r', "raw", 1, "Enable raw output");

    if (!getopt_parse(gopt, argc, argv, 1)) {
        getopt_do_usage(gopt);
        return 1;
    }

    const char *port = getopt_get_string(gopt, "device");
    verbose = getopt_get_bool(gopt, "verbose");
    int hz = getopt_get_int(gopt, "hz");

    lcm = lcm_create(NULL);

    char device[PATH_MAX];
    if (realpath(port, device) == NULL) {
        printf("Error resolving real device path (e.g. following symlinks) for '%s'\n", port);
        return 1;
    }

    printf("Resolved '%s' to device '%s'\n", port, device);

    ub = ublox_create(device , getopt_get_int(gopt, "baud"));
    if (ub == NULL) {
        printf("Error opening device %s\n", device);
        return 1;
    }
    ublox_set_ubx_callback(ub, ubx_callback);
    ublox_set_nmea_callback(ub, nmea_callback);

    // Disable UART (port 1)
    configure_uart(ub, 1, 0);

    // Set nav engine settings to portable
    set_nav_engine(ub, 0);
    // Set nav engine settings to airborne 4g (less filtering)
    //set_nav_engine(ub, 8);
    ublox_command(ub, 0x06, 0x24, 0, NULL);

    // By default enabled: GGA, GLL, GSA, GSV, RMC, VTG, TXT
    // Disable GGA
    //set_msg_rate(ub, 0xF0, 0x00, 0);
    // Disable GLL
    //set_msg_rate(ub, 0xF0, 0x01, 0);
    // Disable VTG
    //set_msg_rate(ub, 0xF0, 0x05, 0);
    // Disable GSV
    //set_msg_rate(ub, 0xF0, 0x03, 0);
    // Disable ZDA
    //set_msg_rate(ub, 0xF0, 0x08, 0);
    // Disable GSA
    //set_msg_rate(ub, 0xF0, 0x02, 0);
    // Disable RMC
    //set_msg_rate(ub, 0xF0, 0x04, 0);

    // Enable UBX,00 (LatLong + Velocity)
    set_msg_rate(ub, 0xF1, 0x00, 1);

    // Enable binary protocol messages
    // Enable I/O debug status MON-TXBUF
    set_msg_rate(ub, 0x0A, 0x08, 1);

    // Enable NAV-DOP
    set_msg_rate(ub, 0x01, 0x04, 1);
    // Enable NAV-POSLLH
    set_msg_rate(ub, 0x01, 0x02, 1);
    // Enable NAV-SBAS
    set_msg_rate(ub, 0x01, 0x32, 1);
    // Enable NAV-SOL (includes POSECEF and VELECEF)
    set_msg_rate(ub, 0x01, 0x06, 1);
    // Enable NAV-SVINFO
    set_msg_rate(ub, 0x01, 0x30, 1);
    // Enable NAV-VELNED
    set_msg_rate(ub, 0x01, 0x12, 1);

    // Set measurement period
    // LEA-6T-0 up to 200ms (5Hz)
    // LEA-6T-1 up to 500ms (2Hz)
    set_measurement_rate(ub, 1000/hz);

    // Enable RXM-RAW messages
    uint8_t enableRaw = getopt_get_bool(gopt, "raw");
    if (enableRaw) {
        set_msg_rate(ub, 0x02, 0x10, 1);
        set_msg_rate(ub, 0x02, 0x11, 1);
    } else {
        set_msg_rate(ub, 0x02, 0x10, 0);
        set_msg_rate(ub, 0x02, 0x11, 0);
    }

    ublox_start(ub);

    while (1) {
        // Poll for all RXM-EPH
        if (enableRaw)
            ublox_command(ub, 0x02, 0x31, 0, NULL);
        sleep(3600);
    }

    ublox_destroy(ub);
}
