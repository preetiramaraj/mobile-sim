/* Driver for KVH DSP-1750 gyro */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "common/getopt.h"
#include "common/serial.h"
#include "common/timestamp.h"
#include "common/timesync.h"

#include "lcmtypes/kvh_t.h"
#include "lcm/lcm.h"

static bool verbose = false;
static lcm_t *lcm;
static const char *lcmchannel;
static int nsamples;
static int32_t *samples;
static timesync_t *ts;

enum {
    STARTUP,
    ACQUIRE1,
    ACQUIRE2,
    LOCKED
};

void print_time()
{
    int64_t hosttime = timestamp_now();
    printf("[%11lld.%06ld] ", (long long)(hosttime/1000000),
	   (long)(hosttime%1000000));
}

void data_received(uint8_t data[])
{
    static int32_t sum = 0;
    static int count = 0;
    static int64_t lastprint = 0LL;
    static int32_t invalid = 0;

    // Can sync even on invalid packets
    uint8_t sync = (data[0] >> 6) & 0x3;
    int64_t hosttime = timestamp_now();
    timesync_update(ts, hosttime, sync);

    bool valid = (data[0] & (1 << 4)) != 0;
    if (!valid)
	invalid += 1;

    if (hosttime - lastprint > 1000000) {
      printf("[%11lld.%06ld] kvh  invalid: %-10d resync: %d\n",
	     (long long)(hosttime/1000000),
	     (long)(hosttime%1000000),
	     invalid, ts->resync_count);
      lastprint = hosttime;
    }

    if (!valid)
        return;

    int32_t rate = (data[3] & 0x3F) << 16 |
        data[4] << 8 |
        data[5];

    // sign extend the angular rate info (rate is sent as a 22-bit int)
    if (rate & 0x200000L)
        rate |= 0xFFC00000L;

    // Fill the buffer
    samples[count++] = rate;
    sum += rate;

    // Publish LCM message every n samples
    if (count == nsamples) {
        double avg_rate = (double)sum / nsamples;
        kvh_t data;

	data.utime = timesync_get_host_utime(ts, sync);
        data.rads = avg_rate * 476.8e-6 * M_PI / 180.0;
        data.nsamples = nsamples;
        data.samples = samples;
        kvh_t_publish(lcm, lcmchannel, &data);

        count = 0;
        sum = 0;
    }
}

void read_loop(FILE *in)
{
    int c;
    int state = STARTUP;
    uint8_t lastsync;
    uint8_t data[6];

    // Temporary variables for detecting frame sync
    uint8_t counts[6];
    uint8_t last[6];
    int i;

    while((c = fgetc(in)) != EOF) {
        switch(state) {
        case STARTUP:
            // Setup variables for acquiring frame sync
            for (i = 0; i < 6; i += 1)
                counts[i] = 0;
            i = 0;
            state = ACQUIRE1;
            break;

        case ACQUIRE1:
            // First collect one cycle (6 bytes) of data
            last[i++] = ((uint8_t)c) >> 6;
            if (i == 6) {
                i = 0;
                state = ACQUIRE2;
            }
            break;

        case ACQUIRE2: {
            // Collect statistics about each possible byte position
            uint8_t sync = ((uint8_t)c) >> 6;

            // Make sure sync bits are counting up (0, 1, 2, 3)
            if (sync == (last[i] + 1) % 4) {
                counts[i] += 1;

                // Received 10 consecutive sync bits in order,
                // so we're /pretty sure/ this is our start byte
                if (counts[i] >= 10) {
                    state = LOCKED;
                    data[0] = c;
                    i = 1;
                    lastsync = sync;
                    break;
                }
            } else {
                counts[i] = 0;
            }

            last[i] = sync;
            i = (i+1) % 6;

	    // For debugging resync
	    if (verbose && i == 0) {
		int j;
		printf("counts: ");
		for (j = 0; j < 6; j +=1)
		    printf("%d ", counts[j]);
		printf("\n");
	    }

            break;
        }

        case LOCKED:
            data[i] = c;
            if (i == 0) {
                uint8_t sync = ((uint8_t)c) >> 6;
                // Check sync bits
                if (sync != (lastsync + 1) % 4) {
                    state = STARTUP;
		    print_time();
                    printf("Lost sync. Resynchronizing\n");
                    break;
                }
                lastsync = sync;
            } else if (i == 5) {
                data_received(data);
            }
            i = (i+1) % 6;
            break;
        }
    }
}

int main (int argc, char* argv[])
{
    getopt_t *gopt;

    // Parse command line options
    gopt = getopt_create();
    getopt_add_string(gopt, 'd', "device", "/dev/ttyUSB0", "Serial device");
    getopt_add_string(gopt, 'c', "channel", "KVH", "LCM channel name");
    getopt_add_int(gopt, 'n', "nsamples", "6", "Samples per LCM message");
    getopt_add_bool(gopt, 'v', "verbose", 0, "Print all raw readings");
    if (!getopt_parse(gopt, argc, argv, 1)) {
        getopt_do_usage(gopt);
        return 1;
    }
    verbose = getopt_get_bool(gopt, "verbose");
    lcmchannel = getopt_get_string(gopt, "channel");
    nsamples = getopt_get_int(gopt, "nsamples");
    samples = (int32_t*)malloc(nsamples*sizeof(int32_t));

    const char *port = getopt_get_string(gopt, "device");
    int fd = serial_open(port, 115200, 1);
    if (fd <= 0) {
        printf("Error opening port %s\n", port);
        return 1;
    }
    // Wrap fd in a stream for buffering
    FILE* in = fdopen(fd, "r");

    lcm = lcm_create(NULL);

    // Rough time sync using the 2-bit sync pattern
    // 989Hz, roll over at 4 ticks, 5% rate error, 
    // resynchronize if error exceeds 100ms
    ts = timesync_create(989, 4, 0.05, 0.1);

    // Read bytes indefinitely
    read_loop(in);

    fclose(in);
}
