include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_DGC) $(CFLAGS_GLIB) $(CFLAGS_LCM) $(CFLAGS_LCMTYPES) -D_GNU_SOURCE
LDFLAGS = $(LDFLAGS_DGC) $(LDFLAGS_COMMON) $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_LCM) $(LDFLAGS_LCMTYPES)

# BELOW FROM gps, above from sick

#LDFLAGS += -lpthread -lm
#LDFLAGS +=   `pkg-config --libs glib-2.0 gthread-2.0`

ublox:=$(BIN_PATH)/ublox

ublox_obj:=ublox.o main.o 

.PHONY: all test clean tidy

all: $(ublox)

$(ublox): $(ublox_obj)
	@echo "    [$@]"
	$(CC) -o $@ $(ublox_obj) $(LDFLAGS)

%.o: %.c
	@echo "    [$@]"
	$(CC) -c -o $@ $< $(CFLAGS)

test:
	@echo gps18 : nothing to test

clean: tidy
	rm -f $(ublox) $(gps_lcm_listen)

tidy:
	rm -f $(ublox_obj) $(gps_lcm_listen_obj)
