include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_DGC) $(CFLAGS_GLIB) $(CFLAGS_LCM) $(CFLAGS_LCMTYPES) -D_GNU_SOURCE
LDFLAGS = $(LDFLAGS_DGC) $(LDFLAGS_COMMON) $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_LCM) $(LDFLAGS_LCMTYPES)

# BELOW FROM gps, above from sick

#LDFLAGS += -lpthread -lm
#LDFLAGS +=   `pkg-config --libs glib-2.0 gthread-2.0`

kvh:=$(BIN_PATH)/kvh

kvh_obj:=main.o 

.PHONY: all test clean tidy

all: $(kvh)

$(kvh): $(kvh_obj)
	@echo "    [$@]"
	$(CC) -o $@ $(kvh_obj) $(LDFLAGS)

%.o: %.c
	@echo "    [$@]"
	$(CC) -c -o $@ $< $(CFLAGS)

test:
	@echo gps18 : nothing to test

clean: tidy
	rm -f $(kvh) $(gps_lcm_listen)

tidy:
	rm -f $(kvh_obj) $(gps_lcm_listen_obj)
