CC ?= gcc

OVNGD_BIN = ovngd
OVNGD_SRC = main.c cfgfile.c eeprom.c ovng_iio.c iio_utils.c ovng_ow.c nmea_server.c baro.c filter.c ahrs.c complementary_filter.c
OVNGD_LIBS = -lm -lpthread

OVNGD_CONF = ovngd.conf

CFLAGS += -Wall
CFLAGS += -D_GNU_SOURCE

CFLAGS += -I.

REMOVE = rm -f

# Define all object files.
OVNGD_OBJ = $(OVNGD_SRC:.c=.o)

# Compiler flags to generate dependency files.
CFLAGS += -MD -MP -MF .dep/$(@F).d

all: $(OVNGD_BIN)

clean:
	$(REMOVE) -r .dep
	$(REMOVE) *~
	$(REMOVE) *.o
	$(REMOVE) $(OVNGD_BIN)


%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@ 

$(OVNGD_BIN): $(OVNGD_OBJ)
	$(CC) $(OVNGD_OBJ) -o $@ $(LDFLAGS) $(OVNGD_LIBS)

install: $(OVNGD_BIN) $(OVNGD_CONF)
	mkdir -p $(TARGET_DIR)/usr/bin
	cp $(OVNGD_BIN) $(TARGET_DIR)/usr/bin
	mkdir -p $(TARGET_DIR)/etc
	cp $(OVNGD_CONF) $(TARGET_DIR)/etc

# Include the dependency files.
-include $(shell mkdir -p .dep) $(wildcard .dep/*)

.PHONY: all clean install

