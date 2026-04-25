##############################################################################
# @author: GaoKong
# @date:   15/09/2017
##############################################################################

TOOL_DIR =
MODULE	 = ak-flash

# Firmware path and device
FW_PATH  = firmware_test/ak-base-kit-stm32l151-application.bin
dev = /dev/ttyUSB0
APP_START_ADDR_VAL = 0x08003000

# USB rules
USB_RULES_FILE = usb_rules/uart_usb_conf.rules

OPTIMIZE = -g -Os
CXX	 = g++
CC	 = gcc
OBJ_DIR	 = build_$(MODULE)

CXXFLAGS += -I/usr/local/include
CXXFLAGS += -I/usr/include

CXXFLAGS += -Isources

VPATH += sources

OBJ += $(OBJ_DIR)/uart_boot.o
OBJ += $(OBJ_DIR)/firmware.o
OBJ += $(OBJ_DIR)/app_dbg.o

# CXX compiler option
CXXFLAGS	+=$(OPTIMIZE)		\
		-std=c++11		\
		-Wall			\
		-Winline		\
		-Wno-unused-result	\
		-pipe			\
		-g			\

# Library paths
LDFLAGS	+= -L/usr/local/lib
LDFLAGS	+= -L/usr/include
LDFLAGS	+= -Wl,-Map=$(OBJ_DIR)/$(MODULE).map

#Library libs
LDLIBS	+=	-lpthread		\
		-lrt			\

all: create $(OBJ_DIR)/$(MODULE)

create:
	@echo mkdir -p $(OBJ_DIR)
	@mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: %.cpp
	@echo CXX $<
	@$(CXX) -c -o $@ $< $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.c
	@echo CXX $<
	@$(CC) -c -o $@ $< $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/$(MODULE): $(OBJ)
	@echo ---------- START LINK PROJECT ----------
	@echo $(CXX) -o $@ $^ $(CXXFLAGS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)

flash:
	@echo $(OBJ_DIR)/$(MODULE) $(dev) $(FW_PATH) $(APP_START_ADDR_VAL)
	@echo Flashing firmware to device $(dev) at address $(APP_START_ADDR_VAL)
	@$(OBJ_DIR)/$(MODULE) $(dev) $(FW_PATH) $(APP_START_ADDR_VAL)

install: usb-rules
	@echo "Installing $(MODULE) to /usr/local/bin/..."
	@cp $(OBJ_DIR)/$(MODULE) /usr/local/bin/

usb-rules: 
	@echo "Installing USB rules for UART device..."
	@cp $(USB_RULES_FILE) /etc/udev/rules.d/
	@udevadm control --reload-rules
	@udevadm trigger

clean:
	@echo rm -rf $(OBJ_DIR)
	@rm -rf $(OBJ_DIR)
