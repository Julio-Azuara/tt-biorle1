# Makefile for the Tiny Tapeout cocotb test harness.
# See https://docs.cocotb.org/en/stable/quickstart.html for more info.
#
# The only change from the Tiny Tapeout template is PROJECT_SOURCES, which now
# lists the five BioRLE-1 RTL files instead of the placeholder project.v.
# PROJECT_SOURCES is used only for RTL simulation; gate-level (GATES=yes) uses
# the generated gate_level_netlist.v copied in by the GDS action.

# defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = tt_um_biorle1.v
PROJECT_SOURCES += biorle1_spi_rx.v
PROJECT_SOURCES += biorle1_delta.v
PROJECT_SOURCES += biorle1_rle.v
PROJECT_SOURCES += biorle1_out.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD = sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD = sim_build/gl
COMPILE_ARGS += -DGL_TEST
COMPILE_ARGS += -DFUNCTIONAL
COMPILE_ARGS += -DUSE_POWER_PINS
COMPILE_ARGS += -DSIM
COMPILE_ARGS += -DUNIT_DELAY=#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# COCOTB_TEST_MODULES is the basename of the python test file
COCOTB_TEST_MODULES = test

include $(shell cocotb-config --makefiles)/Makefile.sim
