
VERILATOR_OBJ_DIR=obj_dir

BIN_DIR=bin
TB_DIR=tb
HDL_DIR=hdl

CXX=g++
CXX_FLAGS=-Iobj_dir -I/usr/share/verilator/include
CXX_OBJS=/usr/share/verilator/include/verilated.cpp

TESTS=$(wildcard $(TB_DIR)/*.cpp)
TARGETS=$(notdir $(basename $(TESTS)))

.SECONDARY: 
# VERILATOR_DIR=$(patsubst %, $(BUILD_DIR)/%, $(TARGETS))

all: run
build: $(VERILATOR_FOLDERS)
run: $(TARGETS)
clean:
	@rm -rf $(VERILATOR_OBJ_DIR) $(BIN_DIR)
	@mkdir -p $(BIN_DIR)

# Verilator
$(BIN_DIR)/%: $(HDL_DIR)/%.v $(TB_DIR)/%.cpp 
	verilator -I$(HDL_DIR) -j 0 --trace -Wall -cc $(HDL_DIR)/$*.v --exe $(TB_DIR)/$*.cpp
	make -C $(VERILATOR_OBJ_DIR) -f V$*.mk V$* -j12
	mv $(VERILATOR_OBJ_DIR)/V$* $@

$(TARGETS): %: $(BIN_DIR)/%

$(BIN_DIR)/% $(BIN_DIR)/%.vcd: $(BIN_DIR)/%
	./$(BIN_DIR)/$* --vcd bin/$*.vcd

%_wave: $(BIN_DIR)/%.vcd
	@echo Output $<
#	gtkwave $<



