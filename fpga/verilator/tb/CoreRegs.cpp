// #include "tb.h"

#include "VCoreRegs.h"
#include "verilated.h"
#include <verilated_vcd_c.h>

#include <string>
#include <optional>

template<class M> class TB {
    VerilatedVcdC* trace;
    M* dut;
    int clk_cnt = 0;

public:
    TB(bool trace_en) {
        Verilated::traceEverOn(trace_en);
        dut = new M;
        clk_cnt = 0;
        trace = nullptr;
    }

    virtual ~TB() {
        delete dut;
        dut = nullptr;
    }

    // M* get_dut() {
    //     return dut;
    // }

    virtual	void opentrace(const char *vcdname) {
		if (!trace) {
			trace = new VerilatedVcdC;
			dut->trace(trace, 99);
			trace->open(vcdname);
		}
	}

    virtual void close() {
        if (trace) {
            trace->close();
            trace = nullptr;
        }
    }

    virtual void reset() {
        dut->i_reset = 1;
        this->tick();
        dut->i_reset = 0;
    }

    virtual void tick() {
        clk_cnt++;

        dut->i_clk = 0;
        dut->eval();
		if(trace) {
            trace->dump(10*clk_cnt-2);
        }

        dut->i_clk = 1;
        dut->eval();
		if(trace) {
            trace->dump(10*clk_cnt);
        }

        dut->i_clk = 0;
        dut->eval();
        if (trace) {
            trace->dump(10*clk_cnt+5);
            trace->flush();
        }
    }

    virtual bool done() { return (Verilated::gotFinish()); }

    void test_mcu_xfer() {
        // Reset
        reset();
        // assert(dut->o_mcu_data_out == 0xFFFF);
        // assert(dut->o_mcu_irq == 0);

        dut->i_mcu_data_in = 0x00;
        tick();
        // assert(dut->o_mcu_data_out == 0xFFFF);

        dut->i_mcu_cs = 1;
        tick();
        // assert(dut->o_mcu_data_out == 0xFFFF);

        dut->i_mcu_wr = 1;
        tick();
        // assert(dut->o_mcu_data_out == 0xFFFF);

        dut->i_mcu_wr = 0;
        dut->i_mcu_bs = 3;
        tick();
        // assert(dut->o_mcu_data_out == 0xFFFF);

        dut->i_mcu_wr = 1;
        dut->i_mcu_bs = 1;
        tick();
        printf("o_mcu_data_out = %04X\n", dut->o_mcu_data_out);
        // assert(dut->o_mcu_data_out == 0x00FF);

        dut->i_mcu_wr = 1;
        dut->i_mcu_bs = 2;
        tick();
        // assert(dut->o_mcu_data_out == 0xFFFF);


        // Check Default Test Reg Value
        
        dut->i_mcu_data_in = 0xC0DE;
        tick();
        tick();
        dut->i_mcu_cs = 1;
        dut->i_mcu_wr = 1;
        tick();
        tick();
        dut->i_mcu_bs = 3;
        tick();
        dut->i_mcu_cs = 0;
        dut->i_mcu_wr = 0;
        dut->i_mcu_bs = 0;
        tick();
    }

    void mcu_write(uint8_t addr, uint16_t data) {

    }
};

void sim(TB<VCoreRegs> *tb) {
    tb->reset();
    // dut->i_reset

    tb->test_mcu_xfer();



}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    std::optional<std::string> vcdfile = std::nullopt;

    for(int i = 0; i < argc; i++) {
        printf("argv[%d] = %s\n", i, argv[i]);
        if (strcmp(argv[i], "--vcd") == 0) {
            i++;
            printf("Writing VCD to %s\n", argv[i]);
            vcdfile = std::string(argv[i]);
        }
    }

	TB<VCoreRegs> *tb = new TB<VCoreRegs>(vcdfile.has_value());

    if (vcdfile) {
        tb->opentrace(vcdfile.value().c_str());
    }

	sim(tb);
    exit(EXIT_SUCCESS);
}