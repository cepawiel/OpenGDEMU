// #include "tb.h"

#include "VRegister.h"
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

    void sim() {
        // Reset
        reset();
        assert(dut->o_data_out == 0x00);

        dut->i_data_in = 0xFF;
        tick();
        assert(dut->o_data_out == 0x00);

        dut->i_we = 1;
        tick();
        assert(dut->o_data_out == 0xFF);
        
        dut->i_we = 0;
        dut->i_data_in = 0x5A;
        assert(dut->o_data_out == 0xFF);

        dut->i_we = 1;
        tick();
        assert(dut->o_data_out == 0x5A);

        dut->i_we = 0;
        dut->i_data_in = 0xA5;
        assert(dut->o_data_out == 0x5A);

        dut->i_we = 1;
        tick();
        assert(dut->o_data_out == 0xA5);

        dut->i_we = 0;
        dut->i_data_in = 0xFF;
        assert(dut->o_data_out == 0xA5);

        reset();
        assert(dut->o_data_out == 0x00);

        tick();
    }
};

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

	TB<VRegister> *tb = new TB<VRegister>(vcdfile.has_value());

    if (vcdfile) {
        tb->opentrace(vcdfile.value().c_str());
    }

	tb->sim();
    exit(EXIT_SUCCESS);
}