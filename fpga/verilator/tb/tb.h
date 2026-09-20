#pragma once

#include <verilated.h>
#include <verilated_vcd_c.h>

#include <string>
#include <optional>
#include <map>


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

    M* get_dut() {
        return dut;
    }

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

};

