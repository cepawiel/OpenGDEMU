#include "tb.h"

#include "Vour.h"
#include "verilated.h"

#include <string>
#include <optional>

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

	TB<Vour> *tb = new TB<Vour>(vcdfile.has_value());

    if (vcdfile) {
        tb->opentrace(vcdfile.value().c_str());
    }

	while(!tb->done()) {
		tb->tick();
	} 
    exit(EXIT_SUCCESS);
}