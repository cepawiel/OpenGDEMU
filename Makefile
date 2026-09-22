FPGA_DIR=$(shell pwd)/fpga

.PHONY: fpga_container fpga_shell

# No separate download rule for the 4.5 GB installer: the Containerfile takes
# it from the build context when it is there and fetches it when it is not,
# and it carries the browser header block Altera's CDN demands. A plain wget
# here just gets a 403.
#
# `-f` because docker/buildx only auto-detects a file named `Dockerfile`;
# podman would find the Containerfile on its own.
fpga_container:
	podman build -t quartus64 \
		-f Containers/QuartusII_x64/Containerfile Containers/QuartusII_x64

# An interactive Quartus shell. To actually build the bitstream use
# fpga/build_fpga.sh, which stages the sources and copies the reports back.
fpga_shell: fpga_container
	podman run --rm -it \
		-v $(FPGA_DIR):/fpga \
		-w /fpga \
		quartus64 /bin/bash
