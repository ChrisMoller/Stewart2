BOARDNAME = arduino:samd:mkrwifi1010
     PORT = /dev/tty`ls -l /dev/serial/by-id | grep Arduino | sed -e's/.*tty//'`
#     PORT=/dev/ttyACM0
#    PORT=/dev/ttyS4


compile:
	arduino-cli compile --upload -p  $(PORT) --fqbn $(BOARDNAME) . 

mon:
	arduino-cli monitor -p  $(PORT) --discovery-timeout 30s

upload: compile
	arduino-cli upload -v -p $(PORT) --fqbn $(BOARDNAME) .

ju:
	arduino-cli upload -v -p $(PORT) --fqbn $(BOARDNAME) --log-level trace


preproc:
	arduino-cli compile --preprocess --fqbn $(BOARDNAME) .

jc:
	arduino-cli compile --fqbn $(BOARDNAME) .

debug:
	arduino-cli debug --fqbn $(BOARDNAME) .

vis: vis.cpp
	gcc -g -o $@ $<

uv:
	arduino-cli upload -v -p $(PORT) --fqbn $(BOARDNAME) . \
           && ./vis/vis $(PORT)

