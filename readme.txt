This is an Arduino project for connection of multiple Arduino nodes and an
Arduino node with a host development machine.

State machine based data-exchange protocols over can-bus and serial-bus are
designed and implemented for connection of Arduinos and a host.

Further, this Arduino project implements oversampling of a 10-bits ADC input to
14-bits for increasing an ADC channel's resolution.

For hardware setup, each Arduino node is connected with a 3-channels analog
output gyroscope device for the ADC testing.

Can-bus state machine protocol is designed to connect up to 24 Arduino nodes at
run-time.
At a time one Arduino node is selected as a master to synchronise with other 23
Arduino nodes.
This enables tranfer of 3-channels 14-bits ADC output data between all connected
Arduino nodes.

Serial-bus state machine protocol is designed to receive control commands from
a host development machine on an Arduino node for the calibration of ADC input
channels and display of ADC output data on a host machine.