# NodeMCU SPI Slave and master example.
### this is just for testing and experimentation purpose. Most likely you don't need this.

To run this project you need a NodeMCU.
and arduino sdk.
https://github.com/esp8266/Arduino

modify ESP_ROOT and COMP_PATH in Makefile.master and Makefile.slave to point where you arduino sdk and gcc xtensa toolchain is installed.
modify

install
https://github.com/plerup/makeEspArduino

and modify last last `include /c/esp/makescript/makeEspArduino.mk`
to point where makeEspArduino is installed.

then run
```sh
make -f Makefile.master # to build master
make -f Makefile.slave # to build slave
```