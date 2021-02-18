wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARM64.tar.gz 
tar xzf arduino-cli_latest_Linux_ARM64.tar.gz
rm arduino-cli_latest_Linux_ARM64.tar.gz

./arduino-cli core install arduino:avr
./arduino-cli lib install "Adafruit Motor Shield library"
./arduino-cli lib install "PID"
./arduino-cli lib install "AutoPID"

