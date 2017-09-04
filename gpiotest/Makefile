gpiotest: Makefile main.cpp
	gcc -std=c++11 -o gpiotest main.cpp MPU9250.cpp quaternionFilters.cpp -lwiringPi -lpthread -lm -lrt -lcrypt

push:
	-git commit . -m -
	-git push 
	sshpass -p pi ssh pi@raspirover sudo mount -a
	sshpass -p pi ssh pi@raspirover 'cd raspirover && git pull'
	sshpass -p pi ssh pi@raspirover 'make -C raspirover/gpiotest'

run_gpiotest:
	sshpass -p pi ssh pi@raspirover 'cd raspirover/gpiotest && ./gpiotest'
	
