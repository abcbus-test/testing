#!/bin/sh
HOST='192.168.1.109'
USER='root'
PASSWD='root'
FILE='./MoxaGateway'

ftp -n $HOST <<END_SCRIPT
quote USER $USER
quote PASS $PASSWD
cd /home/Moxa/
put $FILE
sleep 3
quit
END_SCRIPT
exit 0
