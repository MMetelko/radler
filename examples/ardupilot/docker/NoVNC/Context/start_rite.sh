#!/bin/bash

sleep 2
rm -rf ~riteuser/workspace-RITE
tar xmf /root/Downloads/workspace-RITE.tgz -C ~riteuser
chown -R riteuser:riteuser ~riteuser

exec su - riteuser -P -c "export DISPLAY=:0; cd /opt/RITE; exec /opt/RITE/RITE -data ~riteuser/workspace-RITE"
