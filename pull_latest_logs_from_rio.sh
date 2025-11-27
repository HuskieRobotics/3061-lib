ssh lvuser@10.30.61.2 "ls -t /U/*.wpilog | head -n 4 | tar -cf /U/logdump.tar.gz -T -"
scp lvuser@10.30.61.2:/U/logdump.tar.gz ~
tar -vxf ~/logdump.tar.gz
