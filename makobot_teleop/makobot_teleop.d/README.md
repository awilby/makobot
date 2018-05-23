# Install

These scripts help run the makobot_teleop node as a Linux service on startup. 

To install:

```
sudo cp run_teleop.sh /usr/local/bin/run_teleop.sh
sudo cp makobot_teleop.service /etc/systemd/system/makobot_teleop.service
```

Enable service and start:

```
sudo systemctl enable makobot_teleop.service
sudo systemctl makobot_teleop start
```

To check status of service:

`
sudo systemctl makobot_teleop status
`
