# Auto-starting Robot Control on Raspberry Pi Boot

This guide provides multiple methods to automatically start the robot control application when your Raspberry Pi boots up.

## Method 1: Using systemd (Recommended)

Systemd is the modern init system for Linux and provides the most reliable way to autostart services.

1. Create a systemd service file:

```bash
sudo nano /etc/systemd/system/robot-control.service
```

2. Add the following content (adjust paths as needed):

```
[Unit]
Description=5DOF Robot Control Service
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/5DOF_SampleCode
ExecStart=/usr/bin/python3 /home/pi/5DOF_SampleCode/robot_control.py
Restart=on-failure
RestartSec=5
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=robot-control

[Install]
WantedBy=multi-user.target
```

3. Save and exit (Ctrl+X, then Y, then Enter)

4. Enable and start the service:

```bash
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service
```

5. Check status:

```bash
sudo systemctl status robot-control.service
```

## Method 2: Using rc.local

A simpler but less robust method is to use the `/etc/rc.local` file:

1. Edit rc.local:

```bash
sudo nano /etc/rc.local
```

2. Add this line before the `exit 0` line:

```bash
python3 /home/pi/5DOF_SampleCode/robot_control.py &
```

3. Make sure rc.local is executable:

```bash
sudo chmod +x /etc/rc.local
```

## Method 3: Using crontab

You can also use cron to start the script at boot:

1. Edit the crontab:

```bash
crontab -e
```

2. Add this line:

```
@reboot python3 /home/pi/5DOF_SampleCode/robot_control.py &
```

## Troubleshooting

If the app doesn't start automatically:

1. Check logs with:
   ```bash
   sudo journalctl -u robot-control.service
   ```

2. Make sure your script path is correct in the configuration files.

3. Ensure the Python script has executable permissions:
   ```bash
   chmod +x /home/pi/5DOF_SampleCode/robot_control.py
   ```

4. Check for dependencies. Make sure all required Python packages are installed:
   ```bash
   pip3 install flask pyserial pygame
   ```

5. Create a test script to verify the environment:
   ```bash
   nano ~/test_robot.py
   ```
   
   Add content:
   ```python
   import os
   import time
   
   with open('/home/pi/robot_boot_test.log', 'a') as f:
       f.write(f'Script ran at: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
   ```
   
   Then set this test script to run at boot using one of the methods above. 