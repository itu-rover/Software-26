


### Create a new service

```bash
sudo cp cam@.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cam@0
sudo systemctl start cam@0
```

### Start a service

```bash
sudo systemctl enable --now cam@video4.service
```

### Stop a service

```bash
sudo systemctl stop cam@video4.service
```

