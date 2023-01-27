# pre-reqs
* d1-mini clone
* vs-code
* add pio extension [ctrl-shift-X]

# steps

1. setup project within pre-configured repo:

   ```pio project init -b d1_mini --ide vscode```

1. add config lines for port & speed once connected to the usb

1. give access to port: 

   ```sudo chmod a+rw /dev/ttyUSB0```

1. restart vs-code to pick up config changes and 'reveal' project tasks

1. upload