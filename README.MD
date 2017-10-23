# autelis-polyglot
Polyglot Node Server for Autelis Pool Control for Pentair Intellitouch/EasyTouch. Note this application requires that the Autelis Pool Control device be running the firmware v. 1.6.7.

Installation Instructions:

1. Download the Autelis Pool Control Node Server to the folder /config/node_servers/autelis-polyglot in your Polyglot installation.
2. Rename the config.yaml.tmp file to config.yaml (named with .tmp so subsequent downloads won't overwrite existing configuration).
3. Edit the config.yaml file to contain the following settings:
```
    controller:
        ipaddress: <required - IP address of Autelis Pool Control>
        password: <required - password for Autelis Pool Control>
        username: <required - user ID for Autelis Pool Control>
    configuration:
        pollinginterval: <optional - polling interval in seconds (defaults to 60)>
        ignoresolar: <optional - ignore Solar Heat settings (defaults to False)>
```
4. Restart Polyglot to pickup the Autelis Pool Control Node Server.
5. Add an Autelis Pool Control Node Server instance to Polyglot:
* Log into Polyglot manager website at http://(Polyglot IP address):8080.
* Click "Add Node Server".
* Select/enter the following options:
   * Node Server Type: Autelis Pool Control
   * Name: Autelis Pool Control (or whatever instance name you wish)
   * Node Server ID: Number of next available Node Server slot on ISY
* Click "ADD".
6. Select the newly added Autelis Pool Control Node Server from the left and follow the "Instructions" provided to connect to the Node Server from your ISY.
