#!/usr/bin/python
# Polglot Node Server for Pentair controller through Autelis Pool Control Interface

from polyglot.nodeserver_api import NodeServer, SimpleNodeServer, Node
from polyglot.nodeserver_api import PolyglotConnector
from polyglot.nodeserver_api import NS_API_VERSION

import autelisapi
import threading
import logging
import time
import xml.etree.ElementTree as xml
from functools import partial

_ISY_INDEX_UOM = 25 # Index UOM for custom states (must match editor/NLS in profile):
_ISY_TEMP_F_UOM = 17 # UOM for temperatures

# Node class for controller
class PoolController(Node):

    node_def_id = "CONTROLLER"

    def __init__(self, parent, address, name, runstate, opmode, freeze, waterSensor, solarSensor, airSensor, airTemp, solarTemp, **kwargs):
        self.childNodes = []
        super(PoolController, self).__init__(parent, address, name, **kwargs)
        self.set_driver("GV0", runstate, _ISY_INDEX_UOM, False)
        self.set_driver("GV1", opmode, _ISY_INDEX_UOM, False)
        self.set_driver("GV2", freeze, _ISY_INDEX_UOM, False)
        self.set_driver("GV3", waterSensor, _ISY_INDEX_UOM, False)
        self.set_driver("GV4", solarSensor, _ISY_INDEX_UOM, False)
        self.set_driver("GV5", airSensor, _ISY_INDEX_UOM, False)
        self.set_driver("CLITEMP", airTemp, _ISY_TEMP_F_UOM, False)
        self.set_driver("GV9", solarTemp, _ISY_TEMP_F_UOM, False)
        self.report_driver()

    # Override query to report driver values and child driver values
    def query(self, **kwargs):

        # update all nodes - don't report
        self.parent.update_node_states(False)

        # report drivers and drivers of all child nodes
        self.report_driver()
        for node in self.childNodes:
            node.report_driver()

        return True

    _drivers = {
        "GV0": [1, _ISY_INDEX_UOM, int, False],
        "GV1": [0, _ISY_INDEX_UOM, int, False],
        "GV2": [0, _ISY_INDEX_UOM, int, False],
        "GV3": [0, _ISY_INDEX_UOM, int, False],
        "GV4": [0, _ISY_INDEX_UOM, int, False],
        "GV5": [0, _ISY_INDEX_UOM, int, False],
        "CLITEMP": [0, _ISY_TEMP_F_UOM, int, False],
        "GV9": [0, _ISY_TEMP_F_UOM, int, False]
    }
    _commands = {"QUERY": query}

# Node class for equipment (pumps and aux relays)
class Equipment(Node):

    node_def_id = "EQUIPMENT"

    def __init__(self, parent, address, name, state, **kwargs):
        super(Equipment, self).__init__(parent, address, name, **kwargs)
        self.set_driver("ST", state, _ISY_INDEX_UOM)

    # Turn equipment ON - TCP connection monitoring will pick up status change
    def cmd_don(self, **kwargs):
        if self.parent.autelis.on(self.address):
            self.set_driver("ST", 1, _ISY_INDEX_UOM)
            return True
        else:
            self.logger.warning("Call to Pool Controller in DON command handler failed for node %s.", self.address)
            return False

    # Turn equipment OFF - TCP connection monitoring will pick up status change
    def cmd_dof(self, **kwargs):
        if self.parent.autelis.off(self.address):
            self.set_driver("ST", 0, _ISY_INDEX_UOM)
            return True
        else:
            self.logger.warning("Call to Pool Controller in DOF command handler failed for node %s.", self.address)
            return False

    # Run update function in parent before reporting driver values
    def query(self, **kwargs):
        self.parent.update_node_states(False)
        return self.report_driver()

    _drivers = {"ST": [0, _ISY_INDEX_UOM, int, False]}
    _commands = {
        "DON": cmd_don,
        "DOF": cmd_dof,
        "QUERY": query
    }

# Node class for temperature controls (pool heat, spa heat, etc.)
class TempControl(Node):

    node_def_id = "TEMP_CONTROL"

    def __init__(self, parent, address, name, state, mode, setPoint, currentTemp, **kwargs):
        super(TempControl, self).__init__(parent, address, name, **kwargs)
        self.set_driver("ST", state, _ISY_INDEX_UOM, False)
        self.set_driver("CLIMD", mode, _ISY_INDEX_UOM, False)
        self.set_driver("CLISPH", setPoint, _ISY_TEMP_F_UOM, False)
        self.set_driver("CLITEMP", currentTemp, _ISY_TEMP_F_UOM, False)
        self.report_driver()

     # Set heating mode - TCP connection monitoring will pick up status change
    def cmd_set_mode(self, value, **kwargs):

        # set the setpoint element
        if self.parent.autelis.set_heat_setting(self.address, value):
            self.set_driver("CLIMD", value, _ISY_INDEX_UOM)
            return True
        else:
            self.logger.warning("Call to Pool Controller in SET_MODE command handler failed for node %s.", self.address)
            return False

    # Set set point temperature - TCP connection monitoring will pick up status change
    def cmd_set_temp(self, value, **kwargs):

        # determine setpoint element to change based on the node address
        if self.address == "poolht":
            name = "poolsp"
        elif self.address == "spaht":
            name = "spasp"
        else:
            self.logger.warning("No setpoint for node %s - SET_TEMP command ignored.", self.address)
            return False

        # set the setpoint element
        if self.parent.autelis.set_temp(name, value):
            self.set_driver("CLISPH", value, _ISY_TEMP_F_UOM)
            return True
        else:
            self.logger.warning("Call to Pool Controller in SET_TEMP command handler failed for node %s.", self.address)
            return False

    # Run update function in parent before reporting driver values
    def query(self, **kwargs):
        self.parent.update_node_states(False)
        return self.report_driver()

    _drivers = {
        "ST": [0, _ISY_INDEX_UOM, int, False],
        "CLIMD": [0, _ISY_INDEX_UOM, int, True],
        "CLISPH": [0, _ISY_TEMP_F_UOM, int, True],
        "CLITEMP": [0, _ISY_TEMP_F_UOM, int, False]
    }
    _commands = {
        "SET_MODE": cmd_set_mode,
        "SET_TEMP": cmd_set_temp,
        "QUERY": query
    }

# NodeServer class
class AutelisNodeServer(SimpleNodeServer):

    def __init__(self, *args, **kwargs):
        self.autelis = None
        self.logger = None
        self.pollingInterval = 20
        self.ignoresolar = False
        self.lastPoll = 0
        self.tempUnits = "F"
        super(AutelisNodeServer, self).__init__(*args, **kwargs)

    # setup must be overriden
    def setup(self):

        # class definition of setup() just sets logger from poly object
        self.logger = self.poly.logger

        # get controller information from configuration file
        try:
            controllerSettings = self.poly.nodeserver_config["controller"]
            ip = controllerSettings["ipaddress"]
            username = controllerSettings["username"]
            password = controllerSettings["password"]
        except KeyError:
            self.logger.error("Missing controller settings in config.yaml file")
            raise

        # get polling intervals and configuration settings from configuration file
        if "configuration" in self.poly.nodeserver_config:
            config = self.poly.nodeserver_config["configuration"]
            if "pollinginterval" in config:
                self.pollingInterval = config["pollinginterval"]
            if "ignoresolar" in config:
                self.ignoresolar = config["ignoresolar"]

        # create a object for the autelis interface
        self.autelis = autelisapi.AutelisInterface(ip, username, password, self.logger)

        # setup the nodes from the autelis pool controller
        self.update_node_states(True) # Report driver values
        self.lastPoll = time.time()

        # save config in case new devices were added
        self.update_config(True)

    # called every long_poll seconds (default 30)
    def long_poll(self):

        # save the config
        self.update_config()

        return True

    # called every short_poll seconds (default 1)
    def poll(self):

        # if node server is not setup yet, return
        if self.autelis == None:
            return True

        currentTime = time.time()
        retVal = True

        # check for elapsed polling interval
        if (currentTime - self.lastPoll) >= self.pollingInterval:

            # update the node states
            self.logger.debug("Updating node states in AuteliseNodeServer.poll()...")
            retVal = self.update_node_states(True) # Update node states
            self.lastPoll = currentTime

        return retVal

    # Creates or updates the state values of all nodes from the autelis interface
    def update_node_states(self, reportDrivers=True):

        # load manifest from config file for creating nodes
        manifest = self.config.get("manifest", {})

        # get the status XML from the autelis device
        statusXML = self.autelis.get_status()

        if statusXML == None:
            self.logger.warning("No XML returned from get_status()")
            return False
        else:

            # Parse status XML
            system = statusXML.find("system")
            equipment = statusXML.find("equipment")
            temp = statusXML.find("temp")

            # Get processing elements
            self.tempUnits = temp.find("tempunits").text

            # Get the element values for the controller node
            runstate = int(system.find("runstate").text)
            opmode = int(system.find("opmode").text)
            freeze = int(system.find("freeze").text)
            waterSensor = int(system.find("sensor1").text)
            solarSensor = int(system.find("sensor2").text)
            airSensor = int(system.find("sensor3").text)
            airTemp = int(temp.find("airtemp").text)
            solarTemp = int(temp.find("soltemp").text)

            # Create the CONTROLLER node if it doesn't exist, otherwise update the node drivers
            if not self.exist_node("controller"):
                controllerNode = PoolController(
                    self,
                    "controller",
                    "controller",
                    runstate,
                    opmode,
                    freeze,
                    waterSensor,
                    solarSensor,
                    airSensor,
                    airTemp,
                    solarTemp,
                    primary=True,
                    manifest=manifest)
            else:
                controllerNode = self.get_node("controller")
                controllerNode.set_driver("GV0", runstate, _ISY_INDEX_UOM, reportDrivers)
                controllerNode.set_driver("GV1", opmode, _ISY_INDEX_UOM, reportDrivers)
                controllerNode.set_driver("GV2", freeze, _ISY_INDEX_UOM, reportDrivers)
                controllerNode.set_driver("GV3", waterSensor, _ISY_INDEX_UOM, reportDrivers)
                controllerNode.set_driver("GV4", solarSensor, _ISY_INDEX_UOM, reportDrivers)
                controllerNode.set_driver("GV5", airSensor, _ISY_INDEX_UOM, reportDrivers)
                controllerNode.set_driver("CLITEMP", airTemp, _ISY_TEMP_F_UOM, reportDrivers)
                controllerNode.set_driver("GV9", solarTemp, _ISY_TEMP_F_UOM, reportDrivers)

            # Process poolht temp control elements
            htstatus = int(temp.find("htstatus").text)
            addr = "poolht"
            if htstatus & int("0001", 2):
                state = 1
            elif htstatus & int("0100", 2):
                state = 2
            else:
                state = 0

            mode = int(temp.find("poolht").text)
            setPoint = int(temp.find("poolsp").text)
            currentTemp = int(temp.find("pooltemp").text)

            # Create the TEMP_CONTROL node if it doesn't exist, otherwise update the node drivers
            if not self.exist_node(addr):
                tempNode = TempControl(
                    self,
                    addr,
                    addr,
                    state,
                    mode,
                    setPoint,
                    currentTemp,
                    primary=controllerNode,
                    manifest=manifest
                )
                controllerNode.childNodes.append(tempNode)

            else:
                tempNode = self.get_node(addr)
                tempNode.set_driver("ST", state, _ISY_INDEX_UOM, reportDrivers)
                tempNode.set_driver("CLIMD", mode, _ISY_INDEX_UOM, reportDrivers)
                tempNode.set_driver("CLISPH", setPoint, _ISY_TEMP_F_UOM, reportDrivers)
                tempNode.set_driver("CLITEMP", currentTemp, _ISY_TEMP_F_UOM, reportDrivers)

            # Process spaht temp control elements
            addr = "spaht"
            if htstatus & int("0010", 2):
                state = 1
            elif htstatus & int("1000", 2):
                state = 2
            else:
                state = 0

            mode = int(temp.find("spaht").text)
            setPoint = int(temp.find("spasp").text)
            currentTemp = int(temp.find("spatemp").text)

            # Create the TEMP_CONTROL node if it doesn't exist, otherwise update the node drivers
            if not self.exist_node(addr):
                tempNode = TempControl(
                    self,
                    addr,
                    addr,
                    state,
                    mode,
                    setPoint,
                    currentTemp,
                    primary=controllerNode,
                    manifest=manifest
                )
                controllerNode.childNodes.append(tempNode)

            else:
                tempNode = self.get_node(addr)
                tempNode.set_driver("ST", state, _ISY_INDEX_UOM, reportDrivers)
                tempNode.set_driver("CLIMD", mode, _ISY_INDEX_UOM, reportDrivers)
                tempNode.set_driver("CLISPH", setPoint, _ISY_TEMP_F_UOM, reportDrivers)
                tempNode.set_driver("CLITEMP", currentTemp, _ISY_TEMP_F_UOM, reportDrivers)

            # Iterate equipment child elements and process each
            for element in list(equipment):

                # Only process elements that have text values (assuming blank
                # elements are part of the installed/configured equipment).
                # Also ignore solar heat if configuration flag is not set
                if not element.text == None:

                    addr = element.tag
                    state = int(element.text)

                    # Create the EQUIPMENT node if in setup, otherwise update the node drivers
                    if not self.exist_node(addr):
                        equipNode = Equipment(
                            self,
                            addr,
                            addr,
                            state,
                            primary=controllerNode,
                            manifest=manifest
                        )
                        controllerNode.childNodes.append(equipNode)

                    else:
                        equipNode = self.get_node(addr)
                        equipNode.set_driver("ST", state, _ISY_INDEX_UOM, reportDrivers)

        return True

# Main function to setup Polyglot connection
def main():
    poly = PolyglotConnector()
    nserver = AutelisNodeServer(poly)
    poly.connect()
    poly.wait_for_config()
    nserver.setup()
    nserver.run()

if __name__ == "__main__":
    main()
