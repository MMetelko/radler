#!/usr/bin/env python3
# mavproxy_command_module.py

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil

class MavproxyCommandModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MavproxyCommandModule, self).__init__(mpstate, "commandmodule", "Command Executor Module")
        self.add_command('exec', self.cmd_exec, "Execute MAVProxy command", ['<command>'])

    def cmd_exec(self, args):
        command = ' '.join(args)
        self.mpstate.functions.process_stdin(command + '\n')
        self.console.write(f"Executed command: {command}\n")

def init(mpstate):
    return MavproxyCommandModule(mpstate)