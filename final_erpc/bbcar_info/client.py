#
# Generated by erpcgen 1.9.0 on Thu May 26 15:38:53 2022.
#
# AUTOGENERATED - DO NOT EDIT
#

import erpc
from . import common, interface

# Client for BBCarService
class BBCarServiceClient(interface.IBBCarService):
    def __init__(self, manager):
        super(BBCarServiceClient, self).__init__()
        self._clientManager = manager

    def info(self):
        # Build remote function invocation message.
        request = self._clientManager.create_request()
        codec = request.codec
        codec.start_write_message(erpc.codec.MessageInfo(
                type=erpc.codec.MessageType.kInvocationMessage,
                service=self.SERVICE_ID,
                request=self.INFO_ID,
                sequence=request.sequence))

        # Send request and process reply.
        self._clientManager.perform_request(request)



