import sys #for exit
import socket
import json
import time

class Serverclient:

  def __init__(self, host = '127.0.0.1', port = 1337):
    self.channel = None
    self.VERBOSE = True
    self.HOST = host
    self.PORT = port
    self.messageCallback = None
    self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.bufferlen = 1024 * 16
    self.received = None
    self._isConnected = False
    self.lastReconnectTime = time.perf_counter()
    self.reconnectTime = 15 #Seconds
    #self.buffer = new Buffer.alloc(1024 * 16);
    #self.buffer.len = 0
    
    print("Server Client Created!")

  def reconnect(self):
    if (not self.channel or not self.messageCallback):
      print(f"Reconnect - Channel: {self.channel} Callback: {self.messageCallback}")
      return False
    print("Server Client: attempt to reconnect...")
    return self.subscribe( self.channel, self.messageCallback)

  def subscribe(self, channel, callback):
    self.channel = channel
    self.messageCallback = callback
    #print (f"Channel: {self.channel}, Callback: {self.messageCallback}")
    try:
      self.client.connect((self.HOST, self.PORT))
      self.client.send(bytes("__SUBSCRIBE__" + self.channel + "__ENDSUBSCRIBE__", 'UTF-8'))
      self._isConnected = True
      #   data = {
      # "ssss": "string",
      # "testt": 10,
      # "info": "sample is test"
      # }
      # self.client.send(bytes(self.json.dumps(data), 'UTF-8'))

    except socket.error as e:
        print("Server Client SUBSCRIBE Send Socket Error: " + str(e))
        # set connection status and recreate socket  
        self._isConnected = False  
        #print( "connection lost... attempting reconnect" ) 
        try:
            self.client.connect( ( self.HOST, self.PORT ) )  
            self.client.send(bytes("__SUBSCRIBE__" + self.channel + "__ENDSUBSCRIBE__", 'UTF-8'))
            self._isConnected = True  
            print( "re-connection successful" )  
        except socket.error:
          print( "connection un-successful, lets try again soon" )
          self._isConnected = False

  def unsubscribe(self):
    if self._isConnected:
      self.client.send(bytes('Take care Server Client...', 'UTF-8'))
      self.client.shutdown(2)    # 0 = done receiving, 1 = done sending, 2 = both
      self.client.close()
      self._isConnected = False

  def publish(self, message):

    if not self._isConnected:
      if self.lastReconnectTime < (time.perf_counter() - self.reconnectTime):
        print("Server Client: Attempt to publish without valid subscription (bad socket)")
        self.reconnect()
        self.lastReconnectTime = time.perf_counter()
      return False

    if not isinstance(message, str):
      message = json.dumps(message, separators=(',', ':'))

    try:
      self.client.send(bytes("__JSON__START__" + message + "__JSON__END__", 'UTF-8'))
      reply = self.client.recv(self.bufferlen).decode()
      return reply
    except socket.error as e:
      print("Server Client PUBLISH Sned Socket Error: " + str(e))
      if 'WinError 10054' in str(e):
        self._isConnected = False
        #self.client.close()
        self.lastReconnectTime = time.perf_counter()
        # Need to recreate Socket
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.reconnect()
      return False


  # self._handleIncomingMessage = function(data) {
  #   self.buffer.len += data.copy(self.buffer, self.buffer.len);
  #   let start;
  #   let end;
  #   let str = self.buffer.slice(0, self.buffer.len).toString();

  #   if (
  #     (start = str.indexOf('__JSON__START__')) !== -1 &&
  #     (end = str.indexOf('__JSON__END__')) !== -1
  #   ) {
  #     var json = str.substr(start + 15, end - (start + 15));
  #     str = str.substr(end + 13); // cut the message and remove the precedant part of the buffer since it can't be processed
  #     self.buffer.len = self.buffer.write(str, 0);
  #     json = JSON.parse(json);
  #     if (typeof self.messageCallback === 'function') {
  #       self.messageCallback(json);
  #     }
  #   }
  # };
