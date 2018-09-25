# Client Application
## P5js applicaiton to draw a radar graph in the browser

The P5js code is available in sketch.js file
To learn more about P5js see the following link:
https://p5js.org/reference/

The client application is connected to the webserver via a websocket.
A partial code for the websocket is the follwoing:
```javascript
socket = new WebSocket('ws://' + host+'/ws');
// socket message listener:
socket.onmessage = readMessage;

function readMessage(event) {

  var msg = JSON.parse(event.data);
  console.log(event);
  serialEventHandler(msg.value);
}
```